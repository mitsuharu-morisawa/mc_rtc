/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is modified mainly force distribution 
 * based on Kajita, et al IROS'10
 */

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/DCMStabilizerTask.h>

#include <chrono>

namespace mc_tasks
{
namespace lipm_stabilizer
{

using internal::Contact;
using ::mc_filter::utils::clamp;
using ::mc_filter::utils::clampInPlaceAndWarn;
namespace constants = ::mc_rtc::constants;

DCMStabilizerTask::DCMStabilizerTask(const mc_rbdyn::Robots & robots,
                                       const mc_rbdyn::Robots & realRobots,
                                       unsigned int robotIndex,
                                       const std::string & leftSurface,
                                       const std::string & rightSurface,
                                       const std::string & torsoBodyName,
                                       double dt)
: robots_(robots), realRobots_(realRobots), robotIndex_(robotIndex), dcmEstimator_(dt),
  extWrenchSumLowPass_(dt, /* cutoffPeriod = */ 0.05), comOffsetLowPass_(dt, /* cutoffPeriod = */ 0.05),
  comOffsetLowPassCoM_(dt, /* cutoffPeriod = */ 1.0), comOffsetDerivator_(dt, /* timeConstant = */ 1.),
  mass_(robots.robot(robotIndex).mass())
{
  type_ = "dcm_stabilizer";
  name_ = type_ + "_" + robots.robot(robotIndex).name();
  
  spm_ = std::make_shared<computational_geometry::SupportPolygonManager<ContactState> >();
  
  comTask.reset(new mc_tasks::CoMTask(robots, robotIndex_));
  auto leftCoP = std::allocate_shared<mc_tasks::force::CoPTask>(Eigen::aligned_allocator<mc_tasks::force::CoPTask>{},
                                                                leftSurface, robots, robotIndex_);
  auto rightCoP = std::allocate_shared<mc_tasks::force::CoPTask>(Eigen::aligned_allocator<mc_tasks::force::CoPTask>{},
                                                                 rightSurface, robots, robotIndex_);
  footTasks[ContactState::Left] = leftCoP;
  footTasks[ContactState::Right] = rightCoP;
  
  std::string pelvisBodyName = robot().mb().body(0).name();
  pelvisTask = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots_, robotIndex_);
  torsoTask = std::make_shared<mc_tasks::OrientationTask>(torsoBodyName, robots_, robotIndex_);
  
  // Rename the tasks managed by the stabilizer
  // Doing so helps making the logs more consistent, and having a fixed name
  // allows for predifined custom plots in the log ui.
  const auto n = name_ + "_Tasks";
  comTask->name(n + "_com");
  leftCoP->name(n + "_cop_left");
  rightCoP->name(n + "_cop_right");
  pelvisTask->name(n + "_pelvis");
  torsoTask->name(n + "_torso");
}
  
DCMStabilizerTask::DCMStabilizerTask(const mc_rbdyn::Robots & robots,
                                     const mc_rbdyn::Robots & realRobots,
                                     unsigned int robotIndex,
                                     double dt)
: DCMStabilizerTask(robots,
                    realRobots,
                    robotIndex,
                    robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().leftFootSurface,
                    robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().rightFootSurface,
                    robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().torsoBodyName,
                    dt)
{
  setContacts({ContactState::Left, ContactState::Right});
  reset();
}

void DCMStabilizerTask::reset()
{
  t_ = 0;
  comTask->reset();
  comTarget_ = comTask->com();
  comTargetRaw_ = comTarget_;
  zmpTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), 0.};
  zmpdTarget_ = Eigen::Vector3d::Zero();

  for(auto footTask : footTasks)
  {
    footTask.second->reset();
  }

  pelvisTask->reset();
  torsoTask->reset();
  
  measuredCoMd_pre_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  desiredWrench_ = sva::ForceVecd::Zero();
  distribWrench_ = sva::ForceVecd::Zero();
  vdcHeightError_ = 0.;
  c_.dcmErrorSum = Eigen::Vector3d::Zero();
  
  zmpcc_.reset();

  dcmEstimatorNeedsReset_ = true;

  extWrenches_.clear();
  extWrenchSumTarget_ = sva::ForceVecd::Zero();
  extWrenchSumMeasured_ = sva::ForceVecd::Zero();
  comOffsetTarget_ = Eigen::Vector3d::Zero();
  comOffsetMeasured_ = Eigen::Vector3d::Zero();
  comOffsetErr_ = Eigen::Vector3d::Zero();
  comOffsetErrCoM_ = Eigen::Vector3d::Zero();
  comOffsetErrZMP_ = Eigen::Vector3d::Zero();
  extWrenchSumLowPass_.reset(sva::ForceVecd::Zero());
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());
  
  omega_ = std::sqrt(constants::gravity.z() / robot().com().z());
  commitConfig();
}

void DCMStabilizerTask::dimWeight(const Eigen::VectorXd & /* dim */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("dimWeight not implemented for task {}", type_);
}

Eigen::VectorXd DCMStabilizerTask::dimWeight() const
{
  mc_rtc::log::error_and_throw<std::runtime_error>("dimWeight not implemented for task {}", type_);
}

void DCMStabilizerTask::selectActiveJoints(mc_solver::QPSolver & /* solver */,
                                        const std::vector<std::string> & /* activeJointsName */,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & /* activeDofs */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("Task {} does not implement selectActiveJoints. Please configure it "
                                                   "through the stabilizer configuration instead",
                                                   name_);
}

void DCMStabilizerTask::selectUnactiveJoints(
    mc_solver::QPSolver & /* solver */,
    const std::vector<std::string> & /* unactiveJointsName */,
    const std::map<std::string, std::vector<std::array<int, 2>>> & /* unactiveDofs */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "Task {} does not implement selectUnactiveJoints. Please configure it "
      "through the stabilizer configuration instead.",
      name_);
}

void DCMStabilizerTask::resetJointsSelector(mc_solver::QPSolver & /* solver */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "Task {} does not implement resetJointsSelector. Please configure it "
      "through the stabilizer configuration instead.",
      name_);
}

Eigen::VectorXd DCMStabilizerTask::eval() const
{
  Eigen::VectorXd res(3 + 3 * contactTasks.size());
  res.head(3) = comTask->eval();
  int i = 0;
  for(const auto & task : contactTasks)
  {
    res.segment(3 + 3 * i++, 3) = task->eval();
  }
  return res;
}

Eigen::VectorXd DCMStabilizerTask::speed() const
{
  Eigen::VectorXd res(3 + 3 * contactTasks.size());
  res.head(3) = comTask->speed();
  int i = 0;
  for(const auto & task : contactTasks)
  {
    res.segment(3 + 3 * i++, 3) = task->speed();
  }
  return res;
}

void DCMStabilizerTask::addToSolver(mc_solver::QPSolver & solver)
{
  // Feet tasks are added in update() instead, add all other tasks now
  MetaTask::addToSolver(*comTask, solver);
  MetaTask::addToSolver(*pelvisTask, solver);
  MetaTask::addToSolver(*torsoTask, solver);
}

void DCMStabilizerTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*comTask, solver);
  MetaTask::removeFromSolver(*pelvisTask, solver);
  MetaTask::removeFromSolver(*torsoTask, solver);
  for(const auto footTask : contactTasks)
  {
    MetaTask::removeFromSolver(*footTask, solver);
  }
}

void DCMStabilizerTask::updateContacts(mc_solver::QPSolver & solver)
{
  if(!addContacts_.empty())
  {
    // Remove previous contacts
    for(const auto & contactT : contactTasks)
    {
      mc_rtc::log::info("{}: Removing contact {}", name(), contactT->surface());
      MetaTask::removeFromLogger(*contactT, *solver.logger());
      MetaTask::removeFromSolver(*contactT, solver);
    }
    contactTasks.clear();
    contactSensors.clear();

    // Add new contacts
    for(const auto contactState : addContacts_)
    {
      auto footTask = footTasks[contactState];
      mc_rtc::log::info("{}: Adding contact {}", name(), footTask->surface());
      MetaTask::addToSolver(*footTask, solver);
      MetaTask::addToLogger(*footTask, *solver.logger());
      contactTasks.push_back(footTask);
      const auto & fs = robot().indirectSurfaceForceSensor(footTask->surface());
      contactSensors.push_back(fs.name());
    }
    addContacts_.clear();
  }
}

void DCMStabilizerTask::update(mc_solver::QPSolver & solver)
{
  // Prevent configuration changes while the stabilizer is disabled
  if(!enabled_)
  {
    c_ = disableConfig_;
    zmpcc_.configure(c_.zmpcc);
  }
  if(reconfigure_) configure_(solver);

  // Update contacts if they have changed
  updateContacts(solver);

  updateState(realRobots_.robot().com(), realRobots_.robot().comVelocity());

  // Run stabilizer
  run();

  MetaTask::update(*comTask, solver);
  MetaTask::update(*pelvisTask, solver);
  MetaTask::update(*torsoTask, solver);
  for(const auto footTask : contactTasks)
  {
    MetaTask::update(*footTask, solver);
  }

  t_ += dt_;
}

void DCMStabilizerTask::enable()
{
  mc_rtc::log::info("[DCMStabilizerTask] enabled");
  // Reset DCM integrator when enabling the stabilizer.
  // While idle, it will accumulate a lot of error, and would case the robot to
  // move suddently to compensate it otherwise
  
  extWrenchSumLowPass_.reset(sva::ForceVecd::Zero());
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());

  configure(lastConfig_);
  zmpcc_.enabled(true);
  enabled_ = true;
}

void DCMStabilizerTask::disable()
{
  mc_rtc::log::info("[DCMStabilizerTask] disabled");
  // Save current configuration to be reused when re-enabling
  lastConfig_ = c_;
  disableConfig_ = c_;
  // Set the stabilizer gains to zero
  disableConfig_.copAdmittance.setZero();
  disableConfig_.dcmDerivGain = 0.;
  disableConfig_.dcmIntegralGain = 0.;
  disableConfig_.dcmPropGain = 0.;
  disableConfig_.dfzAdmittance = 0.;
  disableConfig_.vdcFrequency = 0.;
  disableConfig_.vdcStiffness = 0.;
  zmpcc_.enabled(false);
  enabled_ = false;
}

void DCMStabilizerTask::reconfigure()
{
  mc_rtc::log::info("[DCMStabilizerTask] reconfigured to the last commited configuration");
  configure(defaultConfig_);
  enable();
}

void DCMStabilizerTask::configure(const DCMStabilizerConfiguration & config)
{
  checkConfiguration(config);
  lastConfig_ = config;
  disableConfig_ = config;
  c_ = config;
  c_.clampGains();
  reconfigure_ = true;
}

void DCMStabilizerTask::commitConfig()
{
  defaultConfig_ = c_;
}

void DCMStabilizerTask::configure_(mc_solver::QPSolver & solver)
{
  extWrenchSumLowPass_.cutoffPeriod(c_.extWrench.extWrenchSumLowPassCutoffPeriod);
  comOffsetLowPass_.cutoffPeriod(c_.extWrench.comOffsetLowPassCutoffPeriod);
  comOffsetLowPassCoM_.cutoffPeriod(c_.extWrench.comOffsetLowPassCoMCutoffPeriod);
  comOffsetDerivator_.timeConstant(c_.extWrench.comOffsetDerivatorTimeConstant);

  // // Configure upper-body tasks
  pelvisTask->stiffness(c_.pelvisStiffness);
  pelvisTask->weight(c_.pelvisWeight);

  torsoTask->stiffness(c_.torsoStiffness);
  torsoTask->weight(c_.torsoWeight);
  torsoTask->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}));

  zmpcc_.configure(c_.zmpcc);

  if(!c_.comActiveJoints.empty())
  {
    comTask->selectActiveJoints(solver, c_.comActiveJoints);
  }
  comTask->setGains(c_.comStiffness, 2 * c_.comStiffness.cwiseSqrt());
  comTask->weight(c_.comWeight);

  for(const auto & footTask : footTasks)
  {
    footTask.second->maxLinearVel(c_.copMaxVel.linear());
    footTask.second->maxAngularVel(c_.copMaxVel.angular());
  }
  
  dcmBiasEstimatorConfiguration(c_.dcmBias);

  reconfigure_ = false;
}

void DCMStabilizerTask::checkConfiguration(const DCMStabilizerConfiguration & config)
{
  // Check whether feet have force sensors
  auto checkSurface = [&](const std::string & surfaceName) {
    if(!robot().hasSurface(surfaceName))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] requires a surface named {} in robot {}", name(),
                                                       surfaceName, robot().name());
    }
    if(!robot().surfaceHasIndirectForceSensor(surfaceName))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Surface {} must have an associated force sensor.", name(),
                                                       surfaceName);
    }
  };
  checkSurface(config.rightFootSurface);
  checkSurface(config.leftFootSurface);
}

void DCMStabilizerTask::load(mc_solver::QPSolver &, const mc_rtc::Configuration & config)
{
  double height = 0;
  // Load contacts
  ContactDescriptionVector contactsToAdd;
  if(config.has("contacts"))
  {
    const auto & contacts = config("contacts");
    for(const auto & contactName : contacts)
    {
      ContactState s = contactName;
      sva::PTransformd contactPose = footTasks[s]->surfacePose();
      if(config.has(contactName))
      {
        const auto & c = config(contactName);
        if(c.has("rotation"))
        {
          contactPose.rotation() = c("rotation");
        }
        else if(c.has("overwriteRPY"))
        {
          // Only modify the specified DoF of the rotation
          mc_rtc::overwriteRotationRPY(c("overwriteRPY"), contactPose.rotation());
        }
        if(c.has("translation"))
        {
          contactPose.translation() = c("translation");
        }
        if(c.has("height"))
        {
          double h = c("height");
          contactPose.translation().z() = c("height");
          height = (height + h) / 2;
        }
      }
      contactsToAdd.push_back({s, {robot(), footTasks[s]->surface(), contactPose, c_.friction}});
    }
  }
  this->setContacts(contactsToAdd);

  // Target robot com by default
  Eigen::Vector3d comTarget = robot().com();
  if(config.has("staticTarget"))
  {
    if(config.has("com"))
    {
      comTarget = config("staticTarget")("com");
    }
  }
  this->staticTarget(comTarget, height);

  // Allow to start in disabled state
  if(!config("enabled", true))
  {
    this->disable();
  }
}

const DCMStabilizerConfiguration & DCMStabilizerTask::config() const
{
  return c_;
}

const DCMStabilizerConfiguration & DCMStabilizerTask::commitedConfig() const
{
  return defaultConfig_;
}

void DCMStabilizerTask::setContacts(const std::vector<ContactState> & contacts)
{
  ContactDescriptionVector addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact,
                           {robot(), footTasks[contact]->surface(),
                            realRobot().surfacePose(footTasks[contact]->surface()), c_.friction}});
  }
  setContacts(addContacts);
}

void DCMStabilizerTask::setContacts(const std::vector<std::pair<ContactState, sva::PTransformd>> & contacts)
{
  ContactDescriptionVector addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact.first, {robot(), footTasks[contact.first]->surface(), contact.second, c_.friction}});
  }
  setContacts(addContacts);
}

void DCMStabilizerTask::setContacts(const ContactDescriptionVector & contacts)
{
  if(contacts.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[DCMStabilizerTask] Cannot set contacts from an empty list, the stabilizer "
        "requires at least one contact to be set.");
  }
  contacts_.clear();
  spm_->clear();

  // Reset support area boundaries
  supportMin_ = std::numeric_limits<double>::max() * Eigen::Vector2d::Ones();
  supportMax_ = -supportMin_;
  supportPolygons_.clear();

  for(const auto & contact : contacts)
  {
    addContact(contact.first, contact.second);
  }
}

void DCMStabilizerTask::addContact(ContactState contactState, const Contact & contact)
{
  auto footTask = footTasks[contactState];
  // Use real robot's surface pose as contact
  contacts_.emplace(std::make_pair(contactState, contact));

  supportMin_.x() = std::min(contact.xmin(), supportMin_.x());
  supportMin_.y() = std::min(contact.ymin(), supportMin_.y());
  supportMax_.x() = std::max(contact.xmax(), supportMax_.x());
  supportMax_.y() = std::max(contact.ymax(), supportMax_.y());

  supportPolygons_.push_back(contact.polygon());

  // Configure support foot task
  footTask->reset();
  footTask->weight(c_.contactWeight);
  footTask->targetPose(contact.surfacePose());

  addContacts_.push_back(contactState);
  spm_->addVertices(contactState, contact.polygon(), true);
}

void DCMStabilizerTask::setSupportFootGains()
{
  if(inDoubleSupport())
  {
    for(auto contactT : contactTasks)
    {
      contactT->admittance(contactAdmittance());
      contactT->setGains(c_.contactStiffness, c_.contactDamping);
    }
  }
  else
  {
    sva::MotionVecd vdcContactStiffness = {c_.contactStiffness.angular(),
                                           {c_.vdcStiffness, c_.vdcStiffness, c_.vdcStiffness}};
    for(auto contactT : contactTasks)
    {
      contactT->admittance(contactAdmittance());
      contactT->setGains(vdcContactStiffness, c_.contactDamping);
    }
  }
}

void DCMStabilizerTask::checkInTheAir()
{
  inTheAir_ = true;
  for(const auto footT : footTasks)
  {
    inTheAir_ = inTheAir_ && footT.second->measuredWrench().force().z() < c_.safetyThresholds.MIN_DS_PRESSURE;
  }
}

sva::PTransformd DCMStabilizerTask::anchorFrame(const mc_rbdyn::Robot & robot) const
{
  return sva::interpolate(robot.surfacePose(footTasks.at(ContactState::Left)->surface()),
                          robot.surfacePose(footTasks.at(ContactState::Right)->surface()), leftFootRatio_);
}

void DCMStabilizerTask::updateZMPFrame()
{
  if(inDoubleSupport())
  {
    zmpFrame_ = sva::interpolate(contacts_.at(ContactState::Left).surfacePose(),
                                 contacts_.at(ContactState::Right).surfacePose(), 0.5);
  }
  else if(inContact(ContactState::Left))
  {
    zmpFrame_ = contacts_.at(ContactState::Left).surfacePose();
  }
  else
  {
    zmpFrame_ = contacts_.at(ContactState::Right).surfacePose();
  }
}

void DCMStabilizerTask::staticTarget(const Eigen::Vector3d & com, double zmpHeight)
{
  Eigen::Vector3d zmp = Eigen::Vector3d{com.x(), com.y(), zmpHeight};

  target(com, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), zmp);
}

void DCMStabilizerTask::target(const Eigen::Vector3d & com,
                               const Eigen::Vector3d & comd,
                               const Eigen::Vector3d & comdd,
                               const Eigen::Vector3d & zmp,
                               const Eigen::Vector3d & zmpd)
{
  comTargetRaw_ = com;
  comTarget_ = comTargetRaw_ - comOffsetErrCoM_;
  if(c_.extWrench.addExpectedCoMOffset)
  {
    comTarget_ -= comOffsetTarget_;
  }
  comdTarget_ = comd;
  comddTarget_ = comdd;
  zmpTarget_ = zmp;
  zmpdTarget_ = zmpd;
  double comHeight = comTarget_.z() - zmpTarget_.z();
  omega_ = std::sqrt(constants::gravity.z() / comHeight);
  dcmTarget_ = comTarget_ + comdTarget_ / omega_;
}

void DCMStabilizerTask::setExternalWrenches(const std::vector<std::string> & surfaceNames,
                                         const std::vector<sva::ForceVecd> & targetWrenches,
                                         const std::vector<sva::MotionVecd> & gains)
{
  if(surfaceNames.size() > 0
     && !(c_.extWrench.addExpectedCoMOffset || c_.extWrench.modifyCoMErr || c_.extWrench.modifyZMPErr
          || c_.extWrench.modifyZMPErrD))
  {
    mc_rtc::log::warning(
        "[DCMStabilizerTask] external wrenches are set, but the configurations for handling them are invalid.");
  }

  extWrenches_.clear();
  for(unsigned int i = 0; i < surfaceNames.size(); i++)
  {
    extWrenches_.push_back({targetWrenches[i], sva::ForceVecd::Zero(), gains[i], surfaceNames[i]});
    if(!robot().surfaceHasIndirectForceSensor(surfaceNames[i]))
    {
      mc_rtc::log::warning(
          "[DCMStabilizerTask] surface {} does not have force sensor. The target force is used as the measured force.",
          surfaceNames[i]);
    }
  }

  comOffsetTarget_ = computeCoMOffset<&ExternalWrench::target>(robot());
  comTarget_ = comTargetRaw_ - comOffsetErrCoM_;
  if(c_.extWrench.addExpectedCoMOffset)
  {
    comTarget_ -= comOffsetTarget_;
  }
}

template<sva::ForceVecd DCMStabilizerTask::ExternalWrench::*TargetOrMeasured>
Eigen::Vector3d DCMStabilizerTask::computeCoMOffset(const mc_rbdyn::Robot & robot) const
{
  Eigen::Vector3d comOffset = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos, force, moment;
  for(const auto & extWrench : extWrenches_)
  {
    computeExternalContact(robot, extWrench.surfaceName, extWrench.*TargetOrMeasured, pos, force, moment);

    comOffset.x() += (pos.z() - zmpTarget_.z()) * force.x() - pos.x() * force.z() + moment.y();
    comOffset.y() += (pos.z() - zmpTarget_.z()) * force.y() - pos.y() * force.z() - moment.x();
  }
  double verticalComAcc = comddTarget_.z() + constants::gravity.z();
  double verticalComAccThre = 1e-3;
  if(std::abs(verticalComAcc) < verticalComAccThre)
  {
    mc_rtc::log::warning(
        "[DCMStabilizerTask::computeCoMOffset] overwrite verticalComAcc because it's too close to zero: {}",
        verticalComAcc);
    verticalComAcc = verticalComAcc >= 0 ? verticalComAccThre : -verticalComAccThre;
  }
  comOffset /= robot.mass() * verticalComAcc;

  return comOffset;
}

template<sva::ForceVecd DCMStabilizerTask::ExternalWrench::*TargetOrMeasured>
sva::ForceVecd DCMStabilizerTask::computeExternalWrenchSum(const mc_rbdyn::Robot & robot,
                                                        const Eigen::Vector3d & com) const
{
  sva::ForceVecd extWrenchSum = sva::ForceVecd::Zero();
  Eigen::Vector3d pos, force, moment;
  for(const auto & extWrench : extWrenches_)
  {
    computeExternalContact(robot, extWrench.surfaceName, extWrench.*TargetOrMeasured, pos, force, moment);

    extWrenchSum.force() += force;
    extWrenchSum.moment() += (pos - com).cross(force) + moment;
  }

  return extWrenchSum;
}

void DCMStabilizerTask::computeExternalContact(const mc_rbdyn::Robot & robot,
                                            const std::string & surfaceName,
                                            const sva::ForceVecd & surfaceWrench,
                                            Eigen::Vector3d & pos,
                                            Eigen::Vector3d & force,
                                            Eigen::Vector3d & moment) const
{
  sva::PTransformd surfacePose = robot.surfacePose(surfaceName);
  sva::PTransformd T_s_0(Eigen::Matrix3d(surfacePose.rotation().transpose()));
  // Represent the surface wrench in the frame whose position is same with the surface frame and orientation is same
  // with the world frame
  sva::ForceVecd surfaceWrenchW = T_s_0.dualMul(surfaceWrench);
  pos = surfacePose.translation();
  force = surfaceWrenchW.force();
  moment = surfaceWrenchW.moment();
}

void DCMStabilizerTask::run()
{
  using namespace std::chrono;
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  auto startTime = clock::now();

  c_.clampGains();
  checkInTheAir();
  
  computeDesiredWrench();
  distributeWrench();
  
  setSupportFootGains();
  updateZMPFrame();
  if(!inTheAir_)
  {
    measuredNetWrench_ = robots_.robot(robotIndex_).netWrench(contactSensors);
    try
    {
      measuredZMP_ =
          robots_.robot(robotIndex_).zmp(measuredNetWrench_, zmpFrame_, c_.safetyThresholds.MIN_NET_TOTAL_FORCE_ZMP);
    }
    catch(std::runtime_error & e)
    {
      mc_rtc::log::error("[{}] ZMP computation failed, keeping previous value {}", name(), measuredZMP_.transpose());
    }
  }
  else
  {
    measuredNetWrench_ = sva::ForceVecd::Zero();
  }
  
  if(inDoubleSupport())
  {
    
  }
  else if(inContact(ContactState::Left))
  {
    footTasks[ContactState::Right]->setZeroTargetWrench();
  }
  else
  {
    footTasks[ContactState::Left]->setZeroTargetWrench();
  }
  
  updateFootForceDifferenceControl();
  
  comTask->com(comTarget_);
  comTask->refVel(comdTarget_);
  comTask->refAccel(comddTarget_);
  
  // Update orientation tasks according to feet orientation
  sva::PTransformd X_0_a = anchorFrame(robot());
  Eigen::Matrix3d pelvisOrientation = X_0_a.rotation();
  pelvisTask->orientation(pelvisOrientation);
  torsoTask->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}) * pelvisOrientation);

  auto endTime = clock::now();
  runTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
}

void DCMStabilizerTask::updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd)
{
  measuredCoM_ = com;
  measuredCoMd_ += (comd - measuredCoMd_) * c_.comVelLowPassFilter;
  
  Eigen::Vector3d comdd((comd - measuredCoMd_pre_)/dt_);
  measuredCoMdd_ += (comdd - measuredCoMdd_) * c_.comAccLowPassFilter;
  measuredCoMd_pre_ = comd;
  
  measuredDCM_ = measuredCoM_ + measuredCoMd_ / omega_;
}

void DCMStabilizerTask::computeDesiredWrench()
{
  Eigen::Vector3d comError = comTarget_ - measuredCoM_;
  Eigen::Vector3d comdError = comdTarget_ - measuredCoMd_;
  Eigen::Vector3d comddError = comddTarget_ - measuredCoMdd_;
  dcmError_ = comError + comdError / omega_;
  dcmVelError_ = comdError + comddError / omega_;
  dcmError_.z() = 0.;
  dcmVelError_.z() = 0.;
  
  if( c_.hasDCMPoles )
    setDCMGainsFromPoleAssignment(c_.dcmPoles[0], c_.dcmPoles[1], c_.dcmPoles[2], c_.dcmFlexibility);
  
  if(inTheAir_)
  {
    c_.dcmErrorSum.setZero();
    dcmEstimatorNeedsReset_ = true;
  }
  else
  {
    Eigen::Vector3d zmpError = zmpTarget_ - measuredZMP_;
    zmpError.z() = 0.;
    
    if(c_.dcmBias.withDCMBias)
    {
      if(omega_ != dcmEstimator_.getLipmNaturalFrequency())
      {
        dcmEstimator_.setLipmNaturalFrequency(omega_);
      }

      const Eigen::Matrix3d & waistOrientation = robot().posW().rotation().transpose();

      if(dcmEstimatorNeedsReset_)
      {
        dcmEstimator_.resetWithMeasurements(dcmError_.head<2>(), zmpError.head<2>(), waistOrientation, true);
        dcmEstimatorNeedsReset_ = false;
      }
      else
      {
        dcmEstimator_.setInputs(dcmError_.head<2>(), zmpError.head<2>(), waistOrientation);
      }

      /// run the estimation
      dcmEstimator_.update();

      if(c_.dcmBias.withDCMFilter)
      {
        /// the estimators provides a filtered DCM
        dcmError_.head<2>() = dcmEstimator_.getUnbiasedDCM();
      }
      else
      {
        dcmError_.head<2>() -= dcmEstimator_.getBias();
      }
      /// the bias should also correct the CoM
      comError.head<2>() -= dcmEstimator_.getBias();
      /// the unbiased dcm allows also to get the velocity of the CoM
      comdError.head<2>() = omega_ * (dcmError_.head<2>() - comError.head<2>());
      measuredDCMUnbiased_ = dcmTarget_ - dcmError_;
    }
    else
    {
      measuredDCMUnbiased_ = measuredDCM_;
      dcmEstimatorNeedsReset_ = true;
    }
  }
  
  Eigen::Vector3d balanceZMP
    = c_.dcmPropGain * dcmError_
    + c_.dcmDerivGain * dcmVelError_
    + c_.dcmIntegralGain * c_.dcmErrorSum;
  
  for(int axis = 0 ; axis <= 1 ; axis++ ){
    if( fabs(distribZMP_(axis) - modifiedZMP_(axis)) < 0.01 )
      c_.dcmErrorSum(axis) += dcmError_(axis) * dt_;
  }
  c_.dcmErrorSum = clamp(c_.dcmErrorSum,
                         -fabs(c_.safetyThresholds.MAX_DCM_ERROR/c_.dcmIntegralGain),
                         +fabs(c_.safetyThresholds.MAX_DCM_ERROR/c_.dcmIntegralGain));
  
  modifiedZMP_ = zmpTarget_ + c_.zmpdGain * zmpdTarget_  + balanceZMP;
  
  // Calculate CoM offset from measured wrench
  for(auto & extWrench : extWrenches_)
  {
    if(robot().surfaceHasIndirectForceSensor(extWrench.surfaceName))
    {
      extWrench.measured =
          sva::ForceVecd(extWrench.gain.vector().cwiseProduct(robot().surfaceWrench(extWrench.surfaceName).vector()));
    }
    else
    {
      extWrench.measured = extWrench.target;
    }
  }
  comOffsetMeasured_ = computeCoMOffset<&ExternalWrench::measured>(realRobot());
  
  // Modify the desired CoM and ZMP depending on the external wrench error
  comOffsetLowPass_.update(comOffsetMeasured_ - comOffsetTarget_);
  comOffsetErr_ = comOffsetLowPass_.eval();
  if(c_.extWrench.modifyCoMErr)
  {
    comOffsetLowPassCoM_.update(comOffsetErr_);
  }
  else
  {
    comOffsetLowPassCoM_.update(Eigen::Vector3d::Zero());
  }
  comOffsetErrCoM_ = comOffsetLowPassCoM_.eval();
  comOffsetErrZMP_ = comOffsetErr_ - comOffsetErrCoM_;
  clampInPlaceAndWarn(comOffsetErrCoM_, Eigen::Vector3d::Constant(-c_.extWrench.comOffsetErrCoMLimit).eval(),
                      Eigen::Vector3d::Constant(c_.extWrench.comOffsetErrCoMLimit).eval(), "comOffsetErrCoM");
  clampInPlaceAndWarn(comOffsetErrZMP_, Eigen::Vector3d::Constant(-c_.extWrench.comOffsetErrZMPLimit).eval(),
                      Eigen::Vector3d::Constant(c_.extWrench.comOffsetErrZMPLimit).eval(), "comOffsetErrZMP");
  comOffsetDerivator_.update(comOffsetErr_);
  
  distribZMP_ = modifiedZMP_;
  
#if 0
  if(c_.extWrench.modifyZMPErr)
  {
    desiredCoMAccel -= omega_ * omega_ * comOffsetErrZMP_;
  }
  if(c_.extWrench.modifyZMPErrD)
  {
    desiredCoMAccel -= (omega_ * omega_ / c_.zmpdGain) * comOffsetDerivator_.eval();
  }
  
  // Calculate the desired force and moment
  Eigen::Vector3d desiredForce = mass_ * (desiredCoMAccel + constants::gravity);
  Eigen::Vector3d desiredMoment = Eigen::Vector3d::Zero();

  // Subtract the external wrenches from the desired force and moment
  extWrenchSumTarget_ = computeExternalWrenchSum<&ExternalWrench::target>(robot(), comTarget_);
  extWrenchSumLowPass_.update(computeExternalWrenchSum<&ExternalWrench::measured>(realRobot(), measuredCoM_));
  extWrenchSumMeasured_ = extWrenchSumLowPass_.eval();
  if(c_.extWrench.subtractMeasuredValue)
  {
    desiredForce -= extWrenchSumMeasured_.force();
    desiredMoment -= extWrenchSumMeasured_.moment();
  }
  else
  {
    desiredForce -= extWrenchSumTarget_.force();
    desiredMoment -= extWrenchSumTarget_.moment();
  }
  
  distribZMP_ = modifiedZMP_;
    
  // Previous implementation (up to v1.3):
  // return {pendulum_.com().cross(desiredForce), desiredForce};
  // See https://github.com/stephane-caron/lipm_walking_controller/issues/28
  
  return {measuredCoM_.cross(desiredForce) + desiredMoment, desiredForce};
#endif
}

bool DCMStabilizerTask::calcForceDistributionRatio(Eigen::Matrix3d& Rlocal)
{
  bool limit_over = false;
  
  // Project desired ZMP in-between foot-sole ankle frames and compute ratio along the line in-beween the two surfaces
  if( inDoubleSupport() ){
    const auto & leftContact = contacts_.at(ContactState::Left);
    const auto & rightContact = contacts_.at(ContactState::Right);
    if( c_.disableFDZmpOffset ){
      zmpOffsets_.at(ContactState::Left).setZero();
      zmpOffsets_.at(ContactState::Right).setZero();
    }
    const Eigen::Vector3d lfoot_pos
      = (sva::PTransformd(zmpOffsets_.at(ContactState::Left)) * leftContact.anklePose()).translation();
    const Eigen::Vector3d rfoot_pos
      = (sva::PTransformd(zmpOffsets_.at(ContactState::Right)) * rightContact.anklePose()).translation();
    
    const Eigen::Vector3d p0(lfoot_pos - rfoot_pos);
    const double length = sqrt(p0.x()*p0.x() + p0.y()*p0.y());
    const double cth = p0.y() / length;
    const double sth = p0.x() / length;
    Rlocal <<
      cth, -sth, 0.0,
      sth,  cth, 0.0,
      0.0,  0.0, 1.0;
    
    spm_->getGlobalAllSupportPolygon();
    if( spm_->checkAllConvexHullInclusion(modifiedZMP_) ){
      Eigen::Vector3d rfoot_pos_local = Rlocal * (rfoot_pos - modifiedZMP_);
      Eigen::Vector3d lfoot_pos_local = Rlocal * (lfoot_pos - modifiedZMP_);
      leftFootRatio_ = lfoot_pos_local.y() / (lfoot_pos_local.y()-rfoot_pos_local.y());
    }
    else{
      //force distribution by the closed ZMP of support region
      spm_->getClosestPointInAllSupportPolygon(distribZMP_, modifiedZMP_);
      if( spm_->checkConvexHullInclusion(ContactState::Left, distribZMP_) )
        leftFootRatio_ = 0.0;
      else if( spm_->checkConvexHullInclusion(ContactState::Right, distribZMP_) )
        leftFootRatio_ = 1.0;
      else{
        Eigen::Vector3d rfoot_pos_local2 = Rlocal * (rfoot_pos - distribZMP_);
        Eigen::Vector3d lfoot_pos_local2 = Rlocal * (lfoot_pos - distribZMP_);
        leftFootRatio_ = lfoot_pos_local2.y() / (lfoot_pos_local2.y()-rfoot_pos_local2.y());
      }
      limit_over = true;
    }
    clamp(leftFootRatio_, 0.0, 1.0);
  }
  else if( inContact(ContactState::Left) )
  {
    leftFootRatio_ = 0.0;
    
    if( !spm_->checkConvexHullInclusion(ContactState::Left, modifiedZMP_) ){
      spm_->getClosestPointInSupportPolygon(ContactState::Left, distribZMP_, modifiedZMP_);
      limit_over = true;
    }
  }
  else if( inContact(ContactState::Right) )
  {
    leftFootRatio_ = 1.0;
    
    if( !spm_->checkConvexHullInclusion(ContactState::Right, modifiedZMP_) ){
      spm_->getClosestPointInSupportPolygon(ContactState::Right, distribZMP_, modifiedZMP_);
      limit_over = true;
    }
  }
  
  return limit_over;
}

void DCMStabilizerTask::setDCMGainsFromPoleAssignment(double alpha, double beta, double gamma, double omega_zmp)
{
  c_.dcmIntegralGain =  -alpha*beta*gamma/(omega_*omega_zmp);
  c_.dcmPropGain = -(alpha*beta + beta*gamma + gamma*alpha + omega_*omega_zmp) / (omega_*omega_zmp);
  c_.dcmDerivGain = -(alpha + beta + gamma + omega_ - omega_zmp) / (omega_*omega_zmp);
}

void DCMStabilizerTask::distributeWrench()
{
  Eigen::Matrix3d Rlocal = Eigen::Matrix3d::Identity();
  bool zmp_limit_over_prev = zmp_limit_over_;
  zmp_limit_over_ = calcForceDistributionRatio(Rlocal);
  distribWrench_.force() = mass_ * (comddTarget_ + constants::gravity);
  distribWrench_.moment() = distribZMP_.cross(distribWrench_.force());
  
  if( zmp_limit_over_ ){
    if( !zmp_limit_over_ && zmp_limit_over_prev ){
      mc_rtc::log::warning("recovery of zmp over at {} [s]", t_);
    }
    else if( zmp_limit_over_ && !zmp_limit_over_prev ){
      mc_rtc::log::warning("zmp over!! desired({},{}), actual({},{}) since {}[s]",
                           modifiedZMP_.x(), modifiedZMP_.y(), distribZMP_.x(), distribZMP_.y(), t_);
    }
  }
  
  if( zmp_limit_over_ ){
    uncompTorque_ = (distribZMP_ - modifiedZMP_).cross(distribWrench_.force());
  }
  else{
    uncompTorque_.setZero();
  }
    
  Eigen::Vector3d rfoot_torque_local = Eigen::Vector3d::Zero();
  Eigen::Vector3d lfoot_torque_local = Eigen::Vector3d::Zero();
  
  Eigen::Vector3d rfoot_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d lfoot_force = Eigen::Vector3d::Zero();
  
  Eigen::Vector3d rfoot_torque = Eigen::Vector3d::Zero();
  Eigen::Vector3d lfoot_torque = Eigen::Vector3d::Zero();
  
  if( inDoubleSupport() ){
#if 1
    const auto & leftContact = contacts_.at(ContactState::Left);
    const auto & rightContact = contacts_.at(ContactState::Right);
    const sva::PTransformd & X_0_lc = leftContact.surfacePose();
    const sva::PTransformd & X_0_rc = rightContact.surfacePose();
#else  
    const sva::PTransformd & X_0_rc = footTasks.at(ContactState::Right)->targetPose();
    const sva::PTransformd & X_0_lc = footTasks.at(ContactState::Left)->targetPose();
#endif
    
    const Eigen::Vector3d & rfoot_pos = X_0_rc.translation();
    const Eigen::Vector3d & lfoot_pos = X_0_lc.translation();
    rfoot_torque.z() = 0.0;
    lfoot_torque.z() = 0.0;
    
    rfoot_force = distribWrench_.force() * leftFootRatio_;
    lfoot_force = distribWrench_.force() * (1.0 - leftFootRatio_);
    Eigen::Vector3d tau(Rlocal *
                        ((distribZMP_ - rfoot_pos).cross(rfoot_force)
                         + (distribZMP_ - lfoot_pos).cross(lfoot_force)));
    
    if( leftFootRatio_ == 0.0 ){
      if( tau.x() < 0.0 ){
        rfoot_torque_local.x() = 0.0;
        lfoot_torque_local.x() = tau.x();
      }
      else{
        rfoot_torque_local.x() = 0.0;
        lfoot_torque_local.x() = 0.0;
      }
    }
    else if( leftFootRatio_ == 1.0 ){
      if( tau.x() > 0.0 ){
        rfoot_torque_local.x() = tau.x();
        lfoot_torque_local.x() = 0.0;
      }
      else{
        rfoot_torque_local.x() = 0.0;
        lfoot_torque_local.x() = 0.0;
      }
    }
    else{
      rfoot_torque_local.x() = 0.0;
      lfoot_torque_local.x() = 0.0;
    }
    
    rfoot_torque_local.y() = leftFootRatio_ * tau.y();
    lfoot_torque_local.y() = (1.0 - leftFootRatio_) * tau.y();
    
    rfoot_torque_local.z() = 0.0;
    lfoot_torque_local.z() = 0.0;
    
    rfoot_torque = Rlocal.transpose() * rfoot_torque_local;
    lfoot_torque = Rlocal.transpose() * lfoot_torque_local;
    
    rfoot_torque += rfoot_pos.cross(rfoot_force);
    lfoot_torque += lfoot_pos.cross(lfoot_force);
    rfoot_torque.z() = 0.0;
    lfoot_torque.z() = 0.0;
    
    const sva::ForceVecd w_r_0(rfoot_torque, rfoot_force);
    const sva::ForceVecd w_l_0(lfoot_torque, lfoot_force);
    sva::ForceVecd w_l_lc = X_0_lc.dualMul(w_l_0);
    sva::ForceVecd w_r_rc = X_0_rc.dualMul(w_r_0);
    Eigen::Vector2d leftCoP = Eigen::Vector2d::Zero();
    if( w_l_lc.force()(2) > 1e-3 )
      leftCoP = (constants::vertical.cross(w_l_lc.couple()) / w_l_lc.force()(2)).head<2>();
    Eigen::Vector2d rightCoP = Eigen::Vector2d::Zero();
    if( w_r_rc.force()(2) > 1e-3 )
      rightCoP = (constants::vertical.cross(w_r_rc.couple()) / w_r_rc.force()(2)).head<2>();
    footTasks[ContactState::Left]->targetCoP(leftCoP);
    footTasks[ContactState::Left]->targetForce(w_l_lc.force());
    footTasks[ContactState::Right]->targetCoP(rightCoP);
    footTasks[ContactState::Right]->targetForce(w_r_rc.force());
  }
  else if( inContact(ContactState::Left) ){
#if 1
    const auto & leftContact = contacts_.at(ContactState::Left);
    const sva::PTransformd & X_0_lc = leftContact.surfacePose();
#else  
    const sva::PTransformd & X_0_lc = footTasks.at(ContactState::Left)->targetPose();
#endif
    
    lfoot_force = distribWrench_.force();
    Eigen::Vector3d tau(Rlocal * distribZMP_.cross(lfoot_force));
    lfoot_torque_local.x() = tau.x();
    lfoot_torque_local.y() = tau.y();
    lfoot_torque = Rlocal.transpose() * lfoot_torque_local;
    lfoot_torque.z() = 0.0;
    
    const sva::ForceVecd w_l_0(lfoot_torque, lfoot_force);
    sva::ForceVecd w_l_lc = X_0_lc.dualMul(w_l_0);
    Eigen::Vector2d leftCoP = Eigen::Vector2d::Zero();
    if( w_l_lc.force()(2) > 1e-3 )
      leftCoP = (constants::vertical.cross(w_l_lc.couple()) / w_l_lc.force()(2)).head<2>();
    footTasks[ContactState::Left]->targetCoP(leftCoP);
    footTasks[ContactState::Left]->targetForce(w_l_lc.force());
  }
  else if( inContact(ContactState::Right) ){
#if 1
    const auto & rightContact = contacts_.at(ContactState::Right);
    const sva::PTransformd & X_0_rc = rightContact.surfacePose();
#else  
    const sva::PTransformd & X_0_rc = footTasks.at(ContactState::Right)->targetPose();
#endif
    
    rfoot_force = distribWrench_.force();
    Eigen::Vector3d tau(Rlocal * distribZMP_.cross(rfoot_force));
    rfoot_torque_local.x() = tau.x();
    rfoot_torque_local.y() = tau.y();
    rfoot_torque = Rlocal.transpose() * rfoot_torque_local;
    rfoot_torque.z() = 0.0;
    
    const sva::ForceVecd w_r_0(rfoot_torque, rfoot_force);
    sva::ForceVecd w_r_rc = X_0_rc.dualMul(w_r_0);
    Eigen::Vector2d rightCoP = Eigen::Vector2d::Zero();
    if( w_r_rc.force()(2) > 1e-3 )
      rightCoP = (constants::vertical.cross(w_r_rc.couple()) / w_r_rc.force()(2)).head<2>();
    footTasks[ContactState::Right]->targetCoP(rightCoP);
    footTasks[ContactState::Right]->targetForce(w_r_rc.force());
  }
}

void DCMStabilizerTask::updateCoMTaskZMPCC()
{
  c_.zmpcc = zmpcc_.config();
  if(inTheAir_ || (zmpccOnlyDS_ && !inDoubleSupport()))
  {
    zmpcc_.enabled(false); // Leak to zero
  }
  else
  {
    zmpcc_.configure(c_.zmpcc);
    zmpcc_.enabled(enabled_);
    zmpcc_.update(distribZMP_, measuredZMP_, zmpFrame_, dt_);
  }
  zmpcc_.apply(comTarget_, comdTarget_, comddTarget_);
}

void DCMStabilizerTask::updateFootForceDifferenceControl()
{
  auto leftFootTask = footTasks[ContactState::Left];
  auto rightFootTask = footTasks[ContactState::Right];
  if(!inDoubleSupport() || inTheAir_)
  {
    dfzForceError_ = 0.;
    dfzHeightError_ = 0.;
    vdcHeightError_ = 0.;
    leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    return;
  }

  double LFz_d = leftFootTask->targetWrench().force().z();
  double RFz_d = rightFootTask->targetWrench().force().z();
  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  dfzForceError_ = (LFz_d - RFz_d) - (LFz - RFz);

  double LTz_d = leftFootTask->targetPose().translation().z();
  double RTz_d = rightFootTask->targetPose().translation().z();
  double LTz = leftFootTask->surfacePose().translation().z();
  double RTz = rightFootTask->surfacePose().translation().z();
  dfzHeightError_ = (LTz_d - RTz_d) - (LTz - RTz);
  vdcHeightError_ = (LTz_d + RTz_d) - (LTz + RTz);

  double dz_ctrl = c_.dfzAdmittance * dfzForceError_ - c_.dfzDamping * dfzHeightError_;
  double dz_vdc = c_.vdcFrequency * vdcHeightError_;
  sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
  sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_vdc}};
  leftFootTask->refVelB(0.5 * (velT - velF));
  rightFootTask->refVelB(0.5 * (velT + velF));
}

template Eigen::Vector3d DCMStabilizerTask::computeCoMOffset<&DCMStabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &) const;
template Eigen::Vector3d DCMStabilizerTask::computeCoMOffset<&DCMStabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &) const;

template sva::ForceVecd DCMStabilizerTask::computeExternalWrenchSum<&DCMStabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;
template sva::ForceVecd DCMStabilizerTask::computeExternalWrenchSum<&DCMStabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;

} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "dcm_stabilizer",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      unsigned robotIndex = robotIndexFromConfig(config, solver.robots(), "stabilizer");
      const auto & robot = solver.robots().robot(robotIndex);

      // Load user-specified configuration
      mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration stabiConf(robot.module().defaultLIPMStabilizerConfiguration());
      stabiConf.load(config);
      // Load user-specified stabilizer configuration for this robot
      if(config.has(robot.name()))
      {
        stabiConf.load(config(robot.name()));
      }

      auto t = std::allocate_shared<mc_tasks::lipm_stabilizer::DCMStabilizerTask>(
          Eigen::aligned_allocator<mc_tasks::lipm_stabilizer::DCMStabilizerTask>{}, solver.robots(), solver.realRobots(),
          robotIndex, stabiConf.leftFootSurface, stabiConf.rightFootSurface, stabiConf.torsoBodyName, solver.dt());
      t->configure(stabiConf);
      t->load(solver, config);
      t->reset();
      return t;
    });
}
