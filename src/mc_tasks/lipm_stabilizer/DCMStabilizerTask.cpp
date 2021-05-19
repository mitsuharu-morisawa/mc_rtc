/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is extracted hmc2's stabilizer as part of
 * hmc2 <https://github.com/isri-aist/hmc2>
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
  extWrenchSumLowPass_(dt, /* cutoffPeriod = */ 0.05),
  comOffsetLowPass_(dt, /* cutoffPeriod = */ 0.05),
  comOffsetLowPassCoM_(dt, /* cutoffPeriod = */ 1.0),
  comOffsetDerivator_(dt, /* timeConstant = */ 1.),
  dcmDerivator_(dt, /* timeConstant = */ 1.),
  dt_(dt),
  mass_(robots.robot(robotIndex).mass())
{
  std::cout << "rightSurface = " << rightSurface << std::endl;
  std::cout << "leftSurface = " << leftSurface << std::endl;
  
  type_ = "dcm_stabilizer";
  name_ = type_ + "_" + robots.robot(robotIndex).name();
  
  spm_ = std::make_shared<computational_geometry::SupportPolygonManager<ContactState> >();

  comTask_.reset(new mc_tasks::CoMTask(robots, robotIndex_));
  
#if 0
  footTasks_[ContactState::Left]
    = std::allocate_shared<mc_tasks::force::ImpedanceTask>(Eigen::aligned_allocator<mc_tasks::force::ImpedanceTask>{},
                                                           leftSurface, robots, robotIndex_);
  footTasks_[ContactState::Right]
    =  std::allocate_shared<mc_tasks::force::ImpedanceTask>(Eigen::aligned_allocator<mc_tasks::force::ImpedanceTask>{},
                                                            rightSurface, robots, robotIndex_);
#endif
  footTasks_[ContactState::Left] = std::make_shared<mc_tasks::force::ImpedanceTask>(leftSurface, robots, robotIndex_);
  footTasks_[ContactState::Right] =  std::make_shared<mc_tasks::force::ImpedanceTask>(rightSurface, robots, robotIndex_);
  
  std::string pelvisBodyName = robot().mb().body(0).name();
  pelvisTask_ = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots_, robotIndex_);
  torsoTask_ = std::make_shared<mc_tasks::OrientationTask>(torsoBodyName, robots_, robotIndex_);

  // TODO: default value should be set by config
  footMass(ImpedanceType::Stand, {{1, 1, 1}, {50, 50, 50}}, true);
  footDamping(ImpedanceType::Stand, {{100, 100, 100}, {50*400, 50*400, 50*100}}, true);
  footStiffness(ImpedanceType::Stand, {{0, 0, 400}, {0, 0, 0}}, true);
  footWrench(ImpedanceType::Stand, {{1, 1, 0}, {1, 1, 1}}, true);
  
  footMass(ImpedanceType::DoubleSupport, {{1, 1, 1}, {50, 50, 50}});
  footDamping(ImpedanceType::DoubleSupport, {{100, 100, 100}, {50*400, 50*400, 50*30}});
  footStiffness(ImpedanceType::DoubleSupport, {{0, 0, 400}, {0, 0, 0}});
  footWrench(ImpedanceType::DoubleSupport, {{1, 1, 0}, {1, 1, 1}});

  footMass(ImpedanceType::SingleSupport, {{1, 1, 1}, {100, 100, 100}});
  footDamping(ImpedanceType::SingleSupport, {{100, 100, 100}, {100*30, 100*30, 100*30}});
  footStiffness(ImpedanceType::SingleSupport, {{0, 0, 0}, {100*225, 100*225, 100*225}});
  footWrench(ImpedanceType::SingleSupport, {{1, 1, 0}, {0, 0, 0}});
  
  footMass(ImpedanceType::Swing, {{1, 1, 1}, {10, 10, 10}});
  footDamping(ImpedanceType::Swing, {{40, 40, 40}, {10*30, 10*30, 10*30}});
  footStiffness(ImpedanceType::Swing, {{400, 400, 400}, {10*225, 10*225, 10*225}});
  footWrench(ImpedanceType::Swing, {{0, 0, 0}, {0, 0, 0}});
  
  footMass(ImpedanceType::Air, {{1, 1, 1}, {10, 10, 10}});
  footDamping(ImpedanceType::Air, {{30, 30, 30}, {10*30, 10*30, 10*30}});
  footStiffness(ImpedanceType::Air, {{100, 100, 100}, {10*100, 10*100, 10*100}});
  footWrench(ImpedanceType::Air, {{0, 0, 0}, {0, 0, 0}});
  
  walkingPhase(ContactState::Left, ImpedanceType::Stand, dt, 0.0);
  walkingPhase(ContactState::Right, ImpedanceType::Stand, dt, 0.0);
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
  comTask_->reset();
  comTarget_ = comTask_->com();
  comTargetRaw_ = comTarget_;
  zmpTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), 0.};
  zmpdTarget_ = Eigen::Vector3d::Zero();

  for(auto footT : footTasks_)
  {
    footT.second->reset();
  }

  pelvisTask_->reset();
  torsoTask_->reset();
  
  dcmErrorSum_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  distribWrench_ = sva::ForceVecd::Zero();
  
  extWrenches_.clear();
  extWrenchSumTarget_ = sva::ForceVecd::Zero();
  extWrenchSumMeasured_ = sva::ForceVecd::Zero();
  comOffsetTarget_ = Eigen::Vector3d::Zero();
  comOffsetMeasured_ = Eigen::Vector3d::Zero();
  comOffsetErr_ = Eigen::Vector3d::Zero();
  comOffsetErrCoM_ = Eigen::Vector3d::Zero();
  comOffsetErrZMP_ = Eigen::Vector3d::Zero();
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());
  
  dcmDerivator_.reset(Eigen::Vector3d::Zero());
  
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
  Eigen::VectorXd res(3 + 3 * footTasks_.size());
  res.head(3) = comTask_->eval();
  int i = 0;
  for(const auto & footT : footTasks_)
  {
    res.segment(3 + 3 * i++, 3) = footT.second->eval();
  }
  return res;
}

Eigen::VectorXd DCMStabilizerTask::speed() const
{
  Eigen::VectorXd res(3 + 3 * footTasks_.size());
  res.head(3) = comTask_->speed();
  int i = 0;
  for(const auto & footT : footTasks_)
  {
    res.segment(3 + 3 * i++, 3) = footT.second->speed();
  }
  return res;
}

void DCMStabilizerTask::addToSolver(mc_solver::QPSolver & solver)
{
  // Feet tasks are added in update() instead, add all other tasks now
  MetaTask::addToSolver(*comTask_, solver);
  MetaTask::addToSolver(*pelvisTask_, solver);
  MetaTask::addToSolver(*torsoTask_, solver);
  for(const auto footT : footTasks_)
  {
    MetaTask::addToSolver(*(footT.second), solver);
  }
}

void DCMStabilizerTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*comTask_, solver);
  MetaTask::removeFromSolver(*pelvisTask_, solver);
  MetaTask::removeFromSolver(*torsoTask_, solver);
  for(const auto footT : footTasks_)
  {
    MetaTask::removeFromSolver(*(footT.second), solver);
  }
}

void DCMStabilizerTask::updateFootSensors()
{
  footSensors_.clear();
  for(const auto contact : contacts_)
  {
    if( inContact(contact.first) ){
      auto footTask = footTasks_[contact.first];
      const auto & fs = robot().indirectSurfaceForceSensor(footTask->surface());
      footSensors_.push_back(fs.name());
    }
  }
}

void DCMStabilizerTask::update(mc_solver::QPSolver & solver)
{
  // Prevent configuration changes while the stabilizer is disabled
  if(!enabled_)
  {
    c_ = lastConfig_;
  }
  if(reconfigure_) configure_(solver);
  
  updateFootSensors();
  
  updateState(realRobots_.robot().com(), realRobots_.robot().comVelocity());
  
  // Run stabilizer
  run();
  
  MetaTask::update(*comTask_, solver);
  MetaTask::update(*pelvisTask_, solver);
  MetaTask::update(*torsoTask_, solver);
  for(const auto footT : footTasks_)
  {
    MetaTask::update(*(footT.second), solver);
  }

  t_ += dt_;
}

void DCMStabilizerTask::enable()
{
  mc_rtc::log::info("[DCMStabilizerTask] enabled");
  // Reset DCM integrator when enabling the stabilizer.
  // While idle, it will accumulate a lot of error, and would case the robot to
  // move suddently to compensate it otherwise
  dcmDerivator_.reset(Eigen::Vector3d::Zero());
  
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());
  
  configure(lastConfig_);
  enabled_ = true;
}

void DCMStabilizerTask::disable()
{
  mc_rtc::log::info("[DCMStabilizerTask] disabled");
  // Save current configuration to be reused when re-enabling
  lastConfig_ = c_;
  // Set the stabilizer gains to zero
  c_.dcmDerivGain = 0.;
  c_.dcmIntegralGain = 0.;
  c_.dcmPropGain = 0.;
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
  dcmDerivator_.timeConstant(c_.dcmDerivatorTimeConstant);

  
  extWrenchSumLowPass_.cutoffPeriod(c_.extWrench.extWrenchSumLowPassCutoffPeriod);
  comOffsetLowPass_.cutoffPeriod(c_.extWrench.comOffsetLowPassCutoffPeriod);
  comOffsetLowPassCoM_.cutoffPeriod(c_.extWrench.comOffsetLowPassCoMCutoffPeriod);
  comOffsetDerivator_.timeConstant(c_.extWrench.comOffsetDerivatorTimeConstant);

  // // Configure upper-body tasks
  pelvisTask_->stiffness(c_.pelvisStiffness);
  pelvisTask_->weight(c_.pelvisWeight);

  torsoTask_->stiffness(c_.torsoStiffness);
  torsoTask_->weight(c_.torsoWeight);
  torsoTask_->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}));

  if(!c_.comActiveJoints.empty())
  {
    comTask_->selectActiveJoints(solver, c_.comActiveJoints);
  }
  comTask_->setGains(c_.comStiffness, 2 * c_.comStiffness.cwiseSqrt());
  comTask_->weight(c_.comWeight);

  for(const auto & footT : footTasks_)
  {
    if(c_.cutoffRatio > 0.0 )
      footT.second->cutoffPeriod(dt_/c_.cutoffRatio);
    footT.second->setGains(c_.footMotionStiffness, sva::MotionVecd(2 * c_.footMotionStiffness.angular().cwiseSqrt(),
                                                                   2 * c_.footMotionStiffness.linear().cwiseSqrt()) );
    footT.second->weight(c_.footWeight);
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
      sva::PTransformd contactPose = footTasks_[s]->surfacePose();
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
      contactsToAdd.push_back({s, {robot(), footTasks_[s]->surface(), contactPose, c_.friction}});
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
          {robot(), footTasks_[contact]->surface(),
              robot().surfacePose(footTasks_[contact]->surface()), c_.friction}});
  }
  setContacts(addContacts);
}

void DCMStabilizerTask::setContacts(const std::vector<std::pair<ContactState, sva::PTransformd>> & contacts)
{
  ContactDescriptionVector addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact.first,
          {robot(), footTasks_[contact.first]->surface(),
              contact.second, c_.friction}});
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

  for(const auto & contact : contacts)
  {
    addContact(contact.first, contact.second);
  }
}

void DCMStabilizerTask::addContact(ContactState contactState, const Contact & contact)
{
  auto footT = footTasks_[contactState];
  addContacts_.push_back(contactState);
  
  // Use real robot's surface pose as contact
  contacts_.emplace(std::make_pair(contactState, contact));
  
  supportMin_.x() = std::min(contact.xmin(), supportMin_.x());
  supportMin_.y() = std::min(contact.ymin(), supportMin_.y());
  supportMax_.x() = std::max(contact.xmax(), supportMax_.x());
  supportMax_.y() = std::max(contact.ymax(), supportMax_.y());
  
  //std::cout << "size of contact.polygon() = " << contact.polygon().size() << std::endl;
  
  supportPolygons_.clear();
  supportPolygons_.push_back(contact.polygon());
  spm_->addVertices(contactState, contact.polygon(), inContact(contactState));
}

// Todo set impedance gain according to contact phase
void DCMStabilizerTask::setFootImpedanceGains()
{
  ImpedanceType RightWalkingPhase = std::get<0>(walking_phase_[ContactState::Right]);
  double RightWalkingPhaseDuration = std::get<1>(walking_phase_[ContactState::Right]);
  double RightWalkingCurrentTime = std::get<2>(walking_phase_[ContactState::Right]);
  footTasks_[ContactState::Right]->weight(c_.footWeight);
  footTasks_[ContactState::Right]->gains().wrench().vec(c_.footWrench[RightWalkingPhase]);
  footTasks_[ContactState::Right]->gains().M().vec(c_.footMass[RightWalkingPhase]);
  footTasks_[ContactState::Right]->gains().D().vec(c_.footStiffness[RightWalkingPhase]);
  footTasks_[ContactState::Right]->gains().K().vec(c_.footDamping[RightWalkingPhase]);
  
  ImpedanceType LeftWalkingPhase = std::get<0>(walking_phase_[ContactState::Left]);
  double LeftWalkingPhaseDuration = std::get<1>(walking_phase_[ContactState::Left]);
  double LeftWalkingCurrentTime = std::get<2>(walking_phase_[ContactState::Left]);
  footTasks_[ContactState::Left]->weight(c_.footWeight);
  footTasks_[ContactState::Left]->gains().wrench().vec(c_.footWrench[LeftWalkingPhase]);
  footTasks_[ContactState::Left]->gains().M().vec(c_.footMass[LeftWalkingPhase]);
  footTasks_[ContactState::Left]->gains().D().vec(c_.footStiffness[LeftWalkingPhase]);
  footTasks_[ContactState::Left]->gains().K().vec(c_.footDamping[LeftWalkingPhase]);
}

void DCMStabilizerTask::checkInTheAir()
{
  inTheAir_ = true;
  for(const auto footT : footTasks_)
  {
    inTheAir_ = inTheAir_ && footT.second->measuredWrench().force().z() < c_.safetyThresholds.MIN_PRESSURE;
  }
}

bool DCMStabilizerTask::calcForceDistributionRatio(Eigen::Matrix3d& Rfoot)
{
  bool limit_over = false;
  
  // Project desired ZMP in-between foot-sole ankle frames and compute ratio along the line in-beween the two surfaces
  const Eigen::Vector3d & rfoot_pos = footTasks_.at(ContactState::Right)->targetPose().translation();
  const Eigen::Vector3d & lfoot_pos = footTasks_.at(ContactState::Left)->targetPose().translation();
  const Eigen::Vector3d p0(lfoot_pos - rfoot_pos);
  const double length = sqrt(p0.x()*p0.x() + p0.y()*p0.y());
  const double cth = p0.y() / length;
  const double sth = p0.x() / length;
  Rfoot <<
    cth, -sth, 0.0,
    sth,  cth, 0.0,
    0.0,  0.0, 1.0;
  
  if( inDoubleSupport() ){
    spm_->getGlobalAllSupportPolygon();

    if( spm_->checkAllConvexHullInclusion(modifiedZMP_) ){
      Eigen::Vector3d rfoot_pos_local = Rfoot * (rfoot_pos - modifiedZMP_);
      Eigen::Vector3d lfoot_pos_local = Rfoot * (lfoot_pos - modifiedZMP_);
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
        Eigen::Vector3d rfoot_pos_local2 = Rfoot * (rfoot_pos - distribZMP_);
        Eigen::Vector3d lfoot_pos_local2 = Rfoot * (lfoot_pos - distribZMP_);
        leftFootRatio_ = lfoot_pos_local2.y() / (lfoot_pos_local2.y()-rfoot_pos_local2.y());
      }
      limit_over = true;
    }
    clampInPlaceAndWarn(leftFootRatio_, 0.0, 1.0, "leftFootRatio");
  }
  else if( inContact(ContactState::Left) ){
    leftFootRatio_ = 0.0;
  }
  else if( inContact(ContactState::Right) ){
    leftFootRatio_ = 1.0;
  }
  
  //std::cout << "leftFootRatio = " << leftFootRatio_ << std::endl;
  
  return limit_over;
}

sva::PTransformd DCMStabilizerTask::anchorFrame(const mc_rbdyn::Robot & robot) const
{
  return sva::interpolate(robot.surfacePose(footTasks_.at(ContactState::Left)->surface()),
                          robot.surfacePose(footTasks_.at(ContactState::Right)->surface()),
                          leftFootRatio_);
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
  else if(inContact(ContactState::Right))
  {
    zmpFrame_ = contacts_.at(ContactState::Right).surfacePose();
  }
  
  //std::cout << "zmpFrame = " << zmpFrame_.translation().transpose() << std::endl;
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
  
  //std::cout << "zmpTarget = " << zmp.transpose() << std::endl;
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
  
  //c_.clampGains();
  checkInTheAir();

  setFootImpedanceGains();
  updateZMPFrame();
  if(!inTheAir_)
  {
    measuredNetWrench_ = robots_.robot(robotIndex_).netWrench(footSensors_);
    
#if 0
    std::cout << "total force = " << measuredNetWrench_.force().transpose() << std::endl;
    std::cout << "total torque = " << measuredNetWrench_.moment().transpose() << std::endl;
#endif
    
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
  
  computeDesiredWrench();
  distributeWrench();
  //distribZMP_ = mc_rbdyn::zmp(distribWrench_, zmpFrame_);
  updateFootImpedanceControl();
  
  comTask_->com(comTarget_);
  comTask_->refVel(comdTarget_);
  comTask_->refAccel(comddTarget_);

  // Update orientation tasks according to feet orientation
  sva::PTransformd X_0_a = anchorFrame(robot());
  //Eigen::Matrix3d pelvisOrientation = X_0_a.rotation();
  Eigen::Matrix3d pelvisOrientation = sva::RotZ(mc_rbdyn::rpyFromMat(X_0_a.rotation())[2]);
  pelvisTask_->orientation(pelvisOrientation);
  torsoTask_->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}) * pelvisOrientation);
  
  auto endTime = clock::now();
  runTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
}

void DCMStabilizerTask::updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd)
{
  measuredCoM_ = com;
  measuredCoMd_ = comd;
  measuredDCM_ = measuredCoM_ + measuredCoMd_ / omega_;
}

void DCMStabilizerTask::setDCMGainsFromPoleAssignment(double alpha, double beta, double gamma, double omega_zmp)
{
  c_.dcmIntegralGain =  -alpha*beta*gamma/(omega_*omega_zmp);
  c_.dcmPropGain = -(alpha*beta + beta*gamma + gamma*alpha + omega_*omega_zmp) / (omega_*omega_zmp);
  c_.dcmDerivGain = -(alpha + beta + gamma + omega_ - omega_zmp) / (omega_*omega_zmp);
}

void DCMStabilizerTask::computeDesiredWrench()
{
  Eigen::Vector3d comError = comTarget_ - measuredCoM_;
  Eigen::Vector3d comdError = comdTarget_ - measuredCoMd_;
  dcmError_ = comError + comdError / omega_;
  dcmError_.z() = 0.;
  
  if( c_.hasDCMPoles )
    setDCMGainsFromPoleAssignment(c_.dcmPoles[0], c_.dcmPoles[1], c_.dcmPoles[2], c_.dcmFlexibility);
  
  if(inTheAir_)
  {
    dcmDerivator_.reset(Eigen::Vector3d::Zero());
    dcmErrorSum_ = Eigen::Vector3d::Zero();
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
    
    dcmDerivator_.update(dcmError_);
    dcmErrorSum_ += dcmError_ * dt_;
    dcmErrorSum_ = clamp(dcmErrorSum_, -c_.safetyThresholds.MAX_AVERAGE_DCM_ERROR, c_.safetyThresholds.MAX_AVERAGE_DCM_ERROR);
  }
  dcmVelError_ = dcmDerivator_.eval();
  
  modifiedZMP_ = zmpTarget_;
  modifiedZMP_ += c_.dcmPropGain * dcmError_;
  //modifiedZMP_ += c_.dcmIntegralGain * dcmErrorSum_;
  //modifiedZMP_ += c_.dcmDerivGain * dcmVelError_;
  
#if 0
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
  
  // Subtract the external wrenches from the desired force and moment
  extWrenchSumTarget_ = computeExternalWrenchSum<&ExternalWrench::target>(robot(), comTarget_);
  extWrenchSumMeasured_ = computeExternalWrenchSum<&ExternalWrench::measured>(realRobot(), measuredCoM_);
  
  if(c_.extWrench.subtractMeasuredValue)
  {
    Eigen::Vector3d extZMP = mc_rbdyn::zmp(extWrenchSumMeasured_, zmpFrame_);
    modifiedZMP_ -= extZMP;
  }
  else
  {
    Eigen::Vector3d extZMP = mc_rbdyn::zmp(extWrenchSumTarget_, zmpFrame_);
    modifiedZMP_ -= extZMP;
  }
#endif
  
  distribZMP_ = modifiedZMP_;
  
  std::cout << "comError = " << comError.transpose() << std::endl;
  std::cout << "comTarget = " << comTarget_.transpose() << std::endl;
  std::cout << "measuredCoM = " << measuredCoM_.transpose() << std::endl;

  std::cout << "comdError = " << comdError.transpose() << std::endl;
  std::cout << "comdTarget = " << comdTarget_.transpose() << std::endl;
  std::cout << "measuredCoMd = " << measuredCoMd_.transpose() << std::endl;

  std::cout << "comddTarget = " << comddTarget_.transpose() << std::endl;
  
  std::cout << "dcmError = " << dcmError_.transpose() << std::endl;
  std::cout << "dcmVelError = " << dcmVelError_.transpose() << std::endl;
  std::cout << "dcmErrorSum = " << dcmErrorSum_.transpose() << std::endl;
}

void DCMStabilizerTask::distributeWrench()
{
  const sva::PTransformd & X_0_rc = footTasks_.at(ContactState::Right)->targetPose();
  const sva::PTransformd & X_0_lc = footTasks_.at(ContactState::Left)->targetPose();
  
  Eigen::Matrix3d Rfoot;
  bool zmp_limit_over_prev = zmp_limit_over_;
  zmp_limit_over_ = calcForceDistributionRatio(Rfoot);
  distribWrench_.force() = mass_ * (comddTarget_ + constants::gravity);
  distribWrench_.moment() = distribZMP_.cross(distribWrench_.force());
  
#if 1
  std::cout << "modifiedZMP = " << modifiedZMP_.transpose() << std::endl;
  std::cout << "distibZMP = " << distribZMP_.transpose() << std::endl;
#endif
  
  if( zmp_limit_over_ ){
    if( !zmp_limit_over_ && zmp_limit_over_prev ){
      std::cout << "recovery of zmp over at " << t_ << "[s]" << std::endl;
    }
    else if( zmp_limit_over_ && !zmp_limit_over_prev ){
      std::cout << "zmp over!! desired(" << modifiedZMP_.x() << "," << modifiedZMP_.y()
                << "), actual(" << distribZMP_.x() << "," << distribZMP_.y() << ") "
                << "since " << t_ << "[s]" << std::endl;
    }
  }
  
  if( zmp_limit_over_ ){
    uncompTorque_ = (distribZMP_ - modifiedZMP_).cross(distribWrench_.force());
  }
  else{
    uncompTorque_.setZero();
  }
  
  const Eigen::Vector3d & rfoot_pos = X_0_rc.translation();
  const Eigen::Vector3d & lfoot_pos = X_0_lc.translation();
  const Eigen::Vector3d rfoot_force = distribWrench_.force() * leftFootRatio_;
  const Eigen::Vector3d lfoot_force = distribWrench_.force() * (1.0 - leftFootRatio_);
  Eigen::Vector3d tau(Rfoot *
                      ((distribZMP_ - rfoot_pos).cross(rfoot_force)
                       + (distribZMP_ - lfoot_pos).cross(lfoot_force)));
  Eigen::Vector3d rfoot_torque_local;
  Eigen::Vector3d lfoot_torque_local;
  if( inDoubleSupport() ){
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
  }
  else if( inContact(ContactState::Left) ){
    rfoot_torque_local.x() = 0.0;
    lfoot_torque_local.x() = tau.x();
  }
  else if( inContact(ContactState::Right) ){
    rfoot_torque_local.x() = tau.x();
    lfoot_torque_local.x() = 0.0;
  }
  
  rfoot_torque_local.y() = leftFootRatio_ * tau.y();
  lfoot_torque_local.y() = (1.0 - leftFootRatio_) * tau.y();
  rfoot_torque_local.z() = 0.0;
  lfoot_torque_local.z() = 0.0;
  
  Eigen::Vector3d rfoot_torque = Rfoot.transpose() * rfoot_torque_local;
  Eigen::Vector3d lfoot_torque = Rfoot.transpose() * lfoot_torque_local;
  
  std::cout << "rfoot_force = " << rfoot_force.transpose() << std::endl;
  std::cout << "lfoot_force = " << lfoot_force.transpose() << std::endl;
  std::cout << "rfoot_torque = " << rfoot_torque.transpose() << std::endl;
  std::cout << "lfoot_torque = " << lfoot_torque.transpose() << std::endl;
  
  rfoot_torque += rfoot_pos.cross(rfoot_force);
  lfoot_torque += lfoot_pos.cross(lfoot_force);
  
  const sva::ForceVecd w_r_0(rfoot_torque, rfoot_force);
  const sva::ForceVecd w_l_0(lfoot_torque, lfoot_force);
  
  footTasks_.at(ContactState::Right)->targetWrenchW(w_r_0);
  footTasks_.at(ContactState::Left)->targetWrenchW(w_l_0);
  
#if 0
  std::cout << "w_r_rc.force = " << w_r_rc.force().transpose() << std::endl;
  std::cout << "w_l_lc.force = " << w_l_lc.force().transpose() << std::endl;
  std::cout << "w_r_rc.moment = " << w_r_rc.moment().transpose() << std::endl;
  std::cout << "w_l_lc.moment = " << w_l_lc.moment().transpose() << std::endl;
#endif
#if 0
  std::cout << "X_0_rc trs = " << X_0_rc.translation().transpose() << std::endl;
  std::cout << "X_0_rc rot = " << X_0_rc.rotation() << std::endl;
  std::cout << "X_0_lc trs = " << X_0_lc.translation().transpose() << std::endl;
  std::cout << "X_0_lc rot = " << X_0_lc.rotation() << std::endl;
#endif
#if 0
  std::cout << "rfoot_force = " << rfoot_force.transpose() << std::endl;
  std::cout << "lfoot_force = " << lfoot_force.transpose() << std::endl;
  std::cout << "rfoot_torque = " << rfoot_torque.transpose() << std::endl;
  std::cout << "lfoot_torque = " << lfoot_torque.transpose() << std::endl;
#endif
}

void DCMStabilizerTask::updateFootImpedanceControl()
{
  if( inTheAir_)
  {
    std::cout << "Robot is in the air" << std::endl;
    
    for( auto & footT : footTasks_ )
    {
      footT.second->gains().wrench().vec(0., 0.);
      footT.second->gains().wrench().vec(c_.footWrench[ImpedanceType::Air]);
      footT.second->gains().M().vec(c_.footMass[ImpedanceType::Air]);
      footT.second->gains().D().vec(c_.footStiffness[ImpedanceType::Air]);
      footT.second->gains().K().vec(c_.footDamping[ImpedanceType::Air]);
    }
  }
  else{
    for(const auto contact : contacts_)
    {
      ImpedanceType WalkingPhase = std::get<0>(walking_phase_[contact.first]);
      double WalkingPhaseDuration = std::get<1>(walking_phase_[contact.first]);
      double WalkingCurrentTime = std::get<2>(walking_phase_[contact.first]);
      auto & footT = footTasks_[contact.first];
      footT->weight(c_.footWeight);
      footT->gains().wrench().vec(c_.footWrench[WalkingPhase]);
      footT->gains().M().vec(c_.footMass[WalkingPhase]);
      footT->gains().D().vec(c_.footStiffness[WalkingPhase]);
      footT->gains().K().vec(c_.footDamping[WalkingPhase]);

#if 0
      if( contact.first == ContactState::Right )
        std::cout << "Right leg" << std::endl;
      else
        std::cout << "Left leg" << std::endl;

      std::cout << "wrench = "
                << c_.footWrench[WalkingPhase].linear().transpose() << ","
                << c_.footWrench[WalkingPhase].angular().transpose() << std::endl;
      std::cout << "M = "
                << c_.footMass[WalkingPhase].linear().transpose() << ","
                << c_.footMass[WalkingPhase].angular().transpose() << std::endl;;
      std::cout << "D = "
                << c_.footDamping[WalkingPhase].linear().transpose() << ","
                << c_.footDamping[WalkingPhase].angular().transpose() << std::endl;;
      std::cout << "K = "
                << c_.footStiffness[WalkingPhase].linear().transpose() << ","
                << c_.footStiffness[WalkingPhase].angular().transpose() << std::endl;;
#endif
    }
  }
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
      unsigned robotIndex = robotIndexFromConfig(config, solver.robots(), "dcm_stabilizer");
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
