#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/DCMStabilizerStandingState.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/DCMStabilizerTask.h>

namespace mc_control
{
namespace fsm
{

namespace constants = mc_rtc::constants;
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

void DCMStabilizerStandingState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("completion"))
  {
    hasCompletion_ = !config("completion").empty();
    config("completion")("dcmEval", dcmThreshold_);
  }
  config("optionalGUI", optionalGUI_);

  config_.load(config);
}

void DCMStabilizerStandingState::start(Controller & ctl)
{
  if(!config_.has("DCMStabilizerConfig"))
  {
    config_.add("DCMStabilizerConfig");
  }
  config_("DCMStabilizerConfig").add("type", "dcm_stabilizer");

  config_("stiffness", K_);
  D_ = config_("damping", 2 * std::sqrt(K_));

  // create stabilizer task from config
  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::DCMStabilizerTask>(
      ctl.solver(), config_("DCMStabilizerConfig"));
  robot_ = stabilizerTask_->robot().name();
  auto & robot = ctl.robot(robot_);
  anchorFrameFunction_ = config_("anchorFrameFunction", "KinematicAnchorFrame::" + robot_);
  ctl.solver().addTask(stabilizerTask_);

  // Initialize stabilizer targets. Defaults to current CoM/CoP
  config_("comHeight", stabilizerTask_->config().comHeight);
  // Reset linear inverted pendulum model, used here to compute stabilizer references
  double lambda = constants::GRAVITY / stabilizerTask_->config().comHeight;
  pendulum_.reset(lambda, robot.com(), robot.comVelocity(), robot.comAcceleration());
  if(config_.has("above"))
  {
    const std::string above = config_("above");
    if(above == "LeftAnkle")
    {
      targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation());
    }
    else if(above == "RightAnkle")
    {
      targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation());
    }
    else if(above == "CenterAnkles")
    {
      targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                                 stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                    .translation());
    }
    else if(above == "LeftSurface")
    {
      targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Left)).translation());
    }
    else if(above == "RightSurface")
    {
      targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Right)).translation());
    }
    else if(above == "CenterSurfaces")
    {
      targetCoP(sva::interpolate(ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Left)),
                                 ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Right)), 0.5)
                    .translation());
    }
    else if(robot.hasSurface(above))
    {
      targetCoP(robot.surfacePose(above).translation());
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DCMStabilizerStandingState] Requested standing above {} but this is neither one of the state target "
          "(LeftAnkle, RightAnkle, CenterAnkles, LeftSurface, RightSurface, CenterSurfaces), nor a valid robot surface "
          "name",
          above);
    }
  }
  else if(config_.has("com"))
  {
    targetCoM(config_("com"));
  }
  else
  {
    targetCoM(robot.com());
  }

  // Fixme: the stabilizer needs the observed state immediatly
  if(ctl.datastore().has(anchorFrameFunction_))
  {
    mc_rtc::log::warning("[{}] a datastore callback for \"{}\" already exist on the datastore, using it instead",
                         name(), anchorFrameFunction_);
    ownsAnchorFrameCallback_ = false;
  }
  else
  {
    ctl.datastore().make_call(anchorFrameFunction_,
                              [this](const mc_rbdyn::Robot & robot) { return stabilizerTask_->anchorFrame(robot); });
    ownsAnchorFrameCallback_ = true;
  }

  if(optionalGUI_ && stabilizerTask_->inDoubleSupport())
  {
    ctl.gui()->addElement(
        {"FSM", name(), "Move"}, mc_rtc::gui::ElementsStacking::Horizontal,
        mc_rtc::gui::Button(
            "Left foot", [this]() { targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation()); }),
        mc_rtc::gui::Button("Center",
                            [this]() {
                              targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                                                         stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                                            .translation());
                            }),
        mc_rtc::gui::Button("Right foot", [this]() {
          targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation());
        }));
    ctl.gui()->addElement(
        {"FSM", name(), "Move"},
        mc_rtc::gui::ArrayInput("CoM Target", [this]() -> const Eigen::Vector3d & { return comTarget_; },
                                [this](const Eigen::Vector3d & com) { targetCoM(com); }),
        mc_rtc::gui::ArrayInput("Move CoM", []() -> Eigen::Vector3d { return Eigen::Vector3d::Zero(); },
                                [this](const Eigen::Vector3d & com) { targetCoM(comTarget_ + com); }));
  }

  ctl.gui()->addElement(
      {"FSM", name(), "Gains"},
      mc_rtc::gui::NumberInput("CoM stiffness", [this]() { return K_; }, [this](const double & s) { K_ = s; }),
      mc_rtc::gui::NumberInput("CoM damping", [this]() { return D_; }, [this](const double & d) { D_ = d; }),
      mc_rtc::gui::NumberInput("CoM stiffness & damping", [this]() { return K_; },
                               [this](const double & g) {
                                 K_ = g;
                                 D_ = 2 * std::sqrt(K_);
                               }));

#define LOG_MEMBER(NAME, MEMBER) MC_RTC_LOG_HELPER(name() + NAME, MEMBER)
  auto & logger = ctl.logger();
  LOG_MEMBER("_stiffness", K_);
  LOG_MEMBER("_damping", D_);
  LOG_MEMBER("_targetCoM", comTarget_);
  LOG_MEMBER("_targetCoP", copTarget_);
#undef LOG_MEMBER

  // Provide accessor callbacks on the datastore
  ctl.datastore().make_call("DCMStabilizerStandingState::getCoMTarget",
                            [this]() -> const Eigen::Vector3d & { return comTarget_; });
  ctl.datastore().make_call("DCMStabilizerStandingState::setCoMTarget",
                            [this](const Eigen::Vector3d & com) { this->targetCoM(com); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setStiffness", [this](double K) { this->K_ = K; });
  ctl.datastore().make_call("DCMStabilizerStandingState::setDamping", [this](double D) { this->D_ = D; });
  ctl.datastore().make_call("DCMStabilizerStandingState::getStiffness", [this]() { return K_; });
  ctl.datastore().make_call("DCMStabilizerStandingState::getDamping", [this]() { return D_; });
  ctl.datastore().make_call(
      "DCMStabilizerStandingState::getConfiguration",
      [this]() -> mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration { return stabilizerTask_->config(); });
  ctl.datastore().make_call(
      "DCMStabilizerStandingState::setConfiguration",
      [this](const mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration & conf) { stabilizerTask_->configure(conf); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setPelvisWeight",
                            [this](double w) { stabilizerTask_->pelvisWeight(w); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setPelvisStiffness",
                            [this](double s) { stabilizerTask_->pelvisStiffness(s); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setTorsoWeight",
                            [this](double w) { stabilizerTask_->torsoWeight(w); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setTorsoStiffness",
                            [this](double s) { stabilizerTask_->torsoStiffness(s); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setCoMWeight",
                            [this](double w) { stabilizerTask_->comWeight(w); });
  ctl.datastore().make_call("DCMStabilizerStandingState::setCoMStiffness",
                            [this](const Eigen::Vector3d & s) { stabilizerTask_->comStiffness(s); });
}

void DCMStabilizerStandingState::targetCoP(const Eigen::Vector3d & cop)
{
  comTarget_ = cop + Eigen::Vector3d{0., 0., stabilizerTask_->config().comHeight};
  copTarget_ = cop;
}

void DCMStabilizerStandingState::targetCoM(const Eigen::Vector3d & com)
{
  double copHeight = 0;
  if(stabilizerTask_->inDoubleSupport())
  {
    copHeight = (stabilizerTask_->contactAnklePose(ContactState::Left).translation().z()
                 + stabilizerTask_->contactAnklePose(ContactState::Right).translation().z())
                / 2;
  }
  else if(stabilizerTask_->inContact(ContactState::Left))
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Left).translation().z();
  }
  else
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Right).translation().z();
  }

  comTarget_ = com;
  copTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), copHeight};
}

bool DCMStabilizerStandingState::run(Controller & ctl)
{
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  Eigen::Vector3d n = constants::vertical;
  double lambda = n.dot(comdd + constants::gravity) / n.dot(com_ - copTarget_);
  Eigen::Vector3d zmp = com_ - (constants::gravity + comdd) / lambda;

  pendulum_.integrateIPM(zmp, lambda, ctl.timeStep);

  // Update stabilizer target
  stabilizerTask_->target(pendulum_.com(), pendulum_.comd(), pendulum_.comdd(), pendulum_.zmp());

  if(!hasCompletion_)
  {
    output("OK");
    return true;
  }
  const auto & dcm = stabilizerTask_->measuredDCM();
  if((((dcm - comTarget_).cwiseAbs() - dcmThreshold_).array() < 0.).all())
  {
    output("OK");
    return true;
  }
  return false;
}

void DCMStabilizerStandingState::teardown(Controller & ctl)
{
  ctl.solver().removeTask(stabilizerTask_);
  ctl.gui()->removeCategory({"FSM", name()});
  ctl.logger().removeLogEntries(this);

  ctl.datastore().remove("DCMStabilizerStandingState::getCoMTarget");
  ctl.datastore().remove("DCMStabilizerStandingState::setCoMTarget");
  ctl.datastore().remove("DCMStabilizerStandingState::getStiffness");
  ctl.datastore().remove("DCMStabilizerStandingState::setStiffness");
  ctl.datastore().remove("DCMStabilizerStandingState::getDamping");
  ctl.datastore().remove("DCMStabilizerStandingState::setDamping");
  ctl.datastore().remove("DCMStabilizerStandingState::getConfiguration");
  ctl.datastore().remove("DCMStabilizerStandingState::setConfiguration");
  ctl.datastore().remove("DCMStabilizerStandingState::setPelvisWeight");
  ctl.datastore().remove("DCMStabilizerStandingState::setPelvisStiffness");
  ctl.datastore().remove("DCMStabilizerStandingState::setTorsoWeight");
  ctl.datastore().remove("DCMStabilizerStandingState::setTorsoStiffness");
  ctl.datastore().remove("DCMStabilizerStandingState::setCoMWeight");
  ctl.datastore().remove("DCMStabilizerStandingState::setCoMStiffness");
  if(ownsAnchorFrameCallback_)
  {
    ctl.datastore().remove(anchorFrameFunction_);
  }
}

} // namespace fsm
} // namespace mc_control
EXPORT_SINGLE_STATE("DCMStabilizerStandingState", mc_control::fsm::DCMStabilizerStandingState)
