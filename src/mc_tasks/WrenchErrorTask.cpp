/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/WrenchErrorTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

namespace force
{

WrenchErrorTask::WrenchErrorTask(const std::string & surfaceName,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: ImpedanceTask(surfaceName, robots, robotIndex, stiffness, weight), lowPassAccel_(0.005, 0.05)
{
  type_ = "wrench_error";
  name_ = "wrench_error_" + robots.robot(rIndex).name() + "_" + surfaceName;
}

void WrenchErrorTask::update(mc_solver::QPSolver & solver)
{
  double dt = solver.dt();
  
  // 2. Compute the compliance acceleration
  //sva::PTransformd T_0_s(surfacePose().rotation());
  sva::PTransformd T_0_s(targetPoseW_.rotation());
  // deltaCompVelW_ is represented in the world frame
  //   \frac{D}{M} \Delta \dot{p}_{cd} = - \frac{K}{M} \Delta p_{cd})
  //   + \frac{K_f}{M} (f_m - f_d) where \Delta p_{cd} = p_c - p_d
  // See the Constructor description for the definition of symbols
  
  sva::MotionVecd deltaCompVelW_prev(deltaCompVelW_);
  deltaCompVelW_ = T_0_s.invMul( // T_0_s.invMul transforms the MotionVecd value from surface to world frame
      sva::MotionVecd(
          // Compute in the surface frame because the impedance parameters and wrench gain are represented in the
          // surface frame
          gains().D().vector().cwiseInverse().cwiseProduct(
              // T_0_s transforms the MotionVecd value from world to surface frame
              - gains().K().vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
              + gains().wrench().vector().cwiseProduct(wrenchError_.vector()))));
  deltaCompAccelW_ = (deltaCompVelW_ - deltaCompVelW_prev) / dt;
  lowPassAccel_.update(deltaCompAccelW_);
  filteredDeltaCompAccelW_ = lowPassAccel_.eval();
  
  if(deltaCompAccelW_.linear().norm() > deltaCompAccelLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompAccel limited from {} to {}", filteredDeltaCompAccelW_.linear().norm(),
                         deltaCompAccelLinLimit_);
    filteredDeltaCompAccelW_.linear().normalize();
    filteredDeltaCompAccelW_.linear() *= deltaCompAccelLinLimit_;
  }
  if(filteredDeltaCompAccelW_.angular().norm() > deltaCompAccelAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompAccel limited from {} to {}", filteredDeltaCompAccelW_.angular().norm(),
                         deltaCompAccelAngLimit_);
    filteredDeltaCompAccelW_.angular().normalize();
    filteredDeltaCompAccelW_.angular() *= deltaCompAccelAngLimit_;
  }
  
  // 3. Compute the compliance pose and velocity by time integral
  // 3.1 Integrate velocity to pose
  sva::PTransformd T_0_deltaC(deltaCompPoseW_.rotation());
  // Represent the compliance velocity and acceleration in the deltaCompliance frame and scale by dt
  sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * (deltaCompVelW_ + 0.5 * dt * deltaCompAccelW_));
  // Convert the angular velocity to the rotation matrix through AngleAxis representation
  Eigen::AngleAxisd aaDeltaCompVelIntegralC(Eigen::Quaterniond::Identity());
  double mvDeltaCompVelIntegralC_ang_norm = mvDeltaCompVelIntegralC.angular().norm();
  if(mvDeltaCompVelIntegralC_ang_norm > 1e-15)
  {
    aaDeltaCompVelIntegralC =
      Eigen::AngleAxisd(mvDeltaCompVelIntegralC_ang_norm, mvDeltaCompVelIntegralC.angular().normalized());
  }
  sva::PTransformd deltaCompVelIntegral(
      // Rotation matrix is transposed because sva::PTransformd uses the left-handed coordinates
      aaDeltaCompVelIntegralC.toRotationMatrix().transpose(), mvDeltaCompVelIntegralC.linear());
  // Since deltaCompVelIntegral is multiplied by deltaCompPoseW_, it must be represented in the deltaCompliance frame
  deltaCompPoseW_ = deltaCompVelIntegral * deltaCompPoseW_;
  //deltaCompPoseW_ = deltaCompPoseW_ * deltaCompVelIntegral;
  
  if(deltaCompVelW_.linear().norm() > deltaCompVelLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompVel limited from {} to {}", deltaCompVelW_.linear().norm(),
                         deltaCompVelLinLimit_);
    deltaCompVelW_.linear().normalize();
    deltaCompVelW_.linear() *= deltaCompVelLinLimit_;
  }
  if(deltaCompVelW_.angular().norm() > deltaCompVelAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompVel limited from {} to {}", deltaCompVelW_.angular().norm(),
                         deltaCompVelLinLimit_);
    deltaCompVelW_.angular().normalize();
    deltaCompVelW_.angular() *= deltaCompVelAngLimit_;
  }
  
  if(deltaCompPoseW_.translation().norm() > deltaCompPoseLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompPose limited from {} to {}", deltaCompPoseW_.translation().norm(),
                         deltaCompPoseLinLimit_);
    deltaCompPoseW_.translation().normalize();
    deltaCompPoseW_.translation() *= deltaCompPoseLinLimit_;
  }
  Eigen::AngleAxisd aaDeltaCompRot(deltaCompPoseW_.rotation());
  if(aaDeltaCompRot.angle() > deltaCompPoseAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompPose limited from {} to {}", aaDeltaCompRot.angle(), deltaCompPoseAngLimit_);
    aaDeltaCompRot.angle() = deltaCompPoseAngLimit_;
    deltaCompPoseW_.rotation() = aaDeltaCompRot.toRotationMatrix();
  }
  
  // 4. Update deltaCompPoseW_ in hold mode (See the hold method documentation for more information)
  if(hold_)
  {
    // Transform to target pose frame (see compliancePose implementation)
    sva::PTransformd T_0_d(targetPoseW_.rotation());
    // The previous compliancePose() is stored in SurfaceTransformTask::target()
    //deltaCompPoseW_ = T_0_d.inv() * SurfaceTransformTask::target() * targetPoseW_.inv() * T_0_d;
    deltaCompPoseW_ = targetPoseW_.inv() * T_0_d;
  }
  
  // 5. Set compliance values to the targets of SurfaceTransformTask
  SurfaceTransformTask::refAccel(T_0_s * (targetAccelW_ + filteredDeltaCompAccelW_)); // represented in the surface frame
  SurfaceTransformTask::refVelB(T_0_s * (targetVelW_ + deltaCompVelW_)); // represented in the surface frame
  SurfaceTransformTask::target(compliancePose()); // represented in the world frame
}

void WrenchErrorTask::reset()
{
  ImpedanceTask::reset();
  lowPassAccel_.reset(sva::MotionVecd::Zero());
}

void WrenchErrorTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  ImpedanceTask::load(solver, config);
}

void WrenchErrorTask::addToSolver(mc_solver::QPSolver & solver)
{
  ImpedanceTask::addToSolver(solver);
  lowPassAccel_.dt(solver.dt());
}

void WrenchErrorTask::addToLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::addToLogger(logger);
  
  // impedance parameters
  logger.addLogEntry(name_ + "_gains_D", this, [this]() -> const sva::ImpedanceVecd & { return gains().D().vec(); });
  logger.addLogEntry(name_ + "_gains_K", this, [this]() -> const sva::ImpedanceVecd & { return gains().K().vec(); });
  logger.addLogEntry(name_ + "_gains_wrench", this,
                     [this]() -> const sva::ImpedanceVecd & { return gains().wrench().vec(); });
  
  // compliance values
  MC_RTC_LOG_HELPER(name_ + "_deltaCompliancePose", deltaCompPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceVel", deltaCompVelW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceAccel", filteredDeltaCompAccelW_);
  
  // target values
  MC_RTC_LOG_HELPER(name_ + "_targetPose", targetPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_targetVel", targetVelW_);
  MC_RTC_LOG_HELPER(name_ + "_targetAccel", targetAccelW_);
  
  // wrench
  MC_RTC_LOG_HELPER(name_ + "_wrenchError", wrenchError_);
  
  MC_RTC_LOG_HELPER(name_ + "_hold", hold_);
}
  
void WrenchErrorTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // Don't add SurfaceTransformTask because the target of SurfaceTransformTask should not be set by user
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
  
  gui.addElement({"Tasks", name_},
                 // pose
                 mc_rtc::gui::Transform(
                                        "targetPose", [this]() -> const sva::PTransformd & { return this->targetPose(); },
                                        [this](const sva::PTransformd & pos) { this->targetPose(pos); }),
                 mc_rtc::gui::Transform("compliancePose", [this]() { return this->compliancePose(); }),
                 mc_rtc::gui::Transform("pose", [this]() { return this->surfacePose(); }),
                 // wrench
                 mc_rtc::gui::ArrayLabel("wrenchError", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->wrenchError_.vector(); }),
                 mc_rtc::gui::Checkbox(
                                       "hold", [this]() { return hold_; }, [this]() { hold_ = !hold_; }));
  gui.addElement({"Tasks", name_, "Impedance gains"},
                 mc_rtc::gui::ArrayInput(
                                         "damper", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() -> const sva::ImpedanceVecd & { return gains().damper().vec(); },
                                         [this](const Eigen::Vector6d & a) { gains().damper().vec(a); }),
                 mc_rtc::gui::ArrayInput(
                                         "spring", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() -> const sva::ImpedanceVecd & { return gains().spring().vec(); },
                                         [this](const Eigen::Vector6d & a) { gains().spring().vec(a); }),
                 mc_rtc::gui::ArrayInput(
                                         "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() -> const sva::ImpedanceVecd & { return gains().wrench().vec(); },
                                         [this](const Eigen::Vector6d & a) { gains().wrench().vec(a); }));
}
  
} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "wrench",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::WrenchErrorTask>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "wrench");
      auto t = std::allocate_shared<mc_tasks::force::WrenchErrorTask>(Allocator{}, config("surface"), solver.robots(),
                                                                      robotIndex);
      t->reset();
      t->load(solver, config);
      return t;
    });
}
