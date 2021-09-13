/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/ImpedanceTask.h>

namespace mc_tasks
{

namespace force
{

/*! \brief Damping based wrench control of the end-effector.
 *
 *  WrenchErrorTask passes the following "compliance" position and orientation (i.e., \f$ p_c \f$)
 *  to the target of the SurfaceTransformTask, which is the base class of this class.
 *
 *  \f[
 *      M \Delta D \Delta \dot{p}_{cd} + K \Delta p_{cd} = K_f (f_m - f_d)
 *      {\rm where} \Delta p_{cd} = p_c - p_d
 *  \f]
 *  where \f$ p_* \f$ is the end-effector position and orientation, and \f$ f_* \f$ is the end-effector wrench.
 *  Subscripts \f$ d, c, m \f$ mean the desired, compliance, and measured values, respectively.
 *  \f$ M, D, K \f$ are the mass, damper, and spring parameters of the impedance, respectively.
 *  \f$ K_f \f$ is the wrench gain.
 *
 *  Desired values \f$ p_d, \dot{p}_d, \ddot{p}_d, f_d \f$ are given from the user.
 *  The measured value \f$ f_m \f$ is obtained from the sensor.
 *
 *  In the SurfaceTransformTask, the "IK" acceleration (i.e., \f$ \ddot{p}_{IK} \f$) are calculated
 *  and passed to the acceleration-level IK.
 *
 * \f[
 *     \ddot{p}_{IK} = \ddot{p}_{c} + K_d ( \dot{p}_c - \dot{p}_a ) + K_s ( p_c - p_a )
 * \f]
 *
 *  \f$ K_s, K_d \f$ are the stiffness and damping parameters, respectively.
 *  Subscripts \f$ a \f$ means actual value.
 *
 *  Reference:
 *    Bruno Siciliano and Luigi Villani, Robot Force Control, Springer, 1999
 *    https://www.springer.com/jp/book/9780792377337
 *
 */
struct MC_TASKS_DLLAPI WrenchErrorTask : public ImpedanceTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor.
   *
   * \param surfaceName Name of the surface frame to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  WrenchErrorTask(const std::string & surfaceName,
                  const mc_rbdyn::Robots & robots,
                  unsigned robotIndex,
                  double stiffness = 5.0,
                  double weight = 1000.0);
  
  /*! \brief Reset the task
   *
   * Set the target and compliance poses to the current surface, and reset the target and compliance
   * velocity and acceleration to zero.
   *
   */
  void reset() override;
  
  /*! \brief Set the wrench error in the world frame.
   * This function will convert the wrench from the world frame to the surface frame, and call targetWrench().
   *
   */
  void wrenchErrorW(const sva::ForceVecd & wrenchErrorW)
  {
    //const auto & X_0_s = robots.robot(rIndex).surfacePose(surfaceName);
    const auto & X_0_s = targetPoseW_;
    wrenchError(X_0_s.dualMul(wrenchErrorW));
  }

  /*! \brief Set the wrench error in the surface frame. */
  void wrenchError(const sva::ForceVecd & wrenchError)
  {
    wrenchError_ = wrenchError;
  }

  /*! \brief Get the relative acceleration through low-pass filter from target frame to compliance frame represented in the world frame. */
  const sva::MotionVecd & filteredDeltaCompAccel() const
  {
    return filteredDeltaCompAccelW_;
  }
    
  /*! \brief Get the cutoff period for the low-pass filter of compliance acceleration. */
  double cutoffPeriodAccel() const
  {
    return lowPassAccel_.cutoffPeriod();
  }

  /*! \brief Set the cutoff period for the low-pass filter of compliance acceleration. */
  void cutoffPeriodAccel(double cutoffPeriod)
  {
    lowPassAccel_.cutoffPeriod(cutoffPeriod);
  }
  
  /*! \brief Load parameters from a Configuration object. */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;
  
protected:
  sva::MotionVecd filteredDeltaCompAccelW_ = sva::MotionVecd::Zero();
  
  mc_filter::LowPass<sva::MotionVecd> lowPassAccel_;
  
  void update(mc_solver::QPSolver & solver) override;
  
  void addToSolver(mc_solver::QPSolver & solver) override;
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;
  
private:
  sva::ForceVecd wrenchError_ = sva::ForceVecd::Zero();
  
  using ImpedanceTask::targetWrenchW;
  using ImpedanceTask::targetWrench;
  using ImpedanceTask::targetWrench_;
  using ImpedanceTask::measuredWrench_;
};

} // namespace force

} // namespace mc_tasks
