/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is extracted hmc2's stabilizer as part of
 * hmc2 <https://github.com/isri-aist/hmc2>
 */

#pragma once

#include <mc_filter/ExponentialMovingAverage.h>
#include <mc_filter/LeakyIntegrator.h>
#include <mc_filter/LowPass.h>
#include <mc_filter/StationaryOffset.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/OrientationTask.h>

#include <mc_rbdyn/lipm_stabilizer/DCMStabilizerConfiguration.h>
#include <mc_tasks/lipm_stabilizer/SupportPolygonManager.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>
#include <mc_tasks/lipm_stabilizer/ImpedanceType.h>

#include <state-observation/dynamics-estimators/lipm-dcm-estimator.hpp>

#include <Eigen/QR>
#include <eigen-quadprog/QuadProg.h>

namespace mc_tasks
{

namespace lipm_stabilizer
{

using ::mc_filter::utils::clamp;
using DCMStabilizerConfiguration = mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration;
using DCMStabilizerSafetyThresholds = mc_rbdyn::lipm_stabilizer::SafetyThresholdsForDCMStabilizer;
using DCMBiasEstimatorConfiguration = mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration;
using ExternalWrenchConfiguration = mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration;

/** Walking stabilization based on linear inverted pendulum tracking.
 *
 * Stabilization bridges the gap between the open-loop behavior of the
 * pendulum state reference (feedforward controls) and feedback read from
 * state estimation. In our case, feedback is done on the DCM of the LIPM:
 *
 * \f[
 *   \dot{\xi} = \dot{\xi}^{d} + k_p (\xi^d - \xi) + k_d (\dot{\xi}^d - \xi) + k_i \int (\xi^d - \xi)
 * \f]
 *
 * Which boils down into corresponding formulas for the CoP and CoM
 * acceleration targets.
 */
struct MC_TASKS_DLLAPI DCMStabilizerTask : public MetaTask
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  /**
   * @brief Creates a stabilizer meta task
   *
   * @param robots Robots on which the task acts
   * @param realRobots Corresponding real robot instances
   * @param robotIndex Index of the robot to stabilize
   * @param leftSurface Left foot surface name. Its origin should be the center of the foot sole
   * @param rightSurface Left foot surface name. Its origin should be the center of the foot sole
   * @param torsoBodyName Body name of the robot's torso (i.e a link above the
   * floating base)
   * @param dt Controller's timestep
   */
  DCMStabilizerTask(const mc_rbdyn::Robots & robots,
                    const mc_rbdyn::Robots & realRobots,
                    unsigned int robotIndex,
                    const std::string & leftSurface,
                    const std::string & rightSurface,
                    const std::string & torsoBodyName,
                    double dt);

  /**
   * @brief Creates a stabilizer meta task
   *
   * This constructor uses the stabilizer configuration in the robot module associated to the controlled robot. The
   * stabilizer is started with two feet contacts.
   *
   * @param robots Robots on which the task acts
   *
   * @param realRobots Corresponding real robots instance
   *
   * @param robotIndex Index of the robot
   *
   * @param dt Controller's timestep
   */
  DCMStabilizerTask(const mc_rbdyn::Robots & robots,
                    const mc_rbdyn::Robots & realRobots,
                    unsigned int robotIndex,
                    double dt);

  /**
   * @brief Resets the stabilizer tasks and parameters to their default configuration.
   *
   * Resets all tasks and errors/integrator/derivators to their initial
   * configuration. Configures the default stabilizer parameters from the robot
   * module.
   *
   * You can configure the stabilizer parameters (DCM tacking gains, task gains, etc) by calling
   * configure(const DCMStabilizerConfiguration & config)
   *
   * \note If you wish to reset the stabilizer from it's current configuration,
   * you can do so by storing its current configuration as accessed by config()
   * or commitedConfig() and set it explitely after calling reset by calling
   * configure(const DCMStabilizerConfiguration &);
   */
  void reset() override;

  /*! \brief Returns the task error
   *
   * Since the DCMStabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task, and the
   * sub-tasks evaluation depends on their order of insertion.
   */
  Eigen::VectorXd eval() const override;

  /*! \brief Returns the task velocity
   *
   * Since the DCMStabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task, and the
   * sub-tasks evaluation depends on their order of insertion.
   */
  Eigen::VectorXd speed() const override;

  /**
   * @brief Enables stabilizer
   *
   * This will reinitialize all integrators, and set the stabilizer gains
   * according to the last call to configure()
   */
  void enable();

  /** Disable all feedback components. */
  void disable();

  /** Configure stabilizer's parameters from a stabilizer's configuration object
   *
   * @param config Stabilizer configuration. Default values can be found in the
   * RobotModule, and modified from YAML configuration or manually.
   *
   * \see load(mc_solver::QPSolver &, const mc_rtc::Configuration &) to set
   * stabilizer targets and contacts from configuration
   */
  void configure(const DCMStabilizerConfiguration & config);

  /**
   * @brief Use the current configuration as the new default
   */
  void commitConfig();

  /*! \brief Load targets and contacts from configuration */
  void load(mc_solver::QPSolver &, const mc_rtc::Configuration & config) override;

  /**
   * @brief Get current stabilizer's configuration (including changes from
   * GUI/code)
   *
   * \see commitedConfig()
   */
  const DCMStabilizerConfiguration & config() const;

  /**
   * @brief Get last commited configuration
   *
   * Commited configuration is corresponds to the latest one set by calling commitConfig(), either manually or from the
   * GUI.
   *
   * \see config()
   */
  const DCMStabilizerConfiguration & commitedConfig() const;

  /**
   * Reset stabilizer configuration from last configuration set by configure()
   *
   * Does not include changes made from the GUI.
   */
  void reconfigure();

  /** Update QP task targets.
   *
   * This function is called once the reference has been updated.
   */
  void run();

  /** Configure foot tasks for contact at a given location, and add contacts to the solver.
   *
   * \note To use the stabilizer with dynamics constraint, you need to add the
   * corresponding mc_rbdyn::Contact to the solver and free the roll/pitch rotation and z translation (in contact
   * frame). This assumes the foot surfaces to have x pointing towards the front of the foot, and z from the ground up.
   *
   * \param solver The QP solver to which the contact tasks will be added. Note
   * that this method will not add contact constraints to the QPSolver. If you
   * wish to do so, bear in mind that for the stabilizer to work properly, the
   * contact's dofs along the x and y rotations and z translation need to be
   * free.
   */
  void setContacts(const ContactDescriptionVector & contacts);

  /**
   * @brief Helper to set contacts with a provided target pose
   *
   * @param contacts vectors of contacts defined by their ContactState and a
   * desired pose
   */
  void setContacts(const std::vector<std::pair<ContactState, sva::PTransformd>> & contacts);

  /** Helper to set contacts from the current surface pose
   *
   * @param contacts Contacts to add. Their pose will be determined set from the
   * realRobot estimate of the foot surface pose. Use with caution.
   *
   * \see void setContacts(mc_solver::QPSolver &, const std::vector<std::pair<ContactState, sva::PTransformd>> &);
   */
  void setContacts(const std::vector<ContactState> & contacts);

  /**
   * @brief Projected pose of the ankle frame in the contact frame.
   *
   * @param s Contact for which the frame is requested
   *
   * @return The projected ankle frame expressed in world frame.
   */
  inline const sva::PTransformd & contactAnklePose(ContactState s) const
  {
    return contacts_.at(s).anklePose();
  }

  inline std::shared_ptr<mc_tasks::force::ImpedanceTask> footTask(ContactState s)
  {
    return footTasks_.at(s);
  }
  
  inline const std::string & footSurface(ContactState s) const
  {
    return footTasks_.at(s)->surface();
  }

  /**
   * @brief Interpolation paremeter between left and right foot
   *
   * @return Left foot ratio between [0,1]
   */
  inline double leftFootRatio() const noexcept
  {
    return leftFootRatio_;
  }

  /**
   * @brief computes the anchorFrame compatible with the state observers
   * (e.g KinematicInertial)
   *
   * @param robot Robot from which the frame will be computed
   *
   * @return Anchor frame in-between the feet according to leftFootRatio()
   */
  sva::PTransformd anchorFrame(const mc_rbdyn::Robot & robot) const;

  /** Provides a static target to the stabilizer.
   * - CoM target : user-provided
   * - CoM velocity target: zero (static)
   * - CoM acceleration target: zero (static)
   * - ZMP: computed under the CoM
   *
   * @param com desired com position
   *
   * @param zmpHeight[=0] optional height of the ZMP plane (eg contacts), used to compute the
   * pendulums omega.
   *
   * \see target for dynamic motions.
   */
  void staticTarget(const Eigen::Vector3d & com, double zmpHeight = 0);

  /**
   * @brief Provides a dynamic target to the stabilizer.
   *
   * Note that this target should be updated at each iteration and provide a
   * dynamically-consistent trajectory. This would typically be generated by a
   * compatible Model Preview Controller.
   *
   * See https://github.com/jrl-umi3218/lipm_walking_controller for example in the context of walking.
   *
   * @param com Desired CoM position
   * @param comd Desired CoM velocity
   * @param comdd Desired CoM acceleration
   * @param zmp Desired ZMP
   * @param zmpd Desired ZMP velocity (can be omitted when zmpdGain in DCMStabilizerConfiguration is zero)
   *
   * \see staticTarget for a helper to define the stabilizer target when the CoM
   * is static
   */
  void target(const Eigen::Vector3d & com,
              const Eigen::Vector3d & comd,
              const Eigen::Vector3d & comdd,
              const Eigen::Vector3d & zmp,
              const Eigen::Vector3d & zmpd = Eigen::Vector3d::Zero());

  /**
   * @brief Set the wrench that the robot expects to receive from the external contacts.
   *
   * Change the configurations in ExternalWrenchConfiguration to handle external wrenches because external wrenches are
   * ignored by default. \see ExternalWrenchConfiguration
   *
   * @param surfaceNames Names of the surface to which the external wrench is applied
   * @param targetWrenches Target (expected) external wrenches
   * @param gains Gains of measured external wrenches
   */
  void setExternalWrenches(const std::vector<std::string> & surfaceNames,
                           const std::vector<sva::ForceVecd> & targetWrenches,
                           const std::vector<sva::MotionVecd> & gains);

  inline const Eigen::Vector3d & measuredDCM() noexcept
  {
    return measuredDCM_;
  }

  inline const Eigen::Vector3d & measuredZMP() noexcept
  {
    return measuredZMP_;
  }

  inline const Eigen::Vector3d & measuredCoM() noexcept
  {
    return measuredCoM_;
  }

  inline const Eigen::Vector3d & measuredCoMd() noexcept
  {
    return measuredCoM_;
  }
  
  inline void walkingPhase(ContactState state, ImpedanceType impType, double walkingPhase, double currentTime)
  {
    std::cout << "setWalkingPhase:" << (state==ContactState::Right ? "Right" : "Left") << " ";
    if( impType == mc_tasks::lipm_stabilizer::ImpedanceType::Swing )
      std::cout << "Swing" << std::endl;
    else if( impType == mc_tasks::lipm_stabilizer::ImpedanceType::Stand )
      std::cout << "Stand" << std::endl;
    else if( impType == mc_tasks::lipm_stabilizer::ImpedanceType::SingleSupport )
      std::cout << "SingleSupport" << std::endl;
    else if( impType == mc_tasks::lipm_stabilizer::ImpedanceType::DoubleSupport )
      std::cout << "DoubleSupport" << std::endl;
    else if( impType == mc_tasks::lipm_stabilizer::ImpedanceType::Air )
      std::cout << "Air" << std::endl;
    
    walking_phase_[state] = std::make_tuple(impType, walkingPhase, currentTime);
  }
  
  inline bool inContact(ContactState state) const noexcept
  {
    return (std::get<0>(walking_phase_.at(state)) != ImpedanceType::Swing) &&
      (std::get<0>(walking_phase_.at(state)) != ImpedanceType::Air);
  }
  
  inline bool inDoubleSupport() const noexcept
  {
    return inContact(ContactState::Right) & inContact(ContactState::Left);
  }

  inline const mc_rbdyn::Robot & robot() const noexcept
  {
    return robots_.robot(robotIndex_);
  }

  inline const mc_rbdyn::Robot & realRobot() const noexcept
  {
    return realRobots_.robot(robotIndex_);
  }

  /**
   * @name Setters to reconfigure the stabilizer online
   *
   * Setters for the main parameters of the stabilizer.
   * For safety purposes, values are clamped against the maximum values defined
   * by the stabilizer configuration.
   *
   * \see DCMStabilizerConfiguration for details on each
   * of these parameters.
   *
   * \see config() Access the current stabilizer configuration values
   * \see commitConfig() Make the current configuration the new default
   * @{
   */
  inline void torsoPitch(double pitch) noexcept
  {
    c_.torsoPitch = pitch;
  }

  inline void torsoWeight(double weight) noexcept
  {
    c_.torsoWeight = weight;
    torsoTask_->weight(c_.torsoWeight);
  }

  inline void torsoStiffness(double stiffness) noexcept
  {
    c_.torsoStiffness = stiffness;
    torsoTask_->stiffness(stiffness);
  }

  inline void pelvisWeight(double weight) noexcept
  {
    c_.pelvisWeight = weight;
    pelvisTask_->weight(c_.pelvisWeight);
  }

  inline void pelvisStiffness(double stiffness) noexcept
  {
    c_.pelvisStiffness = stiffness;
    pelvisTask_->stiffness(stiffness);
  }

  inline void dcmGains(double p, double i, double d) noexcept
  {
    c_.dcmPropGain = clamp(p, 0., c_.safetyThresholds.MAX_DCM_P_GAIN);
    c_.dcmIntegralGain = clamp(i, 0., c_.safetyThresholds.MAX_DCM_I_GAIN);
    c_.dcmDerivGain = clamp(d, 0., c_.safetyThresholds.MAX_DCM_D_GAIN);
  }

  inline void dcmDerivatorTimeConstant(double dcmDerivatorTimeConstant) noexcept
  {
    c_.dcmDerivatorTimeConstant = dcmDerivatorTimeConstant;
    dcmDerivator_.timeConstant(dcmDerivatorTimeConstant);
  }
  
  inline void extWrenchSumLowPassCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.extWrenchSumLowPassCutoffPeriod = cutoffPeriod;
    extWrenchSumLowPass_.cutoffPeriod(cutoffPeriod);
  }
  
  inline void comOffsetLowPassCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.comOffsetLowPassCutoffPeriod = cutoffPeriod;
    comOffsetLowPass_.cutoffPeriod(cutoffPeriod);
  }

  inline void comOffsetLowPassCoMCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.comOffsetLowPassCoMCutoffPeriod = cutoffPeriod;
    comOffsetLowPassCoM_.cutoffPeriod(cutoffPeriod);
  }

  inline void comOffsetDerivatorTimeConstant(double timeConstant) noexcept
  {
    c_.extWrench.comOffsetDerivatorTimeConstant = timeConstant;
    comOffsetDerivator_.timeConstant(timeConstant);
  }

  inline void comWeight(double weight) noexcept
  {
    c_.comWeight = weight;
    comTask_->weight(weight);
  }

  inline void comStiffness(const Eigen::Vector3d & stiffness) noexcept
  {
    c_.comStiffness = stiffness;
    comTask_->stiffness(stiffness);
  }

  inline void footWeight(double weight) noexcept
  {
    c_.footWeight = weight;
    for(auto footT : footTasks_)
    {
      footT.second->weight(c_.footWeight);
    }
  }
  
  inline void footWrench(const ImpedanceType& impType, const sva::ImpedanceVecd & wrench, bool updateToTask = false) noexcept
  {
    c_.footWrench[impType] = wrench;
    if( updateToTask ){
      for(auto footT : footTasks_)
      {
        footT.second->gains().wrench().vec(wrench);
      }
    }
  }
  
  inline void footMass(const ImpedanceType& impType, const sva::ImpedanceVecd & mass, bool updateToTask = false) noexcept
  {
    c_.footMass[impType] = mass;
    if( updateToTask ){
      for(auto footT : footTasks_)
      {
        footT.second->gains().M().vec(mass);
      }
    }
  }
  
  inline void footStiffness(const ImpedanceType& impType, const sva::ImpedanceVecd & stiffness, bool updateToTask = false) noexcept
  {
    c_.footStiffness[impType] = stiffness;
    if( updateToTask ){
      for(auto footT : footTasks_)
      {
        footT.second->gains().K().vec(stiffness);
      }
    }
  }
  
  inline void footDamping(const ImpedanceType& impType, const sva::ImpedanceVecd & damping, bool updateToTask = false) noexcept
  {
    c_.footDamping[impType] = damping;
    if( updateToTask ){
      for(auto footT : footTasks_)
      {
        footT.second->gains().D().vec(damping);
      }
    }
  }
  
  inline void safetyThresholds(const DCMStabilizerSafetyThresholds & thresholds) noexcept
  {
    c_.safetyThresholds = thresholds;
    c_.clampGains();
    // only requried because we want to apply the new gains immediately
    //copAdmittance(c_.copAdmittance);
  }

  /**
   * @brief Changes the parameters of the DCM bias estimator.
   *
   * @param biasConfig Configuration parameters for the bias estimation
   */
  inline void dcmBiasEstimatorConfiguration(const DCMBiasEstimatorConfiguration & biasConfig) noexcept
  {
    auto & bc = c_.dcmBias;
    bc = biasConfig;
    dcmEstimator_.setBiasDriftPerSecond(bc.biasDriftPerSecondStd);
    dcmEstimator_.setDcmMeasureErrorStd(bc.dcmMeasureErrorStd);
    dcmEstimator_.setZmpMeasureErrorStd(bc.zmpMeasureErrorStd);
    dcmEstimator_.setBiasLimit(bc.biasLimit);
  }

  /** @brief Get parameters of the DCM bias estimator. */
  inline const DCMBiasEstimatorConfiguration & dcmBiasEstimatorConfiguration() const noexcept
  {
    return c_.dcmBias;
  }

  /**
   * @brief Changes the parameters for the external wrenches.
   *
   * @param extWrenchConfig Configuration parameters for the external wrenches
   */
  inline void externalWrenchConfiguration(const ExternalWrenchConfiguration & extWrenchConfig) noexcept
  {
    c_.extWrench = extWrenchConfig;
    comOffsetLowPass_.cutoffPeriod(c_.extWrench.comOffsetLowPassCutoffPeriod);
    comOffsetLowPassCoM_.cutoffPeriod(c_.extWrench.comOffsetLowPassCoMCutoffPeriod);
    comOffsetDerivator_.timeConstant(c_.extWrench.comOffsetDerivatorTimeConstant);
  }

  /** @brief Get the parameters for the external wrenches. */
  inline const ExternalWrenchConfiguration & externalWrenchConfiguration() const noexcept
  {
    return c_.extWrench;
  }

private:
  void dimWeight(const Eigen::VectorXd & dimW) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  /**
   * @brief Add contact to the task (set tasks target, update support area).
   * This function does not immediately add contact tasks to the solver, this
   * will be done by update() when the task is added to the solver.
   */
  void addContact(ContactState contactState, const internal::Contact & contact);

  /** Check whether the robot is in the air. */
  void checkInTheAir();
  
  /** Computes the ratio of force distribution between the feet based on
   * the reference ZMP and contact ankle positions.
   */
  bool calcForceDistributionRatio(Eigen::Matrix3d& Rfoot);
  
  /** Update real-robot state.
   *
   * \param com Position of the center of mass.
   *
   * \param comd Velocity of the center of mass.
   */
  void updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd);
  
  void setDCMGainsFromPoleAssignment(double alpha, double beta, double gamma, double omega_zmp);
  
  /** Compute desired wrench based on DCM error. */
  void computeDesiredWrench();

  /** Distribute a desired wrench in double support.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   */
  void distributeWrench();
  
  /** set admittance gains:mass, damping and stiffness for each foot. */
  void setFootImpedanceGains();
  
  /** register force sensors on contact foot. */
  void updateFootSensors();
  
  /** Apply foot admittance control for each foot. */
  void updateFootImpedanceControl();
  
  /** Update ZMP frame from contact state. */
  void updateZMPFrame();
  
  /** @brief External wrench. */
  struct ExternalWrench
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// Target (expected) external wrench
    sva::ForceVecd target;
    /// Measured external wrench
    sva::ForceVecd measured;
    /// Gain of measured external wrench
    sva::MotionVecd gain;
    /// Name of the surface to which the external wrench is applied
    std::string surfaceName;
  };

  /** @brief Compute the CoM offset and the sum wrench from the external wrenches.
   *
   *  @tparam TargetOrMeasured Change depending on the used wrenches
   *  @param robot Robot used to transform surface wrenches (control robot or real robot)
   */
  template<sva::ForceVecd ExternalWrench::*TargetOrMeasured>
  Eigen::Vector3d computeCoMOffset(const mc_rbdyn::Robot & robot) const;

  /** @brief Compute the sum of external wrenches.
   *
   *  @tparam TargetOrMeasured Change depending on the used wrenches
   *  @param robot Robot used to transform surface wrenches (control robot or real robot)
   *  @param com Robot CoM
   */
  template<sva::ForceVecd ExternalWrench::*TargetOrMeasured>
  sva::ForceVecd computeExternalWrenchSum(const mc_rbdyn::Robot & robot, const Eigen::Vector3d & com) const;

  /** @brief Compute the position, force, and moment of the external contacts in the world frame.
   *
   *  @param [in] robot Robot (control robot or real robot)
   *  @param [in] surfaceName Surface name
   *  @param [in] surfaceWrench Surface wrench
   *  @param [out] pos Position of the external contact in the world frame
   *  @param [out] force Force of the external contact in the world frame
   *  @param [out] moment Moment of the external contact in the world frame
   */
  void computeExternalContact(const mc_rbdyn::Robot & robot,
                              const std::string & surfaceName,
                              const sva::ForceVecd & surfaceWrench,
                              Eigen::Vector3d & pos,
                              Eigen::Vector3d & force,
                              Eigen::Vector3d & moment) const;

  /* Task-related properties */
protected:
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;
  void update(mc_solver::QPSolver &) override;

  /** Log stabilizer entries.
   *
   * \param logger Logger.
   */
  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  /**
   * @brief Actual configuration of the stabilizer.
   * Called when reconfigure_ is true
   *
   * @param solver Solver to which this task has been added
   */
  void configure_(mc_solver::QPSolver & solver);

  /** Ensures that the configuration is valid */
  void checkConfiguration(const DCMStabilizerConfiguration & config);

protected:
  /**
   * @brief Workaround a C++11 standard bug: no specialization of the hash
   * functor exists for enum types.
   * Fixed in GCC 6.1 and clang's libc++ in 2013
   *
   * See http://www.open-std.org/jtc1/sc22/wg21/docs/lwg-defects.html#2148
   */
  struct EnumClassHash
  {
    template<typename T>
    std::size_t operator()(T t) const
    {
      return static_cast<std::size_t>(t);
    }
  };
  
  std::unordered_map<ContactState,
    internal::Contact,
    EnumClassHash,
    std::equal_to<ContactState>,
    Eigen::aligned_allocator<std::pair<const ContactState, internal::Contact>>>
    contacts_;
  
  std::unordered_map<ContactState,
    std::tuple<ImpedanceType, double, double>,
    EnumClassHash>  walking_phase_;
  
  /*! \brief Tasks for foot. */
  std::unordered_map<ContactState,
    std::shared_ptr<mc_tasks::force::ImpedanceTask>,
    EnumClassHash> footTasks_;
  
  std::vector<ContactState> addContacts_; /**< Contacts to add to the QPSolver when the task is inserted */
  std::vector<std::string> footSensors_; /** Force sensors corresponding to established contacts */
  
  std::vector<std::vector<Eigen::Vector3d>> supportPolygons_; /**< For GUI display */
  std::shared_ptr<computational_geometry::SupportPolygonManager<ContactState> > spm_;
  Eigen::Vector2d supportMin_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d supportMax_ = Eigen::Vector2d::Zero();
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::OrientationTask> pelvisTask_; /**< Pelvis orientation task */
  std::shared_ptr<mc_tasks::OrientationTask> torsoTask_; /**< Torso orientation task */
  const mc_rbdyn::Robots & robots_;
  const mc_rbdyn::Robots & realRobots_;
  unsigned int robotIndex_;

  /** Stabilizer targets */
  Eigen::Vector3d comTargetRaw_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comddTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmTarget_ = Eigen::Vector3d::Zero();
  double omega_;

  double t_ = 0.; /**< Time elapsed since the task is running */

protected:
  /**< Default (user-provided) configuration for the stabilizer. This configuration is superseeded by the parameters set
   * in the GUI */
  DCMStabilizerConfiguration defaultConfig_;
  /**< Last valid stabilizer configuration. */
  DCMStabilizerConfiguration lastConfig_;
  /**< Online stabilizer configuration, can be set from the GUI. Defaults to defaultConfig_ */
  DCMStabilizerConfiguration c_;
  /**< Whether the stabilizer needs to be reconfigured at the next
   * update(solver) call */
  bool reconfigure_ = true;
  bool enabled_ = true; /** Whether the stabilizer is enabled */
  
  Eigen::Vector3d dcmErrorSum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmVelError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredDCM_ = Eigen::Vector3d::Zero(); /// Measured DCM (only used for logging)
  Eigen::Vector3d measuredDCMUnbiased_ = Eigen::Vector3d::Zero(); /// DCM unbiased (only used for logging)
  sva::ForceVecd measuredNetWrench_ = sva::ForceVecd::Zero();
  
  bool zmp_limit_over_ = false;
  
  /**
   * Filtering of the divergent component of motion (DCM)
   * and estimation of a bias betweeen the DCM and the corresponding zero moment point for a linearized inverted
   * pendulum model.
   */
  stateObservation::LipmDcmEstimator dcmEstimator_;
  /**< Whether the estimator needs to be reset (robot in the air, initialization) */
  bool dcmEstimatorNeedsReset_ = true;
  
  /** @name Members related to stabilization in the presence of external wrenches
   *
   *  Adding an offset to the CoM for the predictable / measurable external wrenches on the robot surface.
   *  @{
   */
  std::vector<ExternalWrench> extWrenches_;
  sva::ForceVecd extWrenchSumTarget_ = sva::ForceVecd::Zero(); /**< Sum of target (expected) external wrenches */
  sva::ForceVecd extWrenchSumMeasured_ = sva::ForceVecd::Zero(); /**< Sum of measured external wrenches */
  Eigen::Vector3d comOffsetTarget_ = Eigen::Vector3d::Zero(); /**< Target (expected) CoM offset */
  Eigen::Vector3d comOffsetMeasured_ = Eigen::Vector3d::Zero(); /**< Measured CoM offset */
  Eigen::Vector3d comOffsetErr_ = Eigen::Vector3d::Zero(); /**< CoM offset error */
  Eigen::Vector3d comOffsetErrCoM_ = Eigen::Vector3d::Zero(); /**< CoM offset error handled by CoM modification */
  Eigen::Vector3d comOffsetErrZMP_ = Eigen::Vector3d::Zero(); /**< CoM offset error handled by ZMP modification */
  mc_filter::LowPass<sva::ForceVecd>
    extWrenchSumLowPass_; /**< Low-pass filter of the sum of the measured external wrenches */
  mc_filter::LowPass<Eigen::Vector3d> comOffsetLowPass_; /**< Low-pass filter of CoM offset */
  mc_filter::LowPass<Eigen::Vector3d>
    comOffsetLowPassCoM_; /**< Low-pass filter of CoM offset to extract CoM modification */
  mc_filter::StationaryOffset<Eigen::Vector3d> comOffsetDerivator_; /**< Derivator of CoM offset */
  /** @} */
  
  mc_filter::StationaryOffset<Eigen::Vector3d> dcmDerivator_;
  bool inTheAir_ = false; /**< Is the robot in the air? */
  double dt_ = 0.005; /**< Controller cycle in [s] */
  double leftFootRatio_ = 0.5; /**< Weight distribution ratio (0: all weight on right foot, 1: all on left foot) */
  double mass_ = 38.; /**< Robot mass in [kg] */
  double runTime_ = 0.;
  
  sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero(); /**< Result of the force distribution QP */
  Eigen::Vector3d modifiedZMP_ = Eigen::Vector3d::Zero(); /**< zmpTarget + dcm controller */
  Eigen::Vector3d distribZMP_ =
    Eigen::Vector3d::Zero(); /**< ZMP corresponding to force distribution result (desired ZMP) */
  Eigen::Vector3d uncompTorque_ =
    Eigen::Vector3d::Zero(); /**< Uncompensated torque different from modifiedZMP_ and distribZMP_ */
  sva::PTransformd zmpFrame_ = sva::PTransformd::Identity(); /**< Frame in which the ZMP is computed */
};

extern template Eigen::Vector3d DCMStabilizerTask::computeCoMOffset<&DCMStabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &) const;
extern template Eigen::Vector3d DCMStabilizerTask::computeCoMOffset<&DCMStabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &) const;

extern template sva::ForceVecd DCMStabilizerTask::computeExternalWrenchSum<&DCMStabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;
extern template sva::ForceVecd DCMStabilizerTask::computeExternalWrenchSum<&DCMStabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;

} // namespace lipm_stabilizer
} // namespace mc_tasks
