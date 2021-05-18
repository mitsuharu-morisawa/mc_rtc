/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/api.h>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_rbdyn/lipm_stabilizer/DCMBiasConfiguration.h>
#include <mc_rbdyn/lipm_stabilizer/ExternalWrenchConfiguration.h>
#include <mc_tasks/lipm_stabilizer/ImpedanceType.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

using impType = mc_tasks::lipm_stabilizer::ImpedanceType;

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/**
 * @brief Stabilizer safety thresholds
 *
 * The corresponding stabilization entries should be clamped within those limits
 *
 * \warning Developper note: Do not change the default thresholds here, it is likely
 * that robot modules and users do not override every single parameter value,
 * and modifying their default might have serious consequences.
 */
struct SafetyThresholdsForDCMStabilizer
{
  double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  double MAX_DCM_POLES = 30.; /**< Maximum DCM poles in [Hz] */
  double MAX_DCM_FLEXIBILITY = 100.; /**< Maximum DCM flexibility in [Hz] */
  double MIN_PRESSURE = 15.; /**< Minimum normal contact force, used to avoid low-pressure                                                     targets when close to contact switches. */
  /**< Minimum force for valid ZMP computation (throws otherwise) */
  double MIN_NET_TOTAL_FORCE_ZMP = 1.;
  
  void load(const mc_rtc::Configuration & config)
  {
    config("MAX_AVERAGE_DCM_ERROR", MAX_AVERAGE_DCM_ERROR);
    config("MAX_DCM_D_GAIN", MAX_DCM_D_GAIN);
    config("MAX_DCM_I_GAIN", MAX_DCM_I_GAIN);
    config("MAX_DCM_P_GAIN", MAX_DCM_P_GAIN);
    config("MAX_DCM_POLES", MAX_DCM_POLES);
    config("MAX_DCM_FLEXIBILITY", MAX_DCM_FLEXIBILITY);
    config("MIN_PRESSURE", MIN_PRESSURE);
    config("MIN_NET_TOTAL_FORCE_ZMP", MIN_NET_TOTAL_FORCE_ZMP);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("MAX_AVERAGE_DCM_ERROR", MAX_AVERAGE_DCM_ERROR);
    config.add("MAX_DCM_D_GAIN", MAX_DCM_D_GAIN);
    config.add("MAX_DCM_I_GAIN", MAX_DCM_I_GAIN);
    config.add("MAX_DCM_P_GAIN", MAX_DCM_P_GAIN);
    config.add("MAX_DCM_POLES", MAX_DCM_POLES);
    config.add("MAX_DCM_FLEXIBILITY", MAX_DCM_FLEXIBILITY);
    config.add("MIN_PRESSURE", MIN_PRESSURE);
    config.add("MIN_NET_TOTAL_FORCE_ZMP", MIN_NET_TOTAL_FORCE_ZMP);
    return config;
  }
};

} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
/**
 * @brief Read-write stabilizer safety thresholds from configuration
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::SafetyThresholdsForDCMStabilizer>
{
  static mc_rbdyn::lipm_stabilizer::SafetyThresholdsForDCMStabilizer load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::SafetyThresholdsForDCMStabilizer safety;
    safety.load(config);
    return safety;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::SafetyThresholdsForDCMStabilizer & safety)
  {
    return safety.save();
  }
};
} // namespace mc_rtc

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/**
 * @brief Configuration of the LIPMStabilizer. This configuration is meant to be
 * overriden from the RobotModule, and the user YAML configuration of the
 * stabilizer task.
 *
 * \note Developper note: Do not change the default gains here, it is likely
 * that robot modules and users do not override every single parameter value,
 * and modifying their default might have serious consequences.
 */
struct MC_RBDYN_DLLAPI DCMStabilizerConfiguration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SafetyThresholdsForDCMStabilizer safetyThresholds;
  
  double friction = 0.7; /**< Friction coefficient. Same for both feet */
  std::string leftFootSurface; /**< Surface name for the left foot. Origin should be at foot's center */
  std::string rightFootSurface; /**< Surface name for the right foot. Origin should be at foot's center */
  
  double dcmPropGain = 1.; /**< Proportional gain on DCM error */
  double dcmIntegralGain = 5.; /**< Integral gain on DCM error */
  double dcmDerivGain = 0.; /**< Derivative gain on DCM error */
  double dcmDerivatorTimeConstant = 150.0; /**< Time window for the stationary offset filter of the DCM derivator */
  bool hasDCMPoles = false; /**< if pole is set, this flag is true */
  Eigen::Vector3d dcmPoles = {12.0, 3.0, 1.0}; /**< pole assignment for dcm controller */
  double dcmFlexibility = 20.0; /**< flexiblity of dcm around support */
  
  std::vector<std::string> comActiveJoints; /**< Joints used by CoM IK task */
  Eigen::Vector3d comStiffness = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
  double comWeight = 1000.; /**< Weight of CoM IK task */
  double comHeight = 0.84; /**< Desired height of the CoM */
  
  std::string torsoBodyName; /**< Name of the torso body */
  double torsoPitch = 0; /**< Target world pitch angle for the torso */
  double torsoStiffness = 1000.; /**< Stiffness of the torso task. */
  double torsoWeight = 100.; /**< Weight of the torso task. Should be much lower than CoM and Contacts */
  double pelvisStiffness = 1000.; /**< Stiffness of the pelvis task. */
  double pelvisWeight = 100.; /**< Weight of the torso task. Should be much lower than CoM and Contacts */
  
  double cutoffRatio = 0.0; /**< cut off ratio for low pass filter depends on sampling period (cutoffPeriod = cutoffRatio / dt) */
  
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
  
  std::unordered_map<impType, sva::ImpedanceVecd, EnumClassHash> footWrench; /**< Wrench coefficients of Impedance tasks */
  std::unordered_map<impType, sva::ImpedanceVecd, EnumClassHash> footMass; /**< Mass coefficients of Impedance tasks */
  std::unordered_map<impType, sva::ImpedanceVecd, EnumClassHash> footDamping; /**< Damping coefficients of Impedance tasks */
  std::unordered_map<impType, sva::ImpedanceVecd, EnumClassHash> footStiffness; /**< Stiffness coefficients of Impedance tasks */
  sva::MotionVecd footMotionStiffness = {{1000., 1000., 1000.}, {1000., 1000., 1000.}}; /**< Stiffness coefficients of Impedance tasks for motion */
  double footWeight = 100000.; /**< Weight of contact IK tasks */
  
  DCMBiasEstimatorConfiguration dcmBias; /**< Parameters for the DCM bias estimation */
  
  ExternalWrenchConfiguration extWrench; /**< Parameters for the external wrenches */

  DCMStabilizerConfiguration() {
    setDefaultImpedanceGains();
  }
  
  DCMStabilizerConfiguration(const StabilizerConfiguration& lipmStabilizerConfig) {
    setDefaultParametersFromDefaultLIPMStabilizerConfiguration(lipmStabilizerConfig);
    setDefaultImpedanceGains();
  }
  
  DCMStabilizerConfiguration(const mc_rtc::Configuration & conf,
                             const StabilizerConfiguration& lipmStabilizerConfig)
  {
    setDefaultParametersFromDefaultLIPMStabilizerConfiguration(lipmStabilizerConfig);
    setDefaultImpedanceGains();
    load(conf);
  }

  void setDefaultParametersFromDefaultLIPMStabilizerConfiguration(const StabilizerConfiguration& lipmStabilizerConfig)
  {
    leftFootSurface = lipmStabilizerConfig.leftFootSurface;
    rightFootSurface = lipmStabilizerConfig.rightFootSurface;
    torsoBodyName = lipmStabilizerConfig.torsoBodyName;
    comHeight = lipmStabilizerConfig.comHeight;
    comActiveJoints = lipmStabilizerConfig.comActiveJoints;
    
    dcmPropGain = lipmStabilizerConfig.dcmPropGain;
    dcmIntegralGain = lipmStabilizerConfig.dcmIntegralGain;
    dcmDerivGain = lipmStabilizerConfig.dcmDerivGain;
    dcmDerivatorTimeConstant = lipmStabilizerConfig.dcmDerivatorTimeConstant;
  }
     
  void setDefaultImpedanceGains()
  {
    footMass[impType::Stand] = {{1, 1, 1}, {50, 50, 50}};
    footDamping[impType::Stand] = {{100, 100, 100}, {50*400, 50*400, 50*100}};
    footStiffness[impType::Stand] = {{0, 0, 0}, {0, 0, 0}};
    footWrench[impType::Stand] = {{1, 1, 0}, {0.1, 0.1, 0.1}};
    
    footMass[impType::DoubleSupport] = {{1, 1, 1}, {50, 50, 50}};
    footDamping[impType::DoubleSupport] = {{100, 100, 100}, {50*400, 50*400, 50*30}};
    footStiffness[impType::DoubleSupport] = {{0, 0, 400}, {0, 0, 0}};
    footWrench[impType::DoubleSupport] = {{1, 1, 0}, {0.1, 0.1, 0.1}};
    
    footMass[impType::SingleSupport] = {{1, 1, 1}, {100, 100, 100}};
    footDamping[impType::SingleSupport] = {{100, 100, 100}, {100*30, 100*30, 100*30}};
    footStiffness[impType::SingleSupport] = {{0, 0, 0}, {100*225, 100*225, 100*225}};
    footWrench[impType::SingleSupport] = {{1, 1, 0}, {0, 0, 0}};
    
    footMass[impType::Swing] = {{1, 1, 1}, {10, 10, 10}};
    footDamping[impType::Swing] = {{40, 40, 40}, {10*30, 10*30, 10*30}};
    footStiffness[impType::Swing] = {{400, 400, 400}, {10*225, 10*225, 10*225}};  
    footWrench[impType::Swing] = {{0, 0, 0}, {0, 0, 0}};
    
    footMass[impType::Air] = {{1, 1, 1}, {10, 10, 10}};
    footDamping[impType::Air] = {{30, 30, 30}, {10*30, 10*30, 10*30}};
    footStiffness[impType::Air] = {{100, 100, 100}, {10*100, 10*100, 10*100}};
    footWrench[impType::Air] = {{0, 0, 0}, {0, 0, 0}};
  }
  
  /**
   * @brief Checks that the chosen parameters are within the parameters defined
   * by the SafetyThresholds
   */
  void clampGains()
  {
    using ::mc_filter::utils::clampInPlaceAndWarn;
    const auto & s = safetyThresholds;
    if( hasDCMPoles ){
      clampInPlaceAndWarn(dcmPoles[0], 0., s.MAX_DCM_POLES, "DCM poles[0]");
      clampInPlaceAndWarn(dcmPoles[1], 0., s.MAX_DCM_POLES, "DCM poles[1]");
      clampInPlaceAndWarn(dcmPoles[2], 0., s.MAX_DCM_POLES, "DCM poles[2]");
      clampInPlaceAndWarn(dcmFlexibility, 0., s.MAX_DCM_FLEXIBILITY, "DCM flexilibty time constant");
    }
    else{
      clampInPlaceAndWarn(dcmDerivGain, 0., s.MAX_DCM_D_GAIN, "DCM deriv gain");
      clampInPlaceAndWarn(dcmIntegralGain, 0., s.MAX_DCM_I_GAIN, "DCM integral gain");
      clampInPlaceAndWarn(dcmPropGain, 0., s.MAX_DCM_P_GAIN, "DCM prop gain");
    }
  }
  
  void loadImpedanceGains(mc_rtc::Configuration& tasks, const std::string& impedance_str, impType impedance_type)
  {
    auto impedance_config = tasks("foot")(impedance_str);
    if(impedance_config.has("wrench"))
    {
      try
      {
        //double w = impedance_config("wrench");
        //footWrench[impedance_type] = sva::ImpedanceVecd({w, w, w}, {w, w, w});
        impedance_config("wrench", footWrench[impedance_type]);
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        footWrench[impedance_type] = impedance_config("wrench");
      }
    }
    if(impedance_config.has("mass"))
    {
      try
      {
        //double m = impedance_config("mass");
        //footMass[impedance_type] = sva::ImpedanceVecd({m, m, m}, {m, m, m});
        impedance_config("mass", footMass[impedance_type]);
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        footMass[impedance_type] = impedance_config("mass");
      }
    }
    if(impedance_config.has("damper"))
    {
      try
      {
        //double d = impedance_config("damper");
        //footDamping[impedance_type] = sva::ImpedanceVecd({d, d, d}, {d, d, d});
        impedance_config("damper", footDamping[impedance_type]);
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        footDamping[impedance_type] = impedance_config("damper");
      }
    }
    if(impedance_config.has("spring"))
    {
      try
      {
        //double k = impedance_config("spring");
        //footStiffness[impedance_type] = sva::ImpedanceVecd({k, k, k}, {k, k, k});
        impedance_config("spring", footStiffness[impedance_type]);
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        footStiffness[impedance_type] = impedance_config("spring");
      }
    }    
  }
  
  void load(const mc_rtc::Configuration & config)
  {
    if(config.has("safety_tresholds"))
    {
      safetyThresholds.load(config("safety_tresholds"));
    }
    
    config("leftFootSurface", leftFootSurface);
    config("rightFootSurface", rightFootSurface);
    config("torsoBodyName", torsoBodyName);
    
    if(config.has("dcm_tracking"))
    {
      std::cout << "load::pass1..." << std::endl;
      
      auto dcmTracking = config("dcm_tracking");
      if(dcmTracking.has("gains"))
      {
        std::cout << "load::pass2..." << std::endl;
        
        auto dcmGains = dcmTracking("gains");
        if( dcmGains.has("poles") )
        {
          std::cout << "load::pass3..." << std::endl;
        
          hasDCMPoles = true;
          dcmGains("poles", dcmPoles);
          if( !dcmGains.has("flexibility") )
          {
            mc_rtc::log::error_and_throw<std::runtime_error>("flexibility is not set for dcm_tracking");
          }
          else
            dcmGains("flexibility", dcmFlexibility);
          
          std::cout << "load::pass4..." << std::endl;
        }
        else
        {
          hasDCMPoles = false;
          dcmGains("prop", dcmPropGain);
          dcmGains("integral", dcmIntegralGain);
          dcmGains("deriv", dcmDerivGain);
        }
      }
      dcmTracking("derivator_time_constant", dcmDerivatorTimeConstant);
    }
    if(config.has("dcm_bias"))
    {
      dcmBias.load(config("dcm_bias"));
    }
    if(config.has("external_wrench"))
    {
      extWrench.load(config("external_wrench"));
    }
    if(config.has("tasks"))
    {
      auto tasks = config("tasks");
      if(tasks.has("com"))
      {
        tasks("com")("active_joints", comActiveJoints);
        tasks("com")("stiffness", comStiffness);
        tasks("com")("weight", comWeight);
        tasks("com")("height", comHeight);
      }

      if(tasks.has("torso"))
      {

        tasks("torso")("pitch", torsoPitch);
        tasks("torso")("stiffness", torsoStiffness);
        tasks("torso")("weight", torsoWeight);
      }

      if(tasks.has("pelvis"))
      {
        tasks("pelvis")("stiffness", pelvisStiffness);
        tasks("pelvis")("weight", pelvisWeight);
      }

      if(tasks.has("foot"))
      {
        if(tasks("foot").has("stand"))
          loadImpedanceGains(tasks, "stand", impType::Stand);
        if(tasks("foot").has("double-support"))
          loadImpedanceGains(tasks, "double-support", impType::DoubleSupport);
        if(tasks("foot").has("single-support"))
          loadImpedanceGains(tasks, "single-support", impType::SingleSupport);
        if(tasks("foot").has("swing"))
          loadImpedanceGains(tasks, "swing", impType::Swing);
        if(tasks("foot").has("air"))
          loadImpedanceGains(tasks, "air", impType::Air);
        
        tasks("foot")("stiffness", footMotionStiffness);
        tasks("foot")("weight", footWeight);
      }
    }
    if(config.has("forceSensorCutoffRatio"))
    {
      config("forceSensorCutoffRatio", cutoffRatio);
    }
  }
  
  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration conf;
    conf.add("safety_tresholds", safetyThresholds);

    conf.add("torsoBodyName", torsoBodyName);
    conf.add("leftFootSurface", leftFootSurface);
    conf.add("rightFootSurface", rightFootSurface);
    
    conf.add("dcm_tracking");
    conf("dcm_tracking").add("gains");
    if( hasDCMPoles ){
      conf("dcm_tracking")("gains").add("poles", dcmPoles);
      conf("dcm_tracking")("gains").add("flexibility", dcmFlexibility);
    }
    else{
      conf("dcm_tracking")("gains").add("prop", dcmPropGain);
      conf("dcm_tracking")("gains").add("integral", dcmIntegralGain);
      conf("dcm_tracking")("gains").add("deriv", dcmDerivGain);
    }
    conf("dcm_tracking").add("derivator_time_constant", dcmDerivatorTimeConstant);
    
    conf.add("dcm_bias", dcmBias);
    conf.add("external_wrench", extWrench);
    
    conf.add("tasks");
    conf("tasks").add("com");
    conf("tasks")("com").add("active_joints", comActiveJoints);
    conf("tasks")("com").add("stiffness", comStiffness);
    conf("tasks")("com").add("weight", comWeight);
    conf("tasks")("com").add("height", comHeight);
    
    conf("tasks").add("torso");
    conf("tasks")("torso").add("pitch", torsoPitch);
    conf("tasks")("torso").add("stiffness", torsoStiffness);
    conf("tasks")("torso").add("weight", torsoWeight);
    
    conf("tasks").add("pelvis");
    conf("tasks")("pelvis").add("stiffness", pelvisStiffness);
    conf("tasks")("pelvis").add("weight", pelvisWeight);
    
    conf("tasks").add("foot");
    conf("tasks")("foot").add("stand");
    conf("tasks")("foot")("stand").add("mass", footMass.at(impType::Stand));
    conf("tasks")("foot")("stand").add("damper", footDamping.at(impType::Stand));
    conf("tasks")("foot")("stand").add("spring", footStiffness.at(impType::Stand));
    conf("tasks")("foot")("stand").add("wrench", footWrench.at(impType::Stand));
    conf("tasks")("foot").add("double-support");
    conf("tasks")("foot")("double-support").add("mass", footMass.at(impType::DoubleSupport));
    conf("tasks")("foot")("double-support").add("damper", footDamping.at(impType::DoubleSupport));
    conf("tasks")("foot")("double-support").add("spring", footStiffness.at(impType::DoubleSupport));
    conf("tasks")("foot")("double-support").add("wrench", footWrench.at(impType::DoubleSupport));
    conf("tasks")("foot").add("single-support");
    conf("tasks")("foot")("single-support").add("mass", footMass.at(impType::SingleSupport));
    conf("tasks")("foot")("single-support").add("damper", footDamping.at(impType::SingleSupport));
    conf("tasks")("foot")("single-support").add("spring", footStiffness.at(impType::SingleSupport));
    conf("tasks")("foot")("single-support").add("wrench", footWrench.at(impType::SingleSupport));
    conf("tasks")("foot").add("swing");
    conf("tasks")("foot")("swing").add("mass", footMass.at(impType::Swing));
    conf("tasks")("foot")("swing").add("damper", footDamping.at(impType::Swing));
    conf("tasks")("foot")("swing").add("spring", footStiffness.at(impType::Swing));
    conf("tasks")("foot")("swing").add("wrench", footWrench.at(impType::Swing));
    conf("tasks")("foot").add("air");
    conf("tasks")("foot")("air").add("mass", footMass.at(impType::Air));
    conf("tasks")("foot")("air").add("damper", footDamping.at(impType::Air));
    conf("tasks")("foot")("air").add("spring", footStiffness.at(impType::Air));
    conf("tasks")("foot")("air").add("wrench", footWrench.at(impType::Air));
    conf("tasks")("foot").add("stiffness", footMotionStiffness);
    conf("tasks")("foot").add("weight", footWeight);
    
    conf.add("cutoffRatio", cutoffRatio);
    
    return conf;
  }
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration stabi;
    stabi.load(config);
    return stabi;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::DCMStabilizerConfiguration & stabiConf)
  {
    return stabiConf.save();
  }
};
} // namespace mc_rtc
