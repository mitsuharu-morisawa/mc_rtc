/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/api.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/** @brief Parameters for the external wrenches
 *
 *  The external forces affect the two layers, i.e., pattern generation and stabilization control.
 *
 *  For the pattern generation, if the external forces are not taken into account in the input target CoM, an offset is
 * added by feedforward from the target external forces. This feature is enabled by setting addExpectedCoMOffset to
 * true. For the stabilization control, the error between the target external forces and the measured external forces is
 * compensated by the CoM strategy and the ZMP strategy. High-frequency errors are handled by the ZMP strategy, while
 * low-frequency errors are handled by the CoM strategy. The CoM strategy is enabled when modifyCoMErr is set to true.
 * The ZMP strategy is enabled when modifyZMPErr is set to true. Strictly speaking, the ZMP strategy also depends on the
 * derivative of the external force error, and when modifyZMPErrD is true, the corresponding compensation is also
 * enabled. However, the effect of the compensation related to the derivative is not dominant and is affected by the
 * measured external force noise, so it is recommended to try with modifyZMPErrD false first.
 * CommOffsetDerivatorTimeConstant is a time constant for calculating this derivative. The maximum amounts of
 * modification for CoM strategy and ZMP strategy are comOffsetErrCoMLimit and comOffsetErrZMPLimit, respectively.
 *
 *  The total measured external force is passed through the low-pass filter of the cutoff period
 * exitWenchSumLowPassCutoffPeriod. Then, from the error between the target external forces and the measured external
 * forces, the amounts of compensation for the CoM strategy and ZMP strategy are calculated using two low-pass filters.
 * One is a low-pass filter of the cutoff period comOffsetLowPassCutoffPeriod (High-LPF), and the other is a low-pass
 * filter of the cutoff period comOffsetLowPassCoMCutoffPeriod (Low-LPF). Make the comOffsetLowPassCoMCutoffPeriod
 * larger than the comOffsetLowPassCutoffPeriod. The CoM strategy deals with the error components that pass through the
 * High-LPF and pass through the Low-LPF. The ZMP strategy deals with the error components that pass the High-LPF but do
 * NOT pass the Low-LPF.
 *
 *  - If you want the StabilizerTask to deal with external forces in a simple way (i.e., the pattern generator does not
 *    take external forces into account), then set addExpectedCoMOffset, modifyCoMErr, modifyZMPErr to true.
 *  - modifyZMPErrD should theoretically be true, but it depends on the derivative of the measured external forces, so
 *    it becomes sensitive to the noises in the measurement noise.
 *  - subtractMeasuredValue is a more experimental and the option inspired from
 *    https://github.com/stephane-caron/lipm_walking_controller/discussions/28 Normally set to false.
 */
struct ExternalWrenchConfiguration
{
  /// Whether to add the CoM offset expected from the external wrenches.
  /// Should be false if the target CoM generated by the pattern generator already considers the external wrenches, true
  /// otherwise.
  bool addExpectedCoMOffset = false;
  /// Subtract the measured external wrenches instead of target ones
  bool subtractMeasuredValue = false;
  /// Modify CoM depending on the error of the external wrenches in target and measurement
  bool modifyCoMErr = false;
  /// Modify ZMP depending on the error of the external wrenches in target and measurement
  bool modifyZMPErr = false;
  /// Modify ZMP velocity depending on the error velocity of the external wrenches in target and measurement
  bool modifyZMPErrD = false;
  /// Limit of CoM offset error handled by CoM modification
  double comOffsetErrCoMLimit = 0.1;
  /// Limit of CoM offset error handled by ZMP modification [m]
  double comOffsetErrZMPLimit = 0.1;
  /// Cutoff period for the low-pass filter of the sum of the measured external wrenches
  double extWrenchSumLowPassCutoffPeriod = 0.05;
  /// Cutoff period for the low-pass filter of CoM offset
  double comOffsetLowPassCutoffPeriod = 0.05;
  /// Cutoff period for the low-pass filter of CoM offset to extract CoM modification
  double comOffsetLowPassCoMCutoffPeriod = 1.0;
  /// Time window for the stationary offset filter of the CoM offset derivator
  double comOffsetDerivatorTimeConstant = 1.0;

  void load(const mc_rtc::Configuration & config)
  {
    config("add_expected_com_offset", addExpectedCoMOffset);
    config("subtract_measured_value", subtractMeasuredValue);
    config("modify_com_error", modifyCoMErr);
    config("modify_zmp_error", modifyZMPErr);
    config("modify_zmp_error_d", modifyZMPErrD);
    config("com_offset_err_com_limit", comOffsetErrCoMLimit);
    config("com_offset_err_zmp_limit", comOffsetErrZMPLimit);
    config("ext_wrench_sum_cutoff", extWrenchSumLowPassCutoffPeriod);
    config("com_offset_cutoff", comOffsetLowPassCutoffPeriod);
    config("com_offset_com_cutoff", comOffsetLowPassCoMCutoffPeriod);
    config("derivator_time_constant", comOffsetDerivatorTimeConstant);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("add_expected_com_offset", addExpectedCoMOffset);
    config.add("subtract_measured_value", subtractMeasuredValue);
    config.add("modify_com_error", modifyCoMErr);
    config.add("modify_zmp_error", modifyZMPErr);
    config.add("modify_zmp_error_d", modifyZMPErrD);
    config.add("com_offset_err_com_limit", comOffsetErrCoMLimit);
    config.add("com_offset_err_zmp_limit", comOffsetErrZMPLimit);
    config.add("ext_wrench_sum_cutoff", extWrenchSumLowPassCutoffPeriod);
    config.add("com_offset_cutoff", comOffsetLowPassCutoffPeriod);
    config.add("com_offset_com_cutoff", comOffsetLowPassCoMCutoffPeriod);
    config.add("derivator_time_constant", comOffsetDerivatorTimeConstant);
    return config;
  }
};

} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
/**
 * @brief Read parameters for the external wrenches
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration extWrench;
    extWrench.load(config);
    return extWrench;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration & extWrench)
  {
    return extWrench.save();
  }
};
} // namespace mc_rtc
