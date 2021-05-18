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

/** Parameters for the DCM bias estimator */
struct DCMBiasEstimatorConfiguration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// the standard deviation of the dcm estimation error, NOT including the bias [m]
  double dcmMeasureErrorStd = 0.01;
  /// the standard deviaiton of the zmp estimation error [m]
  double zmpMeasureErrorStd = 0.05;
  /// the standard deviation of the drift [m/s]
  double biasDriftPerSecondStd = 0.02;
  /// Maximum bias in the sagital and lateral directions [m]
  Eigen::Vector2d biasLimit = {0.02, 0.02};
  /// Whether the DCM bias estimator is enabled (default: false for backwards compatibility)
  bool withDCMBias = false;
  /// Whether the DCM filter is enabled
  bool withDCMFilter = false;

  void load(const mc_rtc::Configuration & config)
  {
    config("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config("biasLimit", biasLimit);
    config("withDCMBias", withDCMBias);
    config("withDCMFilter", withDCMFilter);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config.add("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config.add("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config.add("biasLimit", biasLimit);
    config.add("withDCMBias", withDCMBias);
    config.add("withDCMFilter", withDCMFilter);
    return config;
  }
};

} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
/**
 * @brief Read DCMBias estimation parameters
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration bias;
    bias.load(config);
    return bias;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration & bias)
  {
    return bias.save();
  }
};

} // namespace mc_rtc
