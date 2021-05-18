/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

namespace mc_tasks
{
namespace lipm_stabilizer
{
/** Set of impedance gain according to ContactState.
 *
 */
enum class MC_TASKS_DLLAPI ImpedanceType
{
  None,
  Stand,
  DoubleSupport,
  SingleSupport,
  Swing,    
  Air
};

} // namespace lipm_stabilizer
} // namespace mc_tasks
