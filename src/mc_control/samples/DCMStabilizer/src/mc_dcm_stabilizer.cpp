/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_dcm_stabilizer.h"

DCMStabilizerController::DCMStabilizerController(mc_rbdyn::RobotModulePtr rm,
                                                 double dt,
                                                 const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
}
