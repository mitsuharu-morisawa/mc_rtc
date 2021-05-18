set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_dcm_stabilizer.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_dcm_stabilizer.h
)

add_fsm_controller(dcm_stabilizer ${controller_SRC} ${controller_HDR})
