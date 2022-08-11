#pragma once
#ifndef TYPES_H
#define TYPES_H

/* includes //{ */

//}


namespace mrs_uav_state_estimation
{

/*//{ typedef */

typedef enum
{

  AXIS_X,
  AXIS_Y

} AxisId_t;

typedef enum
{

  POSITION,
  VELOCITY,
  ACCELERATION

} StateId_t;

typedef enum
{

  UNINITIALIZED_STATE,
  INITIALIZED_STATE,
  READY_STATE,
  STARTED_STATE,
  RUNNING_STATE,
  STOPPED_STATE,
  ERROR_STATE

} SMStates_t;
/*//}*/

namespace sm
{
// clang-format off
  const std::vector<std::string> state_names = {
    "UNINITIALIZED_STATE",
    "INITIALIZED_STATE",
    "READY_STATE",
    "STARTED_STATE",
    "RUNNING_STATE",
    "STOPPED_STATE",
    "ERROR_STATE"
  };
// clang-format on

}  // namespace sm

}  // namespace mrs_uav_state_estimation
#endif
