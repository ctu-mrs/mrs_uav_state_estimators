#ifndef COMMON_HANDLERS_H
#define COMMON_HANDLERS_H

#include <ros/ros.h>
#include <mrs_lib/transformer.h>

namespace mrs_uav_state_estimation
{

struct CommonFrames_t
{

  std::string fcu;
  std::string ns_fcu;
  std::string fcu_untilted;
  std::string ns_fcu_untilted;
  std::string rtk_antenna;
  std::string ns_rtk_antenna;
};

struct CommonHandlers_t
{

  std::string                           package_name;
  std::string                           uav_name;
  CommonFrames_t                        frames;
  std::shared_ptr<mrs_lib::Transformer> transformer;
};


}  // namespace mrs_uav_state_estimation

#endif  // COMMON_HANDLERS_H
