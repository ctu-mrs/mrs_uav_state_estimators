#pragma once
#ifndef TRANSFORM_MANAGER_H
#define TRANSFORM_MANAGER_H

/* //{ includes */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/gps_conversions.h>

#include <mrs_msgs/UavState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

#include "support.h"
#include "common_handlers.h"
#include "tf_source.h"

/*//}*/

namespace mrs_uav_state_estimation
{


/*//{ class TransformManager */
class TransformManager : public nodelet::Nodelet {

public:
  TransformManager() {
  ch_ = std::make_shared<CommonHandlers_t>();

  ch_->package_name = package_name_;
  ch_->nodelet_name = nodelet_name_;
  }

  void onInit();

  std::string getName() const;

  std::string getPrintName() const;

private:

  const std::string package_name_ = "mrs_uav_state_estimation";
  const std::string nodelet_name_ = "TransformManager";
  const std::string name_ = "transform_manager";

  std::string version_;

  bool        publish_fcu_untilted_tf_;

  std::string ns_local_origin_parent_frame_id_;
  std::string ns_local_origin_child_frame_id_;
  bool        publish_local_origin_tf_;

  std::string ns_stable_origin_parent_frame_id_;
  std::string ns_stable_origin_child_frame_id_;
  bool        publish_stable_origin_tf_;

  std::string         ns_fixed_origin_parent_frame_id_;
  std::string         ns_fixed_origin_child_frame_id_;
  bool                publish_fixed_origin_tf_;
  geometry_msgs::Pose pose_fixed_;
  geometry_msgs::Pose pose_fixed_diff_;

  int                  world_origin_units_;
  geometry_msgs::Point world_origin_;

  std::vector<std::string>               tf_source_names_, estimator_names_;
  std::vector<std::unique_ptr<TfSource>> tf_sources_;

  ros::NodeHandle nh_;

  std::shared_ptr<CommonHandlers_t> ch_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;
  void                                          callbackUavState(mrs_lib::SubscribeHandler<mrs_msgs::UavState>& wrp);
  std::string                                   first_frame_id_;
  std::string                                   last_frame_id_;
  bool                                          is_first_frame_id_set_ = false;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  void                                          callbackMavrosOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_mavros_utm_;
  void                                              callbackMavrosUtm(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp);
  std::atomic<bool>                                 got_mavros_utm_offset_ = false;

  void publishFcuUntiltedTf(const nav_msgs::OdometryConstPtr& msg);
};
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif  // TRANSFORM_MANAGER_H
