#define VERSION "0.0.0.1"
#include "transform_manager.h"

namespace mrs_uav_state_estimation
{
/*//{ onInit() */
void TransformManager::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getName().c_str());

  broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

  mrs_lib::ParamLoader param_loader(nh_, getName());

  /*//{ check version */
  param_loader.loadParam("version", version_);

  if (version_ != VERSION) {

    ROS_ERROR("[%s]: the version of the binary (%s) does not match the config file (%s), please build me!", getName().c_str(), VERSION, version_.c_str());
    ros::shutdown();
  }
  /*//}*/

  param_loader.loadParam("uav_name", uav_name_);

/*//{ load fcu_untilted parameters */
  std::string fcu_frame_id;
  param_loader.loadParam("fcu_untilted_tf/parent", fcu_frame_id);
  ns_fcu_frame_id_ = uav_name_ + "/" + fcu_frame_id;

  std::string fcu_untilted_frame_id;
  param_loader.loadParam("fcu_untilted_tf/child", fcu_untilted_frame_id);
  ns_fcu_untilted_frame_id_ = uav_name_ + "/" + fcu_untilted_frame_id;

  param_loader.loadParam("fcu_untilted_tf/enabled", publish_fcu_untilted_tf_);
/*//}*/

/*//{ initialize tf sources */
  param_loader.loadParam("tf_sources", tf_source_names_);
  for (int i = 0; i < int(tf_source_names_.size()); i++) {
    const std::string tf_source_name = tf_source_names_[i];
    ROS_INFO("[%s]: loading tf source: %s", getName().c_str(), tf_source_name.c_str());
    tf_sources_.push_back(TfSource(tf_source_name, nh_, broadcaster_));
  }
/*//}*/

/*//{ initialize subscribers */
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in", &TransformManager::callbackMavrosOdom, this);
/*//}*/

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
    ros::shutdown();
  }

  ROS_INFO("[%s]: initialized", getName().c_str());
}
/*//}*/

/*//{ callbackMavrosOdom() */
void TransformManager::callbackMavrosOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

  nav_msgs::OdometryConstPtr msg = wrp.getMsg();

  if (publish_fcu_untilted_tf_) {
    publishFcuUntiltedTf(msg);
  }
}
/*//}*/

/*//{ publishFcuUntiltedTf() */
void TransformManager::publishFcuUntiltedTf(const nav_msgs::OdometryConstPtr& msg) {

  double heading;

  try {
    heading = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[%s]: Exception caught during getting heading", getName().c_str());
    return;
  }

  const Eigen::Matrix3d odom_pixhawk_R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);
  const Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

  const tf2::Quaternion q     = mrs_lib::AttitudeConverter(undo_heading_R * odom_pixhawk_R);
  const tf2::Quaternion q_inv = q.inverse();

  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = msg->header.stamp;  // TODO ros::Time::now()?
  tf.header.frame_id         = ns_fcu_frame_id_;
  tf.child_frame_id          = ns_fcu_untilted_frame_id_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(q_inv);

  if (noNans(tf)) {
    broadcaster_->sendTransform(tf);
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaN encountered in fcu_untilted tf", getName().c_str());
  }
}
/*//}*/


/*//{ getName() */
std::string TransformManager::getName() const {
  return name_;
}
/*//}*/


}  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::TransformManager, nodelet::Nodelet)
