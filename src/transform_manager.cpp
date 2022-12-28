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

    bool is_origin_param_ok = true;
    double world_origin_x, world_origin_y;
    param_loader.loadParam("utm_origin_units", world_origin_units_);
    if (world_origin_units_ == 0) {
      ROS_INFO("[Odometry]: Loading world origin in UTM units.");
      is_origin_param_ok &= param_loader.loadParam("utm_origin_x", world_origin_x);
      is_origin_param_ok &= param_loader.loadParam("utm_origin_y", world_origin_y);
    } else {
      double lat, lon;
      ROS_INFO("[Odometry]: Loading world origin in LatLon units.");
      is_origin_param_ok &= param_loader.loadParam("utm_origin_lat", lat);
      is_origin_param_ok &= param_loader.loadParam("utm_origin_lon", lon);
      mrs_lib::UTM(lat, lon, &world_origin_x, &world_origin_y);
      ROS_INFO("[Odometry]: Converted to UTM x: %f, y: %f.", world_origin_x, world_origin_y);
    }

    world_origin_.x = world_origin_x;
    world_origin_.y = world_origin_y;
    world_origin_.z = 0;

    /*     is_origin_param_ok &= param_loader.loadParam("init_gps_origin_local", init_gps_origin_local_); */
    /*     is_origin_param_ok &= param_loader.loadParam("init_gps_offset_x", init_gps_offset_x_); */
    /*     is_origin_param_ok &= param_loader.loadParam("init_gps_offset_y", init_gps_offset_y_); */

    if (!is_origin_param_ok) {
      ROS_ERROR("[Odometry]: Could not load all mandatory parameters from world file. Please check your world file.");
      ros::shutdown();
    }

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
    tf_sources_.push_back(std::make_unique<TfSource>(tf_source_name, nh_, broadcaster_));
  }

  // additionally publish tf of all available estimators
  param_loader.loadParam("/" + uav_name_ + "/estimation_manager/state_estimators", estimator_names_);
  for (int i = 0; i < int(estimator_names_.size()); i++) {
    const std::string estimator_name = estimator_names_[i];
    ROS_INFO("[%s]: loading tf source of estimator: %s", getName().c_str(), estimator_name.c_str());
    tf_sources_.push_back(std::make_unique<TfSource>(estimator_name, nh_, broadcaster_));
  }
  /*//}*/

  /*//{ initialize subscribers */
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in", &TransformManager::callbackMavrosOdom, this);

  sh_mavros_utm_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "mavros_utm_in", &TransformManager::callbackMavrosUtm, this);
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

/*//{ callbackMavrosUtm() */
void TransformManager::callbackMavrosUtm(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp) {

  if (!got_mavros_utm_offset_) {

    sensor_msgs::NavSatFixConstPtr msg = wrp.getMsg();

    double out_x;
    double out_y;

    mrs_lib::UTM(msg->latitude, msg->longitude, &out_x, &out_y);

    if (!std::isfinite(out_x)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_x\"!!!");
      return;
    }

    if (!std::isfinite(out_y)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_y\"!!!");
      return;
    }

    geometry_msgs::Point utm_origin;
    utm_origin.x = out_x;
    utm_origin.y = out_y;
    utm_origin.z = msg->altitude;

    ROS_INFO("[%s]: utm_origin position calculated as: x: %.2f, y: %.2f, z: %.2f", getName().c_str(), utm_origin.x, utm_origin.y, utm_origin.z);

    for (size_t i=0; i<tf_sources_.size(); i++) {
      tf_sources_[i]->setUtmOrigin(utm_origin); 
      tf_sources_[i]->setWorldOrigin(world_origin_); 
    }
    got_mavros_utm_offset_ = true;
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

  if (Support::noNans(tf)) {
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
