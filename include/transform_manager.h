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

#include "support.h"

/*//}*/

namespace mrs_uav_state_estimation
{

/*//{ class TfSource */
class TfSource {

public:
  /*//{ constructor */
  TfSource(const std::string& name, ros::NodeHandle nh, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster,
           const std::shared_ptr<mrs_lib::Transformer>& transformer)
      : name_(name), nh_(nh), broadcaster_(broadcaster), transformer_(transformer) {

    ROS_INFO("[%s]: initializing", getName().c_str());

    /*//{ load parameters */
    mrs_lib::ParamLoader param_loader(nh_, getName());
    std::string          topic, ns;
    param_loader.loadParam("uav_name", uav_name_);
    param_loader.loadParam(getName() + "/topic", topic);
    param_loader.loadParam(getName() + "/namespace", ns);
    full_topic_ = "/" + uav_name_ + "/" + ns + "/" + topic;
    param_loader.loadParam(getName() + "/inverted", is_inverted_);
    param_loader.loadParam(getName() + "/republish_in_frames", republish_in_frames_);

    /* coordinate frames origins //{ */
    param_loader.loadParam(getName() + "/utm_based", is_utm_based_);
    if (is_utm_based_) {
      param_loader.loadParam(getName() + "/in_utm", is_in_utm_);
    }

    // set initial UTM coordinates to zero for tf sources already in UTF frame
    if (is_in_utm_) {
      geometry_msgs::Point origin_pt;
      origin_pt.x = 0;
      origin_pt.y = 0;
      origin_pt.z = 0;
      setUtmOrigin(origin_pt);
    }

    //}

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
      ros::shutdown();
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

    sh_tf_source_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, full_topic_, &TfSource::callbackTfSource, this);
    /*//}*/

    transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, getName());
    transformer_->retryLookupNewest(true);

    for (auto frame_id : republish_in_frames_) {
      republishers_.push_back(std::make_pair(frame_id, mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, full_topic_ + "/" + frame_id.substr(0, frame_id.find("_origin")))));
    }
    is_initialized_ = true;
    ROS_INFO("[%s]: initialized", getName().c_str());
  }
  /*//}*/

  /*//{ getName() */
  std::string getName() {
    return name_;
  }
  /*//}*/

  /*//{ setUtmOrigin() */
  void setUtmOrigin(const geometry_msgs::Point& pt) {

    if (is_utm_based_ && !is_utm_origin_set_) {
      utm_origin_        = pt;
      is_utm_origin_set_ = true;
    }
  }
  /*//}*/

  /*//{ setWorldOrigin() */
  void setWorldOrigin(const geometry_msgs::Point& pt) {

    if (is_utm_based_ && !is_world_origin_set_) {
      world_origin_        = pt;
      is_world_origin_set_ = true;
    }
  }
  /*//}*/

private:
  const std::string name_;

  ros::NodeHandle nh_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;
  tf2_ros::StaticTransformBroadcaster            static_broadcaster_;
  std::shared_ptr<mrs_lib::Transformer>          transformer_;

  bool is_inverted_;

  bool is_utm_based_;
  bool is_in_utm_ = false;

  bool                 is_utm_origin_set_ = false;
  geometry_msgs::Point utm_origin_;

  bool                 is_world_origin_set_ = false;
  geometry_msgs::Point world_origin_;

  std::string uav_name_;

  std::string full_topic_;

  std::atomic_bool is_initialized_               = false;
  std::atomic_bool is_local_static_tf_published_ = false;
  std::atomic_bool is_utm_static_tf_published_   = false;
  std::atomic_bool is_world_static_tf_published_ = false;

  std::vector<std::string> republish_in_frames_;

  std::vector<std::pair<std::string, mrs_lib::PublisherHandler<nav_msgs::Odometry>>> republishers_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_tf_source_;
  nav_msgs::OdometryConstPtr                    first_msg_;

  /*//{ callbackTfSource()*/
  void callbackTfSource(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

    if (!is_initialized_) {
      return;
    }

    nav_msgs::OdometryConstPtr msg = wrp.getMsg();
    first_msg_                     = msg;
    publishTfFromOdom(msg);

    if (!is_local_static_tf_published_) {
      publishLocalTf(msg->header.frame_id);
    }

    if (is_utm_based_ && is_utm_origin_set_ && !is_utm_static_tf_published_) {
      publishUtmTf(msg->header.frame_id);
    }

    if (is_utm_based_ && is_world_origin_set_ && !is_world_static_tf_published_) {
      publishWorldTf(msg->header.frame_id);
    }

    for (auto republisher : republishers_) {
      republishInFrame(msg, uav_name_ + "/" + republisher.first, republisher.second);
    }
  }
  /*//}*/

  /* publishTfFromOdom() //{*/
  void publishTfFromOdom(const nav_msgs::OdometryConstPtr& odom) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom->header.stamp;

    if (is_inverted_) {

      const tf2::Transform      tf       = Support::tf2FromPose(odom->pose.pose);
      const tf2::Transform      tf_inv   = tf.inverse();
      const geometry_msgs::Pose pose_inv = Support::poseFromTf2(tf_inv);

      tf_msg.header.frame_id       = odom->child_frame_id;
      tf_msg.child_frame_id        = odom->header.frame_id;
      tf_msg.transform.translation = Support::pointToVector3(pose_inv.position);
      tf_msg.transform.rotation    = pose_inv.orientation;

    } else {
      tf_msg.header.frame_id       = odom->header.frame_id;
      tf_msg.child_frame_id        = odom->child_frame_id;
      tf_msg.transform.translation = Support::pointToVector3(odom->pose.pose.position);
      tf_msg.transform.rotation    = odom->pose.pose.orientation;
    }

    if (Support::noNans(tf_msg)) {
      try {
        broadcaster_->sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on topic: %s", getName().c_str(), tf_msg.header.frame_id.c_str(),
                  tf_msg.child_frame_id.c_str(), full_topic_.c_str());
  }
  /*//}*/

  /* publishLocalTf() //{*/
  void publishLocalTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id       = frame_id;
    tf_msg.child_frame_id        = frame_id.substr(0, frame_id.find("_origin")) + "_local_origin";
    tf_msg.transform.translation = Support::pointToVector3(first_msg_->pose.pose.position);
    tf_msg.transform.rotation    = first_msg_->pose.pose.orientation;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_.c_str());
    is_local_static_tf_published_ = true;
  }
  /*//}*/

  /* publishUtmTf() //{*/
  void publishUtmTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id         = frame_id;
    tf_msg.child_frame_id          = frame_id.substr(0, frame_id.find("_origin")) + "_utm_origin";
    tf_msg.transform.translation.x = -utm_origin_.x;  // minus because inverse tf tree
    tf_msg.transform.translation.y = -utm_origin_.y;  // minus because inverse tf tree
    tf_msg.transform.translation.z = -utm_origin_.z;  // minus because inverse tf tree
    tf_msg.transform.rotation.x    = 0;
    tf_msg.transform.rotation.y    = 0;
    tf_msg.transform.rotation.z    = 0;
    tf_msg.transform.rotation.w    = 1;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_.c_str());
    is_utm_static_tf_published_ = true;
  }
  /*//}*/

  /* publishWorldTf() //{*/
  void publishWorldTf(const std::string& frame_id) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id         = frame_id;
    tf_msg.child_frame_id          = frame_id.substr(0, frame_id.find("_origin")) + "_world_origin";
    tf_msg.transform.translation.x = -(utm_origin_.x - world_origin_.x);  // minus because inverse tf tree
    tf_msg.transform.translation.y = -(utm_origin_.y - world_origin_.y);  // minus because inverse tf tree
    tf_msg.transform.translation.z = -(utm_origin_.z);                    // minus because inverse tf tree
    tf_msg.transform.rotation.x    = 0;
    tf_msg.transform.rotation.y    = 0;
    tf_msg.transform.rotation.z    = 0;
    tf_msg.transform.rotation.w    = 1;

    if (Support::noNans(tf_msg)) {
      try {
        static_broadcaster_.sendTransform(tf_msg);
      }
      catch (...) {
        ROS_ERROR("exception caught ");
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: NaN detected in transform from %s to %s. Not publishing tf.", getName().c_str(), tf_msg.header.frame_id.c_str(),
                        tf_msg.child_frame_id.c_str());
    }
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on first message of: %s", getName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), full_topic_.c_str());
    is_world_static_tf_published_ = true;
  }
  /*//}*/

  /* republishInFrame() //{*/
  void republishInFrame(const nav_msgs::OdometryConstPtr& msg, const std::string& frame_id, mrs_lib::PublisherHandler<nav_msgs::Odometry>& ph) {

    nav_msgs::Odometry msg_out = *msg;
    msg_out.header.frame_id    = frame_id;

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;

    auto res = transformer_->transformSingle(pose, frame_id);
    if (res) {
      msg_out.pose.pose = res->pose;
      ph.publish(msg_out);
    } else {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not republishing odom in this frame.", getName().c_str(), frame_id.c_str());
      return;
    }
  }

  /*//}*/
};

/*//}*/

/*//{ class TransformManager */
class TransformManager : public nodelet::Nodelet {

public:
  TransformManager(){};

  void onInit();

  std::string getName() const;

private:
  const std::string name_ = "TransformManager";

  std::string version_;
  std::string uav_name_;

  std::string ns_fcu_frame_id_;
  std::string ns_fcu_untilted_frame_id_;
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

  std::shared_ptr<mrs_lib::Transformer>          transformer_;
  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  void publishFcuUntiltedTf(const nav_msgs::OdometryConstPtr& msg);
};
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
