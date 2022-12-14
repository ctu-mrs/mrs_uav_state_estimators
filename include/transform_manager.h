#pragma once
#ifndef TRANSFORM_MANAGER_H
#define TRANSFORM_MANAGER_H

/* //{ includes */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include "support.h"

/*//}*/

namespace mrs_uav_state_estimation
{

/*//{ class TfSource */
class TfSource {

public:
  /*//{ constructor */
  TfSource(const std::string& name, ros::NodeHandle nh, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster)
      : name_(name), nh_(nh), broadcaster_(broadcaster) {

    ROS_INFO("[%s]: initializing", getName().c_str());

    mrs_lib::ParamLoader param_loader(nh_, getName());
    std::string          uav_name, topic, ns;
    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam(getName() + "/topic", topic);
    param_loader.loadParam(getName() + "/namespace", ns);
    const std::string full_topic = "/" + uav_name + "/" + ns + "/" + topic;
    param_loader.loadParam(getName() + "/inverted", inverted_);
    param_loader.loadParam(getName() + "/republish_in_frames", republish_in_frames_);

    // | --------------- subscribers initialization --------------- |
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = getName();
    shopts.no_message_timeout = ros::Duration(0.5);
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tf_source_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, full_topic, &TfSource::callbackTfSource, this);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
      ros::shutdown();
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

private:
  const std::string name_;

  ros::NodeHandle nh_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;
  bool                                           inverted_;

  std::atomic_bool is_initialized_ = false;

  std::vector<std::string> republish_in_frames_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_tf_source_;
  /*//{ callbackTfSource()*/
  void callbackTfSource(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

    if (!is_initialized_) {
      return;
    }

    nav_msgs::OdometryConstPtr msg = wrp.getMsg();
    publishTfFromOdom(msg);
  }
  /*//}*/

  /* publishTfFromOdom() //{*/
  void publishTfFromOdom(const nav_msgs::OdometryConstPtr& odom) {


    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom->header.stamp;

    if (inverted_) {

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
    ROS_INFO_ONCE("[%s]: Broadcasting transform from parent frame: %s to child frame: %s based on odometry: %s", getName().c_str(),
                  tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str(), getName().c_str());
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

  bool publish_fcu_untilted_tf_;

  std::vector<std::string>               tf_source_names_, estimator_names_;
  std::vector<std::unique_ptr<TfSource>> tf_sources_;

  ros::NodeHandle nh_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  void                                          callbackMavrosOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);

  mrs_lib::Transformer                           transformer_;
  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  void publishFcuUntiltedTf(const nav_msgs::OdometryConstPtr& msg);
};
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
