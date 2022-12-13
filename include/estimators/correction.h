#pragma once
#ifndef CORRECTION_H
#define CORRECTION_H

#include <Eigen/Dense>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/gps_conversions.h>

#include <mrs_msgs/RtkGps.h>

#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include "types.h"
#include "support.h"
#include "common_handlers.h"

#include "mrs_uav_state_estimation/EstimatorCorrection.h"
#include "mrs_uav_state_estimation/CorrectionConfig.h"

namespace mrs_uav_state_estimation
{

// TODO is needed?
typedef enum
{
  LATERAL,
  ALTITUDE,
  HEADING
} EstimatorType_t;
const int n_EstimatorType_t = 3;

typedef enum
{
  ODOMETRY,
  POSE_S,
  POSE_WCS,
  RANGE,
  RTK_GPS,
} MessageType_t;
const int n_MessageType_t = 5;

template <int n_measurements>
class Correction {

public:
  typedef Eigen::Matrix<double, n_measurements, 1>         measurement_t;
  typedef mrs_lib::DynamicReconfigureMgr<CorrectionConfig> drmgr_t;

public:
  Correction(ros::NodeHandle& nh, const std::string& est_name, const std::string& name, const std::string& frame_id, const EstimatorType_t& est_type,
             const std::shared_ptr<CommonHandlers_t>& ch);

  std::string getName() const;
  std::string getNamespacedName() const;

  double    getR();
  StateId_t getStateId() const;

  bool             isHealthy();
  std::atomic_bool is_healthy_  = true;
  std::atomic_bool is_delay_ok_ = true;
  std::atomic_bool is_dt_ok_    = true;
  std::atomic_bool is_nan_free_ = true;

  bool getCorrection(measurement_t& measurement);

private:
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                       sh_odom_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>               sh_pose_s_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_pose_wcs_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range>                       sh_range_;
  mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>                         sh_rtk_;

  void timeoutCallback(const std::string& topic, const ros::Time& last_msg, const int n_pubs);

  mrs_lib::PublisherHandler<EstimatorCorrection> ph_correction_;

  const std::string                 est_name_;
  const std::string                 name_;
  const std::string                 ns_frame_id_;
  const EstimatorType_t             est_type_;
  std::shared_ptr<CommonHandlers_t> ch_;

  MessageType_t msg_type_;
  std::string   msg_topic_;
  double        msg_timeout_;

  double     R_;
  std::mutex mtx_R_;
  StateId_t  state_id_;

  std::unique_ptr<drmgr_t> drmgr_;

  bool                  getCorrectionFromOdometry(const nav_msgs::Odometry& msg, measurement_t& measurement);
  bool                  getCorrectionFromRange(const sensor_msgs::Range& msg, measurement_t& measurement);
  bool                  getCorrectionFromRtk(const mrs_msgs::RtkGps& msg, measurement_t& measurement);
  std::optional<double> getZVelUntilted(const nav_msgs::Odometry& msg);
  bool                  getVelInFrame(const nav_msgs::Odometry& msg, const std::string frame, measurement_t& measurement_out);

  geometry_msgs::Pose transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const;

  void   checkMsgDelay(const ros::Time& msg_time);
  double msg_delay_limit_;

  double time_since_last_msg_limit_;

  void publishCorrection(const measurement_t& measurement, const ros::Time& measurement_stamp);
};

/*//{ constructor */
template <int n_measurements>
Correction<n_measurements>::Correction(ros::NodeHandle& nh, const std::string& est_name, const std::string& name, const std::string& ns_frame_id,
                                       const EstimatorType_t& est_type, const std::shared_ptr<CommonHandlers_t>& ch)
    : est_name_(est_name), name_(name), ns_frame_id_(ns_frame_id), est_type_(est_type), ch_(ch) {

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, getName());
  param_loader.setPrefix(getNamespacedName() + "/");

  int msg_type_tmp;
  param_loader.loadParam("message/type", msg_type_tmp);
  if (msg_type_tmp < n_MessageType_t) {
    msg_type_ = static_cast<MessageType_t>(msg_type_tmp);
  } else {
    ROS_ERROR("[%s]: wrong message type: %d of correction %s", getNamespacedName().c_str(), msg_type_tmp, getName().c_str());
    ros::shutdown();
  }
  param_loader.loadParam("message/topic", msg_topic_);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;
  param_loader.loadParam("message/limit/delay", msg_delay_limit_);
  param_loader.loadParam("message/limit/time_since_last", time_since_last_msg_limit_);

  int state_id_tmp;
  param_loader.loadParam("state_id", state_id_tmp);
  if (state_id_tmp < n_StateId_t) {
    state_id_ = static_cast<StateId_t>(state_id_tmp);
  } else {
    ROS_ERROR("[%s]: wrong state id: %d of correction %s", getNamespacedName().c_str(), state_id_tmp, getName().c_str());
    ros::shutdown();
  }
  param_loader.loadParam("noise", R_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getNamespacedName().c_str());
    ros::shutdown();
  }

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_               = std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), getNamespacedName());
  drmgr_->config.noise = R_;

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getNamespacedName();
  shopts.no_message_timeout = ros::Duration(time_since_last_msg_limit_);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  switch (msg_type_) {
    case MessageType_t::ODOMETRY: {
      sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::POSE_S: {
      // TODO implement
      /* sh_pose_s_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::POSE_WCS: {
      // TODO implement
      /* sh_pose_wcs_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::RANGE: {
      sh_range_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::RTK_GPS: {
      sh_rtk_ = mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
  }

  // | --------------- initialize publish handlers -------------- |
  ph_correction_ = mrs_lib::PublisherHandler<EstimatorCorrection>(nh, est_name_ + "/correction/" + getName(), 1);
}
/*//}*/

template <int n_measurements>
std::string Correction<n_measurements>::getName() const {
  return name_;
}

template <int n_measurements>
std::string Correction<n_measurements>::getNamespacedName() const {
  return est_name_ + "/" + name_;
}

template <int n_measurements>
double Correction<n_measurements>::getR() {
  std::scoped_lock lock(mtx_R_);
  R_ = drmgr_->config.noise;
  return R_;
}

template <int n_measurements>
StateId_t Correction<n_measurements>::getStateId() const {
  return state_id_;
}

/*//{ isHealthy() */
template <int n_measurements>
bool Correction<n_measurements>::isHealthy() {

  is_healthy_ = is_healthy_ && is_dt_ok_ && is_delay_ok_ && is_nan_free_;

  return is_healthy_;
}
/*//}*/

/*//{ getCorrection() */
template <int n_measurements>
bool Correction<n_measurements>::getCorrection(measurement_t& measurement) {

  ros::Time measurement_stamp;

  switch (msg_type_) {

    case MessageType_t::ODOMETRY: {

      if (!sh_odom_.newMsg()) {
        return false;
      }

      auto msg          = sh_odom_.getMsg();
      measurement_stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamp);

      if (!is_delay_ok_ || !getCorrectionFromOdometry(*msg, measurement)) {
        return false;
      }
      break;
    }

    case MessageType_t::POSE_S: {
      // TODO implement
      /* return getCorrectionFromPoseS(msg); */
      is_healthy_ = false;
      return false;
      break;
    }

    case MessageType_t::POSE_WCS: {
      // TODO implement
      /* return getCorrectionFromPoseWCS(msg); */
      is_healthy_ = false;
      return false;
      break;
    }

    case MessageType_t::RANGE: {

      if (!sh_range_.newMsg()) {
        return false;
      }

      auto msg          = sh_range_.getMsg();
      measurement_stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamp);

      // range correction must be transformed to the untilted frame
      if (!is_delay_ok_ || !getCorrectionFromRange(*msg, measurement)) {
        return false;
      }
      break;
    }

    case MessageType_t::RTK_GPS: {

      if (!sh_rtk_.newMsg()) {
        /* ROS_ERROR(" no new rtk msg"); */
        return false;
      }

      auto msg          = sh_rtk_.getMsg();
      measurement_stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamp);

      if (!is_delay_ok_ || !getCorrectionFromRtk(*msg, measurement)) {
        return false;
      }
      break;
    }

    default: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: this type of correction is not implemented in getCorrectionFromMessage()", getNamespacedName().c_str());
      is_healthy_ = false;
      return false;
    }

      // check for nans
      for (int i = 0; i < measurement.rows(); i++) {
        if (!std::isfinite(measurement(i))) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in correction", getNamespacedName().c_str());
          is_nan_free_ = false;
          return false;
        }
      }
  }
  ROS_INFO("[%s]: debug: rtk correction: %f %f", getNamespacedName().c_str(), measurement(0), measurement(1));

  publishCorrection(measurement, measurement_stamp);

  return true;
}
/*//}*/

/*//{ getCorrectionFromOdometry() */
template <int n_measurements>
bool Correction<n_measurements>::getCorrectionFromOdometry(const nav_msgs::Odometry& msg, measurement_t& measurement) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = msg.pose.pose.position.x;
          measurement(1) = msg.pose.pose.position.y;
          break;
        }

        case StateId_t::VELOCITY: {
          if (!getVelInFrame(msg, ns_frame_id_, measurement)) {
            return false;
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getNamespacedName().c_str());
          /* return {}; */
          return false;
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = msg.pose.pose.position.z;
          break;
        }

        case StateId_t::VELOCITY: {
          auto res = getZVelUntilted(msg);
          if (res) {
            measurement(0) = res.value();
          } else {
            return false;
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getNamespacedName().c_str());
          return false;
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          try {
            measurement(0) = mrs_lib::AttitudeConverter(msg.pose.pose.orientation).getHeading();
          }
          catch (...) {
            ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading (getCorrectionFromOdometry())", getNamespacedName().c_str());
            return false;
          }
          break;
        }

        case StateId_t::VELOCITY: {
          try {
            measurement(0) = mrs_lib::AttitudeConverter(msg.pose.pose.orientation).getHeadingRate(msg.twist.twist.angular);
          }
          catch (...) {
            ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading rate (getCorrectionFromOdometry())", getNamespacedName().c_str());
            return false;
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getNamespacedName().c_str());
          return false;
        }
      }
      break;
    }
  }

  return true;
}
/*//}*/

/*//{ getCorrectionFromRange() */
template <int n_measurements>
bool Correction<n_measurements>::getCorrectionFromRange(const sensor_msgs::Range& msg, measurement_t& measurement) {

  geometry_msgs::PoseStamped range_point;

  range_point.header           = msg.header;
  range_point.pose.position.x  = msg.range;
  range_point.pose.position.y  = 0;
  range_point.pose.position.z  = 0;
  range_point.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  auto res = ch_->transformer->transformSingle(range_point, ch_->frames.ns_fcu_untilted);

  /* Correction::measurement_t measurement; */

  if (res) {
    measurement(0) = -res.value().pose.position.z;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform range measurement to %s. Not using this correction.", getNamespacedName().c_str(),
                       ch_->frames.ns_fcu_untilted.c_str());
    /* return {}; */
    return false;
  }

  return true;
}
/*//}*/

/*//{ getCorrectionFromRtk() */
template <int n_measurements>
bool Correction<n_measurements>::getCorrectionFromRtk(const mrs_msgs::RtkGps& msg, measurement_t& measurement) {

  geometry_msgs::PoseStamped rtk_pos;

  if (msg.header.frame_id == "gps") {

    if (!std::isfinite(msg.gps.latitude)) {
      ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->latitude\"!!!", getNamespacedName().c_str());
      return false;
    }

    if (!std::isfinite(msg.gps.longitude)) {
      ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->longitude\"!!!", getNamespacedName().c_str());
      return false;
    }

    rtk_pos.header = msg.header;
    mrs_lib::UTM(msg.gps.latitude, msg.gps.longitude, &rtk_pos.pose.position.x, &rtk_pos.pose.position.y);
    rtk_pos.pose.position.z  = msg.gps.altitude;
    rtk_pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  } else if (msg.header.frame_id == "utm") {

    rtk_pos.pose = msg.pose.pose;

  } else {

    ROS_INFO_THROTTLE(1.0, "[%s]: RTK message has unknown frame_id: '%s'", getNamespacedName().c_str(), msg.header.frame_id.c_str());
  }

  // transform the RTK position from antenna to FCU
  rtk_pos.pose = transformRtkToFcu(rtk_pos);

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = rtk_pos.pose.position.x;
          measurement(1) = rtk_pos.pose.position.y;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromRtk() switch", getNamespacedName().c_str());
          /* return {}; */
          return false;
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = rtk_pos.pose.position.z;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromRtk() switch", getNamespacedName().c_str());
          return false;
        }
      }
      break;
    }

    case EstimatorType_t::HEADING: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: should not be possible to get into this branch of getCorrectionFromRtk() switch", getNamespacedName().c_str());
      break;
    }
  }

  return true;
}
/*//}*/

/*//{ timeoutCallback() */
template <int n_measurements>
void Correction<n_measurements>::timeoutCallback(const std::string& topic, const ros::Time& last_msg, const int n_pubs) {
  is_dt_ok_ = false;
  ROS_ERROR_STREAM("[" << getNamespacedName() << "]: not received message from topic '" << topic << "' for " << (ros::Time::now() - last_msg).toSec()
                       << " seconds (" << n_pubs << " publishers on topic)");
}
/*//}*/

/*//{ getZVelUntilted() */
template <int n_measurements>
std::optional<double> Correction<n_measurements>::getZVelUntilted(const nav_msgs::Odometry& msg) {

  // untilt the desired acceleration vector
  geometry_msgs::PointStamped vel;
  vel.point.x         = msg.twist.twist.linear.x;
  vel.point.y         = msg.twist.twist.linear.y;
  vel.point.z         = msg.twist.twist.linear.z;
  vel.header.frame_id = ch_->frames.ns_fcu;
  vel.header.stamp    = msg.header.stamp;

  auto res = ch_->transformer->transformSingle(vel, ch_->frames.ns_fcu_untilted);
  if (res) {
    return res.value().point.z;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform from %s to %s failed", getNamespacedName().c_str(), vel.header.frame_id.c_str(),
                      ch_->frames.ns_fcu_untilted.c_str());
    return {};
  }
}
/*//}*/

/*//{ getVelInFrame() */
template <int n_measurements>
bool Correction<n_measurements>::getVelInFrame(const nav_msgs::Odometry& msg, const std::string frame, measurement_t& measurement_out) {

  // velocity from mavros is already in global frame
  if (msg.header.frame_id == "map" && msg.child_frame_id == "base_link") {
    measurement_out(0) = msg.twist.twist.linear.x; 
    measurement_out(1) = msg.twist.twist.linear.y; 
    return true;
  }

  geometry_msgs::Vector3Stamped body_vel;
  body_vel.header   = msg.header;
  body_vel.vector.x = msg.twist.twist.linear.x;
  body_vel.vector.y = msg.twist.twist.linear.y;
  body_vel.vector.z = msg.twist.twist.linear.z;

  geometry_msgs::Vector3Stamped transformed_vel;
  auto                          res = ch_->transformer->transformSingle(body_vel, frame);
  if (res) {
    transformed_vel    = res.value();
    measurement_out(0) = transformed_vel.vector.x;
    measurement_out(1) = transformed_vel.vector.y;
    return true;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform of velocity from %s to %s failed.", getNamespacedName().c_str(), body_vel.header.frame_id.c_str(), frame.c_str());
    return false;
  }
}
/*//}*/

/*//{ transformRtkToFcu() */
template <int n_measurements>
geometry_msgs::Pose Correction<n_measurements>::transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const {

  geometry_msgs::PoseStamped pose_tmp = pose_in;

  // inject current orientation into rtk pose
  auto res1 = ch_->transformer->getTransform(ch_->frames.ns_fcu_untilted, ch_->frames.ns_fcu, ros::Time::now());
  if (res1) {
    pose_tmp.pose.orientation = res1.value().transform.rotation;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not obtain transform from %s to %s. Not using this correction.", getNamespacedName().c_str(),
                       ch_->frames.ns_fcu_untilted.c_str(), ch_->frames.ns_fcu.c_str());
    return pose_in.pose;
  }

  ROS_INFO("[%s]: debug antenna in rtk_origin: %.2f %.2f %.2f", getNamespacedName().c_str(), pose_tmp.pose.position.x, pose_tmp.pose.position.y, pose_tmp.pose.position.z);

  // invert tf
  tf2::Transform             tf_utm_to_antenna = Support::tf2FromPose(pose_tmp.pose);
  geometry_msgs::PoseStamped utm_in_antenna;
  utm_in_antenna.pose            = Support::poseFromTf2(tf_utm_to_antenna.inverse());
  utm_in_antenna.header.stamp    = pose_in.header.stamp;
  utm_in_antenna.header.frame_id = ch_->frames.ns_rtk_antenna;

  ROS_INFO("[%s]: debug rtk_origin in antenna: %.2f %.2f %.2f", getNamespacedName().c_str(), utm_in_antenna.pose.position.x, utm_in_antenna.pose.position.y, utm_in_antenna.pose.position.z);


  // transform to fcu
  geometry_msgs::PoseStamped utm_in_fcu;
  utm_in_fcu.header.frame_id = ch_->frames.ns_fcu;
  utm_in_fcu.header.stamp    = pose_in.header.stamp;
  auto res2                  = ch_->transformer->transformSingle(utm_in_antenna, ch_->frames.ns_fcu);

  if (res2) {
    utm_in_fcu = res2.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not using this correction.", getNamespacedName().c_str(), ch_->frames.ns_fcu.c_str());
    return pose_in.pose;
  }

  ROS_INFO("[%s]: debug rtk_origin in fcu: %.2f %.2f %.2f", getNamespacedName().c_str(), utm_in_fcu.pose.position.x, utm_in_fcu.pose.position.y, utm_in_fcu.pose.position.z);

  // invert tf
  tf2::Transform      tf_fcu_to_utm = Support::tf2FromPose(utm_in_fcu.pose);
  geometry_msgs::Pose fcu_in_utm    = Support::poseFromTf2(tf_fcu_to_utm.inverse());

  ROS_INFO("[%s]: debug fcu in rtk_origin: %.2f %.2f %.2f", getNamespacedName().c_str(), fcu_in_utm.position.x, fcu_in_utm.position.y, fcu_in_utm.position.z);

  return fcu_in_utm;
}
/*//}*/

/*//{ isMsgDelayOk() */
template <int n_measurements>
void Correction<n_measurements>::checkMsgDelay(const ros::Time& msg_time) {

  const double delay = (ros::Time::now() - msg_time).toSec();
  if (delay > msg_delay_limit_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: message too delayed (%.4f s)", getNamespacedName().c_str(), delay);
    is_delay_ok_ = false;
  } else {
    is_delay_ok_ = true;
  }
}
/*//}*/

/*//{ publishCorrection() */
template <int n_measurements>
void Correction<n_measurements>::publishCorrection(const measurement_t& measurement, const ros::Time& measurement_stamp) {
  EstimatorCorrection msg;
  msg.header.stamp    = measurement_stamp;
  msg.header.frame_id = ns_frame_id_;
  msg.name            = name_;
  msg.estimator_name  = est_name_;
  msg.state_id        = state_id_;
  msg.covariance.resize(n_measurements * n_measurements);
  for (int i = 0; i < measurement.rows(); i++) {
    msg.state.push_back(measurement(i));
    msg.covariance[n_measurements * i + i] = getR();
  }

  ph_correction_.publish(msg);
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
