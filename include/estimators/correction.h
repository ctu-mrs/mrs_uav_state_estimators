#pragma once
#ifndef CORRECTION_H
#define CORRECTION_H

#include <Eigen/Dense>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

#include "types.h"

namespace mrs_uav_state_estimation
{

// TODO is needed?
typedef enum
{
  LATERAL,
  ALTITUDE,
  HEADING
} EstimatorType_t;

typedef enum
{
  ODOMETRY,
  POSE_S,
  POSE_WCS,
  RANGE,
} MessageType_t;

template <int n_measurements>
class Correction {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurements_t;

public:
  Correction(ros::NodeHandle& nh, const std::string& name, const std::string& uav_name);

  std::string getName();

  bool      isHealthy();
  double    getR();
  StateId_t getStateId();

  std::optional<double> getValue();

private:
  const std::string name_;
  const std::string uav_name_;

  MessageType_t msg_type_;
  std::string   msg_topic_;

  std::atomic_bool is_healthy_;

  double    R_;
  StateId_t state_id_;
};

// constructor
template <int n_measurements>
Correction<n_measurements>::Correction(ros::NodeHandle& nh, const std::string& name, const std::string& uav_name) : name_(name), uav_name_(uav_name) {

  mrs_lib::ParamLoader param_loader(nh, getName());
  param_loader.loadParam("message/type", msg_type_);
  param_loader.loadParam("message/topic", msg_topic_);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  switch (msg_type_) {
    case MessageType_t::ODOMETRY: {
      sh_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic_);
      break;
    }
    case MessageType_t::POSE_S: {
      sh_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, topic_);
      break;
    }
    case MessageType_t::POSE_WCS: {
      sh_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, topic_);
      break;
    }
    case MessageType_t::RANGE: {
      sh_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, topic_);
      break;
    }
  }
}

std::string Correction::getName() {
  return name_;
}

bool Correction::isHealthy() {
  return is_healthy_;
}

double Correction::getR() {
  return R_;
}

StateId_t Correction::getHIdx() {
  return H_idx_;
}

template <int n_measurements>
std::optional<measurement_t> Correction::getCorrection() {

  // check age of message
  const double time_since_last_msg_ = (ros::Time::now() - sh_.lastMsgTime()).toSec();
  if (sh_.hasMsg() && time_since_last_msg_ > msg_timeout_) {
    ROS_ERROT_THROTTLE(1.0, "[%s]: message too old (%.4f s)", getName().c_str(), time_since_last_msg_);
    return {};
  }

  // extract correct part from common message types
  const measurement_t measurement = getCorrectionFromMessage();

  // check for nans
  for (int i = 0; i < measurement.rows(); i++) {
    if (!std::isfinite(measurement(i))) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in correction", getName().c_str());
      return {};
    }
  }

  return measurement;
}

/*//{ getCorrectionFromMessage() */
template <int n_measurements>
measurement_t Correction<n_measurements>::getCorrectionFromMessage() {

  auto msg = sh_.getMsg();

  switch (msg_type_) {

    case MessageType_t::ODOMETRY: {
      return getCorrectionFromOdometry(msg);
      break;
    }

    case MessageType_t::POSE_S: {
      // TODO implement
      /* return getCorrectionFromPoseS(msg); */
      return {};
      break;
    }
    case MessageType_t::POSE_WCS: {
      // TODO implement
      /* return getCorrectionFromPoseWCS(msg); */
      return {};
      break;
    }
    case MessageType_t::RANGE: {
      return getCorrectionFromRange(msg);
      break;
    }
    default: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: this type of correction is not implemented in getCorrectionFromMessage()", getName().c_str());
      return {};
    }
  }
}
/*//}*/

/*//{ getCorrectionFromOdometry() */
template <int n_measurements>
measurement_t Correction<n_measurements>::getCorrectionFromOdometry(const std::shared_ptr<nav_msgs::Odometry>& msg) {

  measurement_t measurement;

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {
      switch (state_id_) {
        case StateId_t::POSITION: {
          measurement(0) = msg->pose.pose.position.x;
          measurement(1) = msg->pose.pose.position.y;
          break;
        }
        case StateId_t::VELOCITY: {
          measurement(0) = msg->twist.linear.x;
          measurement(1) = msg->twist.linear.y;
          break;
        }
        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {
      case StateId_t::POSITION: {
        measurement(0) = msg->pose.pose.position.z;
        break;
      }
      case StateId_t::VELOCITY: {
        measurement(0) = msg->twist.linear.z;
        break;
      }
      default: {
        ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getName().c_str());
        return {};
      }
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {
      case StateId_t::POSITION: {
        try {
          measurement(0) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
        }
        catch (...) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading (getCorrectionFromOdometry())", getName().c_str());
          return {};
        }
        break;
      }
      case StateId_t::VELOCITY: {
        try {
          measurement(0) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeadingRate(msg->twist.angular);
        }
        catch (...) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading rate (getCorrectionFromOdometry())", getName().c_str());
          return {};
        }
        break;
      }
      default: {
        ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getName().c_str());
        return {};
      }
    }
  }

  return measurement;
}
/*//}*/

/*//{ getCorrectionFromRange() */
template <int n_measurements>
measurement_t Correction<n_measurements>::getCorrectionFromRange(const std::shared_ptr<sensor_msgs::Range>& msg) {

  measurement_t measurement;
  measurement(0) = msg->range;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
