#pragma once
#ifndef ESTIMATORS_CORRECTION_H
#define ESTIMATORS_CORRECTION_H

#include <Eigen/Dense>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/gps_conversions.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/EstimatorCorrection.h>

#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/SetBool.h>

#include <functional>

#include <estimation_manager/types.h>
#include <estimation_manager/support.h>
#include <estimation_manager/common_handlers.h>

#include "processors/processor.h"
#include "processors/proc_median_filter.h"
#include "processors/proc_saturate.h"
#include "processors/proc_excessive_tilt.h"

#include "mrs_uav_state_estimators/CorrectionConfig.h"


namespace mrs_uav_state_estimators
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
  UNKNOWN,
  ODOMETRY,
  POSE,
  POSECOV,
  RANGE,
  RTK_GPS,
  POINT,
  VECTOR,
  QUAT,
} MessageType_t;
const int n_MessageType_t = 9;

const std::map<std::string, MessageType_t> map_msg_type{{"nav_msgs/Odometry", MessageType_t::ODOMETRY},
                                                        {"geometry_msgs/PoseStamped", MessageType_t::POSE},
                                                        {"geometry_msgs/PoseWithCovarianceStamped", MessageType_t::POSECOV},
                                                        {"sensor_msgs/Range", MessageType_t::RANGE},
                                                        {"mrs_msgs/RtkGps", MessageType_t::RTK_GPS},
                                                        {"geometry_msgs/PointStamped", MessageType_t::POINT},
                                                        {"geometry_msgs/Vector3Stamped", MessageType_t::VECTOR},
                                                        {"geometry_msgs/QuaternionStamped", MessageType_t::QUAT}};

template <int n_measurements>
class Correction {

  using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;
  using StateId_t        = mrs_uav_managers::estimation_manager::StateId_t;

public:
  typedef Eigen::Matrix<double, n_measurements, 1>         measurement_t;
  typedef mrs_lib::DynamicReconfigureMgr<CorrectionConfig> drmgr_t;

  struct MeasurementStamped
  {
    ros::Time     stamp;
    measurement_t value;
  };

public:
  Correction(ros::NodeHandle& nh, const std::string& est_name, const std::string& name, const std::string& frame_id, const EstimatorType_t& est_type,
             const std::shared_ptr<CommonHandlers_t>& ch, std::function<double(int, int)> fun_get_state);

  std::string getName() const;
  std::string getNamespacedName() const;
  std::string getPrintName() const;

  double    getR();
  StateId_t getStateId() const;

  bool             isHealthy();
  std::atomic_bool is_healthy_    = true;
  std::atomic_bool is_delay_ok_   = true;
  std::atomic_bool is_dt_ok_      = true;
  std::atomic_bool is_nan_free_   = true;
  std::atomic_bool got_first_msg_ = false;

  int counter_nan_ = 0;

  std::optional<MeasurementStamped> getRawCorrection();
  std::optional<MeasurementStamped> getProcessedCorrection();

private:
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                       sh_odom_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>               sh_pose_s_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_pose_wcs_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range>                       sh_range_;
  mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>                         sh_rtk_;
  mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>              sh_point_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>            sh_vector_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>         sh_quat_;

  ros::ServiceServer ser_toggle_range_;
  bool               callbackToggleRange(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool               range_enabled_ = true;

  void timeoutCallback(const std::string& topic, const ros::Time& last_msg, const int n_pubs);

  mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection> ph_correction_raw_;
  mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection> ph_correction_proc_;

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

  std::optional<measurement_t> getCorrectionFromOdometry(const nav_msgs::OdometryConstPtr msg);
  std::optional<measurement_t> getCorrectionFromRange(const sensor_msgs::RangeConstPtr msg);
  std::optional<measurement_t> getCorrectionFromRtk(const mrs_msgs::RtkGpsConstPtr msg);
  std::optional<measurement_t> getCorrectionFromPoint(const geometry_msgs::PointStampedConstPtr msg);
  std::optional<measurement_t> getCorrectionFromVector(const geometry_msgs::Vector3StampedConstPtr msg);
  std::optional<measurement_t> getCorrectionFromQuat(const geometry_msgs::QuaternionStampedConstPtr msg);
  std::optional<measurement_t> getZVelUntilted(const geometry_msgs::Vector3& msg, const std_msgs::Header& header);
  std::optional<measurement_t> getVelInFrame(const geometry_msgs::Vector3& vel, const std_msgs::Header& header, const std::string frame);

  std::optional<geometry_msgs::Pose> transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const;

  void   checkMsgDelay(const ros::Time& msg_time);
  double msg_delay_limit_;

  double time_since_last_msg_limit_;

  std::shared_ptr<Processor<n_measurements>> createProcessorFromName(const std::string& name, ros::NodeHandle& nh);
  bool                                       process(measurement_t& measurement);

  std::vector<std::string>                                                    processor_names_;
  std::unordered_map<std::string, std::shared_ptr<Processor<n_measurements>>> processors_;

  std::function<double(int, int)> fun_get_state_;

  void publishCorrection(const MeasurementStamped& measurement_stamped, mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>& ph_corr);
};

/*//{ constructor */
template <int n_measurements>
Correction<n_measurements>::Correction(ros::NodeHandle& nh, const std::string& est_name, const std::string& name, const std::string& ns_frame_id,
                                       const EstimatorType_t& est_type, const std::shared_ptr<CommonHandlers_t>& ch,
                                       std::function<double(int, int)> fun_get_state)
    : est_name_(est_name), name_(name), ns_frame_id_(ns_frame_id), est_type_(est_type), ch_(ch), fun_get_state_(fun_get_state) {

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, getPrintName());
  param_loader.setPrefix(getNamespacedName() + "/");

  std::string msg_type_string;
  param_loader.loadParam("message/type", msg_type_string);
  if (map_msg_type.find(msg_type_string) == map_msg_type.end()) {
    ROS_ERROR("[%s]: wrong message type: %s of correction %s", getPrintName().c_str(), msg_type_string.c_str(), getName().c_str());
    ros::shutdown();
  }
  msg_type_ = map_msg_type.at(msg_type_string);

  param_loader.loadParam("message/topic", msg_topic_);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;
  param_loader.loadParam("message/limit/delay", msg_delay_limit_);
  param_loader.loadParam("message/limit/time_since_last", time_since_last_msg_limit_);

  int state_id_tmp;
  param_loader.loadParam("state_id", state_id_tmp);
  if (state_id_tmp < n_StateId_t) {
    state_id_ = static_cast<StateId_t>(state_id_tmp);
  } else {
    ROS_ERROR("[%s]: wrong state id: %d of correction %s", getPrintName().c_str(), state_id_tmp, getName().c_str());
    ros::shutdown();
  }
  param_loader.loadParam("noise", R_);

  // | --------------- processors initialization --------------- |
  param_loader.loadParam("processors", processor_names_);

  for (auto proc_name : processor_names_) {
    processors_[proc_name] = createProcessorFromName(proc_name, nh);
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_               = std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), getPrintName());
  drmgr_->config.noise = R_;

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
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
    case MessageType_t::POSE: {
      // TODO implement
      /* sh_pose_s_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::POSECOV: {
      // TODO implement
      /* sh_pose_wcs_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::RANGE: {
      sh_range_                   = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      const std::size_t found     = ros::this_node::getName().find_last_of("/");
      std::string       node_name = ros::this_node::getName().substr(found + 1);
      /* ser_toggle_range_ = nh.advertiseService("/" + ch_->uav_name + "/" + node_name + "/" + getNamespacedName() + "/toggle_range_in",
       * &Correction::callbackToggleRange, this); */
      ser_toggle_range_ =
          nh.advertiseService(ros::this_node::getName() + "/" + getNamespacedName() + "/toggle_range_in", &Correction::callbackToggleRange, this);
      break;
    }
    case MessageType_t::RTK_GPS: {
      sh_rtk_ = mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::POINT: {
      sh_point_ = mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::VECTOR: {
      sh_vector_ = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::QUAT: {
      sh_quat_ = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, msg_topic_, &Correction::timeoutCallback, this);
      break;
    }
    case MessageType_t::UNKNOWN: {
      ROS_ERROR("[%s]: UNKNOWN message type of correction", getPrintName().c_str());
      break;
    }
    default: {
      ROS_ERROR("[%s]: unhandled message type", getPrintName().c_str());
    }
  }

  // | --------------- initialize publish handlers -------------- |
  ph_correction_raw_  = mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>(nh, est_name_ + "/" + getName() + "_raw", 1);
  ph_correction_proc_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>(nh, est_name_ + "/" + getName() + "_proc", 1);
}
/*//}*/

/*//{ getName() */
template <int n_measurements>
std::string Correction<n_measurements>::getName() const {
  return name_;
}
/*//}*/

/*//{ getNamespacedName() */
template <int n_measurements>
std::string Correction<n_measurements>::getNamespacedName() const {
  return est_name_ + "/" + name_;
}
/*//}*/

/*//{ getPrintName() */
template <int n_measurements>
std::string Correction<n_measurements>::getPrintName() const {
  return ch_->nodelet_name + "/" + est_name_ + "/" + name_;
}
/*//}*/

/*//{ getR() */
template <int n_measurements>
double Correction<n_measurements>::getR() {
  std::scoped_lock lock(mtx_R_);
  R_ = drmgr_->config.noise;
  return R_;
}
/*//}*/

/*//{ getStateId() */
template <int n_measurements>
StateId_t Correction<n_measurements>::getStateId() const {
  return state_id_;
}
/*//}*/

/*//{ isHealthy() */
template <int n_measurements>
bool Correction<n_measurements>::isHealthy() {

  if (!is_dt_ok_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: dt not ok", getPrintName().c_str());
  }

  if (!is_delay_ok_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: delay not ok", getPrintName().c_str());
  }

  is_healthy_ = is_healthy_ && is_dt_ok_ && is_delay_ok_;

  return is_healthy_;
}
/*//}*/

/*//{ getRawCorrection() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::MeasurementStamped> Correction<n_measurements>::getRawCorrection() {

  MeasurementStamped measurement_stamped;

  switch (msg_type_) {

    case MessageType_t::ODOMETRY: {

      if (!sh_odom_.newMsg()) {
        return {};
      }

      auto msg                  = sh_odom_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }
      auto res = getCorrectionFromOdometry(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::POSE: {
      // TODO implement
      /* return getCorrectionFromPoseS(msg); */
      is_healthy_ = false;
      return {};
      break;
    }

    case MessageType_t::POSECOV: {
      // TODO implement
      /* return getCorrectionFromPoseWCS(msg); */
      is_healthy_ = false;
      return {};
      break;
    }

    case MessageType_t::RANGE: {

      if (!range_enabled_) {
        ROS_INFO_THROTTLE(1.0, "[%s]: fusing range corrections is disabled", getPrintName().c_str());
        return {};
      }

      if (!sh_range_.newMsg()) {
        return {};
      }

      auto msg                  = sh_range_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }

      auto res = getCorrectionFromRange(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::RTK_GPS: {

      if (!sh_rtk_.newMsg()) {
        /* ROS_ERROR(" no new rtk msg"); */
        return {};
      }

      auto msg                  = sh_rtk_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }

      auto res = getCorrectionFromRtk(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }

      break;
    }

    case MessageType_t::POINT: {

      if (!sh_point_.newMsg()) {
        return {};
      }

      auto msg                  = sh_point_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }
      auto res = getCorrectionFromPoint(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::VECTOR: {

      if (!sh_vector_.newMsg()) {
        return {};
      }

      auto msg                  = sh_vector_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }
      auto res = getCorrectionFromVector(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::QUAT: {

      if (!sh_quat_.newMsg()) {
        return {};
      }

      auto msg                  = sh_quat_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      checkMsgDelay(measurement_stamped.stamp);

      if (!is_delay_ok_) {
        return {};
      }
      auto res = getCorrectionFromQuat(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    default: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: this type of correction is not implemented in getCorrectionFromMessage()", getPrintName().c_str());
      is_healthy_ = false;
      return {};
    }
  }

  // check for nans
  is_nan_free_ = true;
  for (int i = 0; i < measurement_stamped.value.rows(); i++) {
    if (!std::isfinite(measurement_stamped.value(i))) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in correction. Total NaNs: %d", getPrintName().c_str(), ++counter_nan_);
      is_nan_free_ = false;
      return {};
    }
  }
  /* ROS_INFO("[%s]: debug: rtk correction: %f %f", getPrintName().c_str(), measurement(0), measurement(1)); */

  got_first_msg_ = true;
  publishCorrection(measurement_stamped, ph_correction_raw_);

  return measurement_stamped;
}
/*//}*/

/*//{ getProcessedCorrection() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::MeasurementStamped> Correction<n_measurements>::getProcessedCorrection() {

  MeasurementStamped measurement_stamped;
  auto               res = getRawCorrection();
  if (res) {
    MeasurementStamped measurement_stamped = res.value();
    if (process(measurement_stamped.value)) {
      publishCorrection(measurement_stamped, ph_correction_proc_);
      return measurement_stamped;
    } else {
      return {};  // invalid correction
    }
  } else {
    return {};  // invalid correction
  }
}  // namespace mrs_uav_state_estimation
/*//}*/

/*//{ getCorrectionFromOdometry() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromOdometry(const nav_msgs::OdometryConstPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->pose.pose.position.x;
          measurement(1) = msg->pose.pose.position.y;
          return measurement;
          break;
        }

        case StateId_t::VELOCITY: {
          std_msgs::Header header = msg->header;
          header.frame_id         = ch_->frames.ns_fcu;  // message in odometry is publisher in body frame
          auto res                = getVelInFrame(msg->twist.twist.linear, header, ns_frame_id_ + "_att_only");
          if (res) {
            measurement_t measurement;
            measurement = res.value();
            return measurement;
          } else {
            return {};
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->pose.pose.position.z;
          return measurement;
          break;
        }

        case StateId_t::VELOCITY: {
          std_msgs::Header header = msg->header;
          header.frame_id         = ch_->frames.ns_fcu;
          auto res                = getZVelUntilted(msg->twist.twist.linear, header);
          if (res) {
            measurement_t measurement;
            measurement = res.value();
            return measurement;
          } else {
            return {};
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

          /* case StateId_t::VELOCITY: { */
          /*   try { */
          /*     measurement_t measurement; */
          /*     measurement(0) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeadingRate(msg->twist.twist.angular); */
          /*     return measurement; */
          /*   } */
          /*   catch (...) { */
          /*     ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading rate (getCorrectionFromOdometry())", getPrintName().c_str()); */
          /*     return {}; */
          /*   } */
          /*   break; */
          /* } */

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  ROS_ERROR("[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ getCorrectionFromRange() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRange(const sensor_msgs::RangeConstPtr msg) {

  geometry_msgs::PoseStamped range_point;

  range_point.header           = msg->header;
  range_point.pose.position.x  = msg->range;
  range_point.pose.position.y  = 0;
  range_point.pose.position.z  = 0;
  range_point.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  auto res = ch_->transformer->transformSingle(range_point, ch_->frames.ns_fcu_untilted);

  Correction::measurement_t measurement;

  if (res) {
    measurement(0) = -res.value().pose.position.z;
    return measurement;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform range measurement to %s. Not using this correction.", getPrintName().c_str(),
                       ch_->frames.ns_fcu_untilted.c_str());
    return {};
  }
}
/*//}*/

/*//{ getCorrectionFromRtk() */
/* template <int n_measurements, typename Correction<n_measurements>::measurement_t> */
/* std::optional<Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRtk(const mrs_msgs::RtkGpsConstPtr msg) { */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRtk(const mrs_msgs::RtkGpsConstPtr msg) {

  geometry_msgs::PoseStamped rtk_pos;

  if (msg->header.frame_id == "gps") {

    if (!std::isfinite(msg->gps.latitude)) {
      ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->latitude\"!!!", getPrintName().c_str());
      return {};
    }

    if (!std::isfinite(msg->gps.longitude)) {
      ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in RTK variable \"msg->longitude\"!!!", getPrintName().c_str());
      return {};
    }

    rtk_pos.header = msg->header;
    mrs_lib::UTM(msg->gps.latitude, msg->gps.longitude, &rtk_pos.pose.position.x, &rtk_pos.pose.position.y);
    rtk_pos.pose.position.z  = msg->gps.altitude;
    rtk_pos.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  } else if (msg->header.frame_id == "utm") {

    rtk_pos.pose = msg->pose.pose;

  } else {

    ROS_INFO_THROTTLE(1.0, "[%s]: RTK message has unknown frame_id: '%s'", getPrintName().c_str(), msg->header.frame_id.c_str());
  }

  rtk_pos.pose.position.x -= ch_->utm_origin.x;
  rtk_pos.pose.position.y -= ch_->utm_origin.y;

  Correction::measurement_t measurement;

  // transform the RTK position from antenna to FCU
  auto res = transformRtkToFcu(rtk_pos);
  if (res) {
    rtk_pos.pose = res.value();
  } else {
    return {};
  }

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = rtk_pos.pose.position.x;
          measurement(1) = rtk_pos.pose.position.y;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromRtk() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = rtk_pos.pose.position.z;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromRtk() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    case EstimatorType_t::HEADING: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: should not be possible to get into this branch of getCorrectionFromRtk() switch", getPrintName().c_str());
      return {};
      break;
    }
  }

  ROS_ERROR("[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ getCorrectionFromPoint() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromPoint(
    const geometry_msgs::PointStampedConstPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->point.x;
          measurement(1) = msg->point.y;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->point.z;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    default: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
      return {};
    }
  }


  ROS_ERROR("[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ getCorrectionFromVector() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromVector(
    const geometry_msgs::Vector3StampedConstPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          auto res = getVelInFrame(msg->vector, msg->header, ns_frame_id_ + "_att_only");
          if (res) {
            measurement_t measurement;
            measurement = res.value();
            return measurement;
          } else {
            return {};
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          auto res = getZVelUntilted(msg->vector, msg->header);
          if (res) {
            measurement_t measurement;
            measurement = res.value();
            return measurement;
          } else {
            return {};
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

          // TODO: needs orientation

          /* case StateId_t::VELOCITY: { */
          /*   try { */
          /*     measurement_t measurement; */
          /*     measurement(0) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeadingRate(msg->twist.twist.angular); */
          /*     return measurement; */
          /*   } */
          /*   catch (...) { */
          /*     ROS_ERROR_THROTTLE(1.0, "[%s]: Exception caught during getting heading rate (getCorrectionFromOdometry())", getPrintName().c_str()); */
          /*     return {}; */
          /*   } */
          /*   break; */
          /* } */

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  ROS_ERROR("[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ getCorrectionFromQuat() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromQuat(
    const geometry_msgs::QuaternionStampedConstPtr msg) {

  switch (est_type_) {

      // handle lateral estimators
      /* case EstimatorType_t::LATERAL: { */

      /*   default: { */
      /*     ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str()); */
      /*     return {}; */
      /*   } break; */
      /* } */

      /* // handle altitude estimators */
      /* case EstimatorType_t::ALTITUDE: { */

      /* default: { */
      /*     ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str()); */
      /*     return {}; */
      /*   } break; */
      /* } */

      /* // handle heading estimators */
      /* case EstimatorType_t::HEADING: { */

      /*   switch (state_id_) { */

      /*     /1* default: { *1/ */
      /*       ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str()); */
      /*       return {}; */
      /*     } */
      /*   } */
    default: {
      ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
      return {};
    }
  }

  ROS_ERROR("[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ timeoutCallback() */
template <int n_measurements>
void Correction<n_measurements>::timeoutCallback(const std::string& topic, const ros::Time& last_msg, const int n_pubs) {
  if (got_first_msg_) {
    is_dt_ok_ = false;
  }
  ROS_WARN_THROTTLE(5.0, "[%s]: Did not receive message from topic '%s' for %.2f seconds (%d publishers on topic)", getPrintName().c_str(), topic.c_str(),
                    (ros::Time::now() - last_msg).toSec(), n_pubs);
}
/*//}*/

/* //{ callbackToggleRange() */
template <int n_measurements>
bool Correction<n_measurements>::callbackToggleRange(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!range_enabled_ && req.data) {
    processors_["saturate"]->toggle(true);
  }

  range_enabled_ = req.data;

  // after enabling range we want to start correcting the altitude slowly

  res.success = true;
  res.message = (range_enabled_ ? "Range enabled" : "Range disabled");

  if (range_enabled_) {

    ROS_INFO("[%s]: Range enabled.", getPrintName().c_str());

  } else {

    ROS_INFO("[%s]: Range disabled", getPrintName().c_str());
  }

  return true;
}

//}

/*//{ getZVelUntilted() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getZVelUntilted(const geometry_msgs::Vector3& msg,
                                                                                                              const std_msgs::Header&       header) {

  // untilt the desired vector
  geometry_msgs::PointStamped vel;
  vel.point.x = msg.x;
  vel.point.y = msg.y;
  vel.point.z = msg.z;
  vel.header  = header;
  /* vel.header.frame_id = ch_->frames.ns_fcu; */
  vel.header.stamp = header.stamp;

  auto res = ch_->transformer->transformSingle(vel, ch_->frames.ns_fcu_untilted);
  if (res) {
    measurement_t measurement;
    measurement(0) = res.value().point.z;
    return measurement;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform from %s to %s failed", getPrintName().c_str(), vel.header.frame_id.c_str(), ch_->frames.ns_fcu_untilted.c_str());
    return {};
  }
}
/*//}*/

/*//{ getVelInFrame() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getVelInFrame(const geometry_msgs::Vector3& vel,
                                                                                                            const std_msgs::Header&       header,
                                                                                                            const std::string             frame) {

  measurement_t measurement;

  geometry_msgs::Vector3Stamped body_vel;
  body_vel.header   = header;
  body_vel.vector.x = vel.x;
  body_vel.vector.y = vel.y;
  body_vel.vector.z = vel.z;

  geometry_msgs::Vector3Stamped transformed_vel;
  auto                          res = ch_->transformer->transformSingle(body_vel, frame);
  if (res) {
    transformed_vel = res.value();
    measurement(0)  = transformed_vel.vector.x;
    measurement(1)  = transformed_vel.vector.y;
    return measurement;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform of velocity from %s to %s failed.", getPrintName().c_str(), body_vel.header.frame_id.c_str(), frame.c_str());
    return {};
  }
}
/*//}*/

/*//{ transformRtkToFcu() */
template <int n_measurements>
std::optional<geometry_msgs::Pose> Correction<n_measurements>::transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const {

  geometry_msgs::PoseStamped pose_tmp = pose_in;

  // inject current orientation into rtk pose
  auto res1 = ch_->transformer->getTransform(ch_->frames.ns_fcu_untilted, ch_->frames.ns_fcu, ros::Time::now());
  if (res1) {
    pose_tmp.pose.orientation = res1.value().transform.rotation;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not obtain transform from %s to %s. Not using this correction.", getPrintName().c_str(),
                       ch_->frames.ns_fcu_untilted.c_str(), ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform             tf_utm_to_antenna = Support::tf2FromPose(pose_tmp.pose);
  geometry_msgs::PoseStamped utm_in_antenna;
  utm_in_antenna.pose            = Support::poseFromTf2(tf_utm_to_antenna.inverse());
  utm_in_antenna.header.stamp    = pose_in.header.stamp;
  utm_in_antenna.header.frame_id = ch_->frames.ns_rtk_antenna;

  // transform to fcu
  geometry_msgs::PoseStamped utm_in_fcu;
  utm_in_fcu.header.frame_id = ch_->frames.ns_fcu;
  utm_in_fcu.header.stamp    = pose_in.header.stamp;
  auto res2                  = ch_->transformer->transformSingle(utm_in_antenna, ch_->frames.ns_fcu);

  if (res2) {
    utm_in_fcu = res2.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Could not transform pose to %s. Not using this correction.", getPrintName().c_str(), ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform      tf_fcu_to_utm = Support::tf2FromPose(utm_in_fcu.pose);
  geometry_msgs::Pose fcu_in_utm    = Support::poseFromTf2(tf_fcu_to_utm.inverse());

  return fcu_in_utm;
}
/*//}*/

/*//{ checkMsgDelay() */
template <int n_measurements>
void Correction<n_measurements>::checkMsgDelay(const ros::Time& msg_time) {

  const double delay = (ros::Time::now() - msg_time).toSec();
  if (delay > msg_delay_limit_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: message too delayed (%.4f s)", getPrintName().c_str(), delay);
    is_delay_ok_ = false;
  } else {
    is_delay_ok_ = true;
  }
}
/*//}*/

/*//{ createProcessorFromName() */
template <int n_measurements>
std::shared_ptr<Processor<n_measurements>> Correction<n_measurements>::createProcessorFromName(const std::string& name, ros::NodeHandle& nh) {

  if (name == "median_filter") {
    return std::make_shared<ProcMedianFilter<n_measurements>>(nh, getNamespacedName(), name, ch_);
  } else if (name == "saturate") {
    return std::make_shared<ProcSaturate<n_measurements>>(nh, getNamespacedName(), name, ch_, state_id_, fun_get_state_);
  } else if (name == "excessive_tilt") {
    return std::make_shared<ProcExcessiveTilt<n_measurements>>(nh, getNamespacedName(), name, ch_);
  } else {
    ROS_ERROR("[%s]: requested invalid processor %s", getPrintName().c_str(), name.c_str());
    ros::shutdown();
  }
  return std::shared_ptr<Processor<n_measurements>>(nullptr);
}
/*//}*/

/*//{ process() */
template <int n_measurements>
bool Correction<n_measurements>::process(Correction<n_measurements>::measurement_t& measurement) {
  bool ok_flag = true;
  for (auto proc_name :
       processor_names_) {  // need to access the estimators in the specific order from the config (e.g. median filter should go before saturation etc.)
    if (!processors_[proc_name]->process(measurement)) {
      ok_flag = false;
    }
  }
  return ok_flag;
}
/*//}*/

/*//{ publishCorrection() */
template <int n_measurements>
void Correction<n_measurements>::publishCorrection(const MeasurementStamped&                                 measurement_stamped,
                                                   mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>& ph_corr) {
  mrs_msgs::EstimatorCorrection msg;
  msg.header.stamp    = measurement_stamped.stamp;
  msg.header.frame_id = ns_frame_id_;
  msg.name            = name_;
  msg.estimator_name  = est_name_;
  msg.state_id        = state_id_;
  msg.covariance.resize(n_measurements * n_measurements);
  for (int i = 0; i < measurement_stamped.value.rows(); i++) {
    msg.state.push_back(measurement_stamped.value(i));
    msg.covariance[n_measurements * i + i] = getR();
  }

  ph_corr.publish(msg);
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_CORRECTION_H
