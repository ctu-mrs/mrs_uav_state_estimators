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
#include "processors/proc_tf_to_world.h"

#include "mrs_uav_state_estimators/CorrectionConfig.h"


namespace mrs_uav_state_estimators
{

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
             const std::shared_ptr<CommonHandlers_t>& ch, std::function<double(int, int)> fun_get_state,
             std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction);

  std::string getName() const;
  std::string getNamespacedName() const;
  std::string getPrintName() const;

  double    getR();
  void      setR(const double R);
  StateId_t getStateId() const;

  bool             isHealthy();
  ros::Time        healthy_time_;
  std::atomic_bool is_healthy_    = true;
  std::atomic_bool is_delay_ok_   = true;
  std::atomic_bool is_dt_ok_      = true;
  std::atomic_bool is_nan_free_   = true;
  std::atomic_bool got_first_msg_ = false;

  int counter_nan_ = 0;

  std::optional<MeasurementStamped> getRawCorrection();
  std::optional<MeasurementStamped> getProcessedCorrection();

private:
  std::atomic_bool is_initialized_ = false;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
  void                                          callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);
  std::optional<measurement_t>                  getCorrectionFromOdometry(const nav_msgs::OdometryConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sh_pose_s_;
  void                                                  callbackPoseStamped(mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>& wrp);
  std::optional<measurement_t>                          getCorrectionFromPoseStamped(const geometry_msgs::PoseStampedConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_pose_wcs_;

  mrs_lib::SubscribeHandler<mrs_msgs::RtkGps> sh_rtk_;
  void                                        callbackRtk(mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>& wrp);
  std::optional<measurement_t>                getCorrectionFromRtk(const mrs_msgs::RtkGpsConstPtr msg);
  void                                        getAvgRtkInitZ(const double rtk_z);
  bool                                        got_avg_init_rtk_z_ = false;
  double                                      rtk_init_z_avg_     = 0.0;
  int                                         got_rtk_counter_    = 0;

  mrs_lib::SubscribeHandler<geometry_msgs::PointStamped> sh_point_;
  void                                                   callbackPoint(mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>& wrp);
  std::optional<measurement_t>                           getCorrectionFromPoint(const geometry_msgs::PointStampedConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> sh_vector_;
  std::optional<measurement_t>                             getCorrectionFromVector(const geometry_msgs::Vector3StampedConstPtr msg);
  void                                                     callbackVector(mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>& wrp);

  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_quat_;

  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_range_;
  std::optional<measurement_t>                  getCorrectionFromRange(const sensor_msgs::RangeConstPtr msg);
  void                                          callbackRange(mrs_lib::SubscribeHandler<sensor_msgs::Range>& wrp);
  ros::ServiceServer                            ser_toggle_range_;
  bool                                          callbackToggleRange(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool                                          range_enabled_ = true;

  void applyCorrection(const measurement_t& meas, const ros::Time& stamp);

  mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection> ph_correction_raw_;
  mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection> ph_correction_proc_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>      ph_delay_;

  const std::string                 est_name_;
  const std::string                 name_;
  const std::string                 ns_frame_id_;
  const EstimatorType_t             est_type_;
  std::shared_ptr<CommonHandlers_t> ch_;

  MessageType_t msg_type_;
  std::string   msg_topic_;
  double        msg_timeout_;

  double     R_;
  double     default_R_;
  double     R_coeff_;
  std::mutex mtx_R_;
  StateId_t  state_id_;
  bool       is_in_body_frame_ = true;

  std::unique_ptr<drmgr_t> drmgr_;

  std::optional<measurement_t> getCorrectionFromQuat(const geometry_msgs::QuaternionStampedConstPtr msg);
  std::optional<measurement_t> getZVelUntilted(const geometry_msgs::Vector3& msg, const std_msgs::Header& header);
  std::optional<measurement_t> getVelInFrame(const geometry_msgs::Vector3& vel_in, const std_msgs::Header& source_header, const std::string target_frame);

  std::optional<geometry_msgs::Pose> transformRtkToFcu(const geometry_msgs::PoseStamped& pose_in) const;

  void   checkMsgDelay(const ros::Time& msg_time);
  double msg_delay_limit_;
  double msg_delay_warn_limit_;

  double time_since_last_msg_limit_;

  std::shared_ptr<Processor<n_measurements>> createProcessorFromName(const std::string& name, ros::NodeHandle& nh);
  bool                     process(measurement_t& measurement);

  bool             isTimestampOk();
  bool             isMsgComing();
  std::atomic_bool first_timestamp_ = true;
  ros::Time        msg_time_;
  ros::Time        prev_msg_time_;
  std::mutex       mtx_msg_time_;

  std::vector<std::string>                                                    processor_names_;
  std::unordered_map<std::string, std::shared_ptr<Processor<n_measurements>>> processors_;

  std::function<double(int, int)>                            fun_get_state_;
  std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction_;

  void publishCorrection(const MeasurementStamped& measurement_stamped, mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>& ph_corr);
  void publishDelay(const double delay);
};

/*//{ constructor */
template <int n_measurements>
Correction<n_measurements>::Correction(ros::NodeHandle& nh, const std::string& est_name, const std::string& name, const std::string& ns_frame_id,
                                       const EstimatorType_t& est_type, const std::shared_ptr<CommonHandlers_t>& ch,
                                       std::function<double(int, int)>                            fun_get_state,
                                       std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction)
    : est_name_(est_name),
      name_(name),
      ns_frame_id_(ns_frame_id),
      est_type_(est_type),
      ch_(ch),
      fun_get_state_(fun_get_state),
      fun_apply_correction_(fun_apply_correction) {

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
  msg_delay_warn_limit_ = msg_delay_limit_ / 2;  // maybe specify this as a param?
  param_loader.loadParam("message/limit/time_since_last", time_since_last_msg_limit_);

  int state_id_tmp;
  param_loader.loadParam("state_id", state_id_tmp);
  if (state_id_tmp < n_StateId_t) {
    state_id_ = static_cast<StateId_t>(state_id_tmp);
  } else {
    ROS_ERROR("[%s]: wrong state id: %d of correction %s", getPrintName().c_str(), state_id_tmp, getName().c_str());
    ros::shutdown();
  }
  if (state_id_ == VELOCITY) {
    param_loader.loadParam("body_frame", is_in_body_frame_, true);
  }

  param_loader.loadParam("noise", R_);
  param_loader.loadParam("noise_unhealthy_coeff", R_coeff_);
  default_R_ = R_;

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
  drmgr_->update_config(drmgr_->config);

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  switch (msg_type_) {
    case MessageType_t::ODOMETRY: {
      sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, msg_topic_, &Correction::callbackOdometry, this);
      break;
    }
    case MessageType_t::POSE: {
      sh_pose_s_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, msg_topic_, &Correction::callbackPoseStamped, this);
      break;
    }
    case MessageType_t::POSECOV: {
      // TODO implement
      /* sh_pose_wcs_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::RANGE: {
      sh_range_                   = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, msg_topic_, &Correction::callbackRange, this);
      const std::size_t found     = ros::this_node::getName().find_last_of("/");
      std::string       node_name = ros::this_node::getName().substr(found + 1);
      ser_toggle_range_ =
          nh.advertiseService(ros::this_node::getName() + "/" + getNamespacedName() + "/toggle_range_in", &Correction::callbackToggleRange, this);
      break;
    }
    case MessageType_t::RTK_GPS: {
      sh_rtk_ = mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>(shopts, msg_topic_, &Correction::callbackRtk, this);
      break;
    }
    case MessageType_t::POINT: {
      sh_point_ = mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>(shopts, msg_topic_, &Correction::callbackPoint, this);
      break;
    }
    case MessageType_t::VECTOR: {
      sh_vector_ = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, msg_topic_, &Correction::callbackVector, this);
      break;
    }
    case MessageType_t::QUAT: {
      sh_quat_ = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, msg_topic_);
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
  if (ch_->debug_topics.correction) {
    ph_correction_raw_  = mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>(nh, est_name_ + "/correction/" + getName() + "/raw", 10);
    ph_correction_proc_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>(nh, est_name_ + "/correction/" + getName() + "/proc", 10);
  }
  if (ch_->debug_topics.corr_delay) {
    ph_delay_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh, est_name_ + "/correction/" + getName() + "/delay", 10);
  }

  healthy_time_ = ros::Time(0);

  is_initialized_ = true;
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
  default_R_         = drmgr_->config.noise;
  return R_;
}
/*//}*/

/*//{ setR() */
template <int n_measurements>
void Correction<n_measurements>::setR(const double R) {
  std::scoped_lock lock(mtx_R_);
  R_ = R;
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

  if (!is_initialized_) {
    return false;
  }

  is_dt_ok_ = isMsgComing();

  if (!is_delay_ok_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: delay not ok", getPrintName().c_str());
  }

  if (!is_healthy_) {
    if (is_dt_ok_ && is_delay_ok_) {
      if (healthy_time_ > ros::Time(10)) {
        is_healthy_ = true;
      }
    } else {
      healthy_time_ = ros::Time(0);
    }
  }

  is_healthy_ = is_healthy_ && is_dt_ok_ && is_delay_ok_;

  return is_healthy_;
}
/*//}*/

/*//{ getRawCorrection() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::MeasurementStamped> Correction<n_measurements>::getRawCorrection() {

  if (!is_initialized_) {
    return {};
  }

  MeasurementStamped measurement_stamped;

  switch (msg_type_) {

    case MessageType_t::ODOMETRY: {

      if (!sh_odom_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_odom_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* if (!isTimestampOk(measurement_stamped.stamp)) { */
      /*   return {}; */
      /* } */
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
      auto res = getCorrectionFromOdometry(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::POSE: {

      if (!sh_pose_s_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_pose_s_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* if (!isTimestampOk(measurement_stamped.stamp)) { */
      /*   return {}; */
      /* } */
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
      auto res = getCorrectionFromPoseStamped(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
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

      if (!sh_range_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_range_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */

      auto res = getCorrectionFromRange(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::RTK_GPS: {

      if (!sh_rtk_.hasMsg()) {
        ROS_ERROR_THROTTLE(1.0, " no rtk msgs so far");
        return {};
      }

      auto msg                  = sh_rtk_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   ROS_ERROR("[%s]: rtk msg delay not ok", ros::this_node::getName().c_str()); */
      /*   return {}; */
      /* } */

      auto res = getCorrectionFromRtk(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        ROS_ERROR_THROTTLE(1.0, "[%s]: could not get rtk correction", ros::this_node::getName().c_str());
        return {};
      }

      break;
    }

    case MessageType_t::POINT: {

      if (!sh_point_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_point_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
      auto res = getCorrectionFromPoint(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::VECTOR: {

      if (!sh_vector_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_vector_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
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
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
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

/*//{ callbackOdometry() */
template <int n_measurements>
void Correction<n_measurements>::callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromOdometry(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
}
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
          if (is_in_body_frame_) {
            std_msgs::Header header = msg->header;
            header.frame_id         = ch_->frames.ns_fcu;  // message in odometry is published in body frame
            auto res                = getVelInFrame(msg->twist.twist.linear, header, ns_frame_id_ + "_att_only");
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              return measurement;
            } else {
              ROS_WARN_THROTTLE(1.0, "[%s]: could not transform vel from odom", ros::this_node::getName().c_str());
              return {};
            }
          } else {
            measurement_t measurement;
            measurement(0) = msg->twist.twist.linear.x;
            measurement(1) = msg->twist.twist.linear.y;
            return measurement;
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
          if (is_in_body_frame_) {
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
          } else {
            measurement_t measurement;
            measurement(0) = msg->twist.twist.linear.z;
            return measurement;
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

        case StateId_t::POSITION: {
          measurement_t measurement;
          try {
            measurement(0) = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
            return measurement;
          }
          catch (...) {
            ROS_ERROR_THROTTLE(1.0, "[%s]: failed to obtain heading", getPrintName().c_str());
            return {};
          }
          break;
        }

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

/*//{ callbackPoseStamped() */
template <int n_measurements>
void Correction<n_measurements>::callbackPoseStamped(mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromPoseStamped(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromPoseStamped() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromPoseStamped(
    const geometry_msgs::PoseStampedConstPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->pose.position.x;
          measurement(1) = msg->pose.position.y;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
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
          measurement(0) = msg->pose.position.z;
          return measurement;
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          try {
            measurement(0) = mrs_lib::AttitudeConverter(msg->pose.orientation).getHeading();
            return measurement;
          }
          catch (...) {
            ROS_ERROR_THROTTLE(1.0, "[%s]: failed to obtain heading", getPrintName().c_str());
            return {};
          }
          break;
        }

        default: {
          ROS_ERROR_THROTTLE(1.0, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
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

/*//{ callbackRange() */
template <int n_measurements>
void Correction<n_measurements>::callbackRange(mrs_lib::SubscribeHandler<sensor_msgs::Range>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromRange(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromRange() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRange(const sensor_msgs::RangeConstPtr msg) {

  if (!range_enabled_) {
    ROS_INFO_THROTTLE(1.0, "[%s]: fusing range corrections is disabled", getPrintName().c_str());
    return {};
  }

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

/*//{ callbackRtk() */
template <int n_measurements>
void Correction<n_measurements>::callbackRtk(mrs_lib::SubscribeHandler<mrs_msgs::RtkGps>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromRtk(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromRtk() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRtk(const mrs_msgs::RtkGpsConstPtr msg) {

  geometry_msgs::PoseStamped rtk_pos;

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

  rtk_pos.pose.position.x -= ch_->world_origin.x;
  rtk_pos.pose.position.y -= ch_->world_origin.y;

  Correction::measurement_t measurement;

  // transform the RTK position from antenna to FCU
  auto res = transformRtkToFcu(rtk_pos);
  if (res) {
    rtk_pos.pose = res.value();
  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: transform to fcu failed", getPrintName().c_str());
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
          if (!got_avg_init_rtk_z_) {
            getAvgRtkInitZ(measurement(0));
            return {};
          }
          measurement(0) -= rtk_init_z_avg_;
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

/*//{ callbackPoint() */
template <int n_measurements>
void Correction<n_measurements>::callbackPoint(mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromPoint(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
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

/*//{ callbackVector() */
template <int n_measurements>
void Correction<n_measurements>::callbackVector(mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>& wrp) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromVector(wrp.getMsg());
  if (res) {
    applyCorrection(res.value(), wrp.getMsg()->header.stamp);
  }
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

/*//{ applyCorrection() */
template <int n_measurements>
void Correction<n_measurements>::applyCorrection(const measurement_t& meas, const ros::Time& stamp) {

  {
    std::scoped_lock lock(mtx_msg_time_);
    if (first_timestamp_) {
      prev_msg_time_   = stamp - ros::Duration(0.01);
      msg_time_        = stamp;
      healthy_time_    = ros::Time(0);
      first_timestamp_ = false;
    }

    prev_msg_time_ = msg_time_;
    msg_time_      = stamp;
    healthy_time_ += msg_time_ - prev_msg_time_;
  }

  MeasurementStamped meas_stamped;
  meas_stamped.value = meas;
  meas_stamped.stamp = stamp;
  publishCorrection(meas_stamped, ph_correction_raw_);
  if (process(meas_stamped.value)) {
    publishCorrection(meas_stamped, ph_correction_proc_);
    fun_apply_correction_(meas_stamped, getR(), getStateId());
  }
}
/*//}*/

/* //{ callbackToggleRange() */
template <int n_measurements>
bool Correction<n_measurements>::callbackToggleRange(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

  if (!is_initialized_) {
    return false;
  }

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
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getVelInFrame(const geometry_msgs::Vector3& vel_in,
                                                                                                            const std_msgs::Header&       source_header,
                                                                                                            const std::string             target_frame) {

  measurement_t measurement;

  geometry_msgs::Vector3Stamped vel;
  vel.header = source_header;
  vel.vector = vel_in;
  /* body.vector.x = vel.x; */
  /* body.vector.y = vel.y; */
  /* body.vector.z = vel.z; */

  geometry_msgs::Vector3Stamped transformed_vel;
  auto                          res = ch_->transformer->transformSingle(vel, target_frame);
  if (res) {
    transformed_vel = res.value();
    measurement(0)  = transformed_vel.vector.x;
    measurement(1)  = transformed_vel.vector.y;
    return measurement;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform of velocity from %s to %s failed.", getPrintName().c_str(), vel.header.frame_id.c_str(), target_frame.c_str());
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

/*//{ getAvgRtkInitZ() */
template <int n_measurements>
void Correction<n_measurements>::getAvgRtkInitZ(const double rtk_z) {

  if (!got_avg_init_rtk_z_) {

    double rtk_avg = rtk_init_z_avg_ / got_rtk_counter_;

    if (got_rtk_counter_ < 10 || (got_rtk_counter_ < 300 && std::fabs(rtk_z - rtk_avg) > 0.1)) {

      rtk_init_z_avg_ += rtk_z;
      got_rtk_counter_++;
      rtk_avg = rtk_init_z_avg_ / got_rtk_counter_;
      ROS_INFO("[%s]: RTK ASL altitude sample #%d: %.2f; avg: %.2f", getPrintName().c_str(), got_rtk_counter_, rtk_z, rtk_avg);
      return;

    } else {

      rtk_init_z_avg_     = rtk_avg;
      got_avg_init_rtk_z_ = true;
      ROS_INFO("[%s]: RTK ASL altitude avg: %f", getPrintName().c_str(), rtk_avg);
    }
  }
}
/*//}*/

/*//{ checkMsgDelay() */
template <int n_measurements>
void Correction<n_measurements>::checkMsgDelay(const ros::Time& msg_time) {

  const double delay = (ros::Time::now() - msg_time).toSec();
  if (delay > msg_delay_warn_limit_) {
    if (delay > msg_delay_limit_) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: message too delayed (%.4f s)", getPrintName().c_str(), delay);
      is_delay_ok_ = false;
    } else {
      ROS_WARN_THROTTLE(5.0, "[%s]: message delayed (%.4f s)", getPrintName().c_str(), delay);
      is_delay_ok_ = true;
    }
  } else {
    is_delay_ok_ = true;
  }
  publishDelay(delay);
}
/*//}*/

/*//{ isTimestampOk() */
template <int n_measurements>
bool Correction<n_measurements>::isTimestampOk() {

  if (first_timestamp_) {
    return true;
  }

  ros::Time msg_time, prev_msg_time;
  {
    std::scoped_lock lock(mtx_msg_time_);
    msg_time      = msg_time_;
    prev_msg_time = prev_msg_time_;
  }
  const double delta = msg_time.toSec() - prev_msg_time.toSec();

  if (msg_time.toSec() <= 0.0) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: current timestamp non-positive: %f", getPrintName().c_str(), msg_time.toSec());
    return false;
  }

  if (delta <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "[%s]: time delta non-positive: %f", getPrintName().c_str(), delta);
    return true;
  }

  if (delta < 0.001) {
    ROS_WARN_THROTTLE(1.0, "[%s]: time delta too small: %f", getPrintName().c_str(), delta);
    return true;
  }

  if (delta > time_since_last_msg_limit_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: time since last msg too long %f > %f", getPrintName().c_str(), delta, time_since_last_msg_limit_);
    return false;
  }

  return true;
}  // namespace mrs_uav_state_estimators
/*//}*/

/*//{ isMsgComing() */
template <int n_measurements>
bool Correction<n_measurements>::isMsgComing() {

  const ros::Time msg_time = mrs_lib::get_mutexed(mtx_msg_time_, msg_time_);
  const double    delta    = ros::Time::now().toSec() - msg_time.toSec();

  if (msg_time.toSec() <= 0.0) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: current timestamp non-positive: %f", getPrintName().c_str(), msg_time.toSec());
    return false;
  }

  if (delta > time_since_last_msg_limit_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: time since last msg too long %f > %f", getPrintName().c_str(), delta, time_since_last_msg_limit_);
    return false;
  }

  return true;
}  // namespace mrs_uav_state_estimators
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
  } else if (name == "tf_to_world") {
    return std::make_shared<ProcTfToWorld<n_measurements>>(nh, getNamespacedName(), name, ch_);
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

  bool ok_flag   = true;
  bool fuse_flag = true;

  for (auto proc_name :
       processor_names_) {  // need to access the estimators in the specific order from the config (e.g. median filter should go before saturation etc.)
    /* bool is_ok, should_fuse; */
    auto [ is_ok, should_fuse ] = processors_[proc_name]->process(measurement);
    ok_flag &= is_ok;
    fuse_flag &= should_fuse;
  }
  if (fuse_flag) {
    if (!ok_flag) {
      setR(default_R_ * R_coeff_);
      ROS_INFO_THROTTLE(1.0, "[%s]: set R to %.4f", getPrintName().c_str(), default_R_ * R_coeff_);
      return true;
    } else {
      setR(default_R_);
      return true;
    }
  }
  return false;
}
/*//}*/

/*//{ publishCorrection() */
template <int n_measurements>
void Correction<n_measurements>::publishCorrection(const MeasurementStamped&                                 measurement_stamped,
                                                   mrs_lib::PublisherHandler<mrs_msgs::EstimatorCorrection>& ph_corr) {

  if (!ch_->debug_topics.correction) {
    return;
  }

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

/*//{ publishDelay() */
template <int n_measurements>
void Correction<n_measurements>::publishDelay(const double delay) {

  if (!ch_->debug_topics.corr_delay) {
    return;
  }

  mrs_msgs::Float64Stamped msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = ns_frame_id_;
  msg.value           = delay;

  ph_delay_.publish(msg);
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_CORRECTION_H
