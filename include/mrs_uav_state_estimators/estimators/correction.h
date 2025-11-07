#pragma once
#ifndef ESTIMATORS_CORRECTION_H
#define ESTIMATORS_CORRECTION_H

/* includes //{ */

#include <Eigen/Dense>

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/service_server_handler.h>

#include <mrs_msgs/msg/rtk_gps.hpp>
#include <mrs_msgs/msg/estimator_correction.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <functional>

#include <mrs_uav_managers/estimation_manager/types.h>
#include <mrs_uav_managers/estimation_manager/support.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <mrs_uav_managers/estimation_manager/private_handlers.h>

#include <mrs_uav_state_estimators/processors/processor.h>
#include <mrs_uav_state_estimators/processors/proc_median_filter.h>
#include <mrs_uav_state_estimators/processors/proc_saturate.h>
#include <mrs_uav_state_estimators/processors/proc_excessive_tilt.h>
#include <mrs_uav_state_estimators/processors/proc_tf_to_world.h>
#include <mrs_uav_state_estimators/processors/proc_mag_declination.h>

//}

namespace mrs_uav_state_estimators
{

typedef enum
{
  LATERAL,
  ALTITUDE,
  HEADING,
  LATALT,
} EstimatorType_t;
const int n_EstimatorType_t = 4;

typedef enum
{
  UNKNOWN,
  ODOMETRY,
  POSE,
  POSECOV,
  RANGE,
  IMU,
  RTK_GPS,
  NAVSATFIX,
  MAG_HDG,
  MAG_FIELD,
  POINT,
  VECTOR,
  QUAT,
} MessageType_t;
const int n_MessageType_t = 10;

const std::map<std::string, MessageType_t> map_msg_type{{"nav_msgs/Odometry", MessageType_t::ODOMETRY},
                                                        {"geometry_msgs/PoseStamped", MessageType_t::POSE},
                                                        {"geometry_msgs/PoseWithCovarianceStamped", MessageType_t::POSECOV},
                                                        {"sensor_msgs/Range", MessageType_t::RANGE},
                                                        {"sensor_msgs/Imu", MessageType_t::IMU},
                                                        {"mrs_msgs/RtkGps", MessageType_t::RTK_GPS},
                                                        {"sensor_msgs/NavSatFix", MessageType_t::NAVSATFIX},
                                                        {"mrs_msgs/Float64Stamped", MessageType_t::MAG_HDG},
                                                        {"sensor_msgs/MagneticField", MessageType_t::MAG_FIELD},
                                                        {"geometry_msgs/PointStamped", MessageType_t::POINT},
                                                        {"geometry_msgs/Vector3Stamped", MessageType_t::VECTOR},
                                                        {"geometry_msgs/QuaternionStamped", MessageType_t::QUAT}};

template <int n_measurements>
class Correction {

  using CommonHandlers_t  = mrs_uav_managers::estimation_manager::CommonHandlers_t;
  using PrivateHandlers_t = mrs_uav_managers::estimation_manager::PrivateHandlers_t;
  using StateId_t         = mrs_uav_managers::estimation_manager::StateId_t;

protected:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

  struct MeasurementStamped
  {
    rclcpp::Time  stamp;
    measurement_t value;
  };

public:
  Correction(const rclcpp::Node::SharedPtr& node, const std::string& est_name, const std::string& name, const std::string& frame_id,
             const EstimatorType_t& est_type, const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph,
             std::function<double(int, int)> fun_get_state, std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction);

  std::string getName() const;
  std::string getNamespacedName() const;
  std::string getPrintName() const;

  double    getR();
  void      setR(const double R);
  StateId_t getStateId() const;

  bool             isHealthy();
  rclcpp::Time     healthy_time_;
  std::atomic_bool is_healthy_    = true;
  std::atomic_bool is_delay_ok_   = true;
  std::atomic_bool is_dt_ok_      = true;
  std::atomic_bool is_nan_free_   = true;
  std::atomic_bool got_first_msg_ = false;

  int counter_nan_ = 0;

  std::optional<MeasurementStamped> getRawCorrection();
  std::optional<MeasurementStamped> getProcessedCorrection();

  void resetProcessors();

private:
  std::atomic_bool is_initialized_ = false;

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odom_;
  void                                                callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  std::optional<measurement_t>                        getCorrectionFromOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped> sh_pose_s_;
  void                                                        callbackPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  std::optional<measurement_t>                                getCorrectionFromPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped> sh_pose_wcs_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::RtkGps> sh_rtk_;
  void                                              callbackRtk(const mrs_msgs::msg::RtkGps::ConstSharedPtr msg);
  std::optional<measurement_t>                      getCorrectionFromRtk(const mrs_msgs::msg::RtkGps::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_navsatfix_;
  void                                                    callbackNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  std::optional<measurement_t>                            getCorrectionFromNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  void   getAvgInitZ(const double z);
  bool   got_avg_init_z_ = false;
  double init_z_avg_     = 0.0;
  int    got_z_counter_  = 0;

  mrs_lib::SubscriberHandler<geometry_msgs::msg::PointStamped> sh_point_;
  void                                                         callbackPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  std::optional<measurement_t>                                 getCorrectionFromPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped> sh_vector_;
  std::optional<measurement_t>                                   getCorrectionFromVector(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
  void                                                           callbackVector(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_orientation_;  // for obtaining heading rate
  std::string                                                       orientation_topic_;

  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_quat_;
  measurement_t                                                     prev_hdg_measurement_;
  bool                                                              got_first_hdg_measurement_ = false;
  bool                                                              init_hdg_in_zero_          = false;
  double                                                            first_hdg_measurement_     = 0.0;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_imu_;
  std::optional<measurement_t>                      getCorrectionFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void                                              callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped> sh_mag_hdg_;
  std::optional<measurement_t>                              getCorrectionFromMagHeading(const mrs_msgs::msg::Float64Stamped::ConstSharedPtr msg);
  void                                                      callbackMagHeading(const mrs_msgs::msg::Float64Stamped::ConstSharedPtr msg);
  double                                                    mag_hdg_previous_;
  bool                                                      got_first_mag_hdg_;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_mag_field_;
  std::optional<measurement_t>                                getCorrectionFromMagField(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void                                                        callbackMagField(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);

  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>   sh_range_;
  std::optional<measurement_t>                          getCorrectionFromRange(const sensor_msgs::msg::Range::ConstSharedPtr msg);
  void                                                  callbackRange(const sensor_msgs::msg::Range::ConstSharedPtr msg);
  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool> ss_toggle_range_;
  bool callbackToggleRange(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool range_enabled_ = true;

  void applyCorrection(const measurement_t& meas, const rclcpp::Time& stamp);

  mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection> ph_correction_raw_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection> ph_correction_proc_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>      ph_delay_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>   ph_mag_field_untilted_;

  const std::string                  est_name_;
  const std::string                  name_;
  const std::string                  ns_frame_id_;
  const EstimatorType_t              est_type_;
  std::shared_ptr<CommonHandlers_t>  ch_;
  std::shared_ptr<PrivateHandlers_t> ph_;

  MessageType_t msg_type_;
  std::string   msg_topic_;
  double        msg_timeout_;

  double     R_;
  double     default_R_;
  double     R_coeff_;
  std::mutex mtx_R_;
  StateId_t  state_id_;
  bool       is_in_body_frame_ = true;
  double     gravity_norm_     = 9.8066;

  bool        transform_to_frame_enabled_ = false;
  std::string transform_to_frame_;
  std::string transform_from_frame_;

  std::optional<measurement_t>               getCorrectionFromQuat(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg);
  std::optional<measurement_t>               getZVelUntilted(const geometry_msgs::msg::Vector3& msg, const std_msgs::msg::Header& header);
  std::optional<measurement_t>               getVecInFrame(const geometry_msgs::msg::Vector3& vec_in, const std_msgs::msg::Header& source_header,
                                                           const std::string target_frame);
  std::optional<geometry_msgs::msg::Vector3> transformVecToFrame(const geometry_msgs::msg::Vector3& vec_in, const std_msgs::msg::Header& source_header,
                                                                 const std::string target_frame);
  std::optional<geometry_msgs::msg::Point>   getInFrame(const geometry_msgs::msg::Point& vec_in, const std_msgs::msg::Header& source_header,
                                                        const std::string target_frame);

  std::optional<geometry_msgs::msg::Pose> transformRtkToFcu(const geometry_msgs::msg::PoseStamped& pose_in) const;

  void   checkMsgDelay(const rclcpp::Time& msg_time);
  double msg_delay_limit_;
  double msg_delay_warn_limit_;

  double time_since_last_msg_limit_;

  std::shared_ptr<Processor<n_measurements>> createProcessorFromName(const std::string& name, const rclcpp::Node::SharedPtr& subnode);
  bool                                       process(measurement_t& measurement);

  bool             isTimestampOk();
  bool             isMsgComing();
  std::atomic_bool first_timestamp_ = true;
  rclcpp::Time     msg_time_;
  rclcpp::Time     prev_msg_time_;
  std::mutex       mtx_msg_time_;

  std::vector<std::string>                                                    processor_names_;
  std::unordered_map<std::string, std::shared_ptr<Processor<n_measurements>>> processors_;

  std::function<double(int, int)>                            fun_get_state_;
  std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction_;

  void publishCorrection(const MeasurementStamped& measurement_stamped, mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection>& ph_corr);
  void publishDelay(const double delay);

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;

  struct drs_params
  {
    double sensor_noise = 1.0;
  };

  drs_params drs_params_;
  std::mutex mutex_drs_params_;
};

/*//{ constructor */
template <int n_measurements>
Correction<n_measurements>::Correction(const rclcpp::Node::SharedPtr& node, const std::string& est_name, const std::string& name,
                                       const std::string& ns_frame_id, const EstimatorType_t& est_type, const std::shared_ptr<CommonHandlers_t>& ch,
                                       const std::shared_ptr<PrivateHandlers_t>& ph, std::function<double(int, int)> fun_get_state,
                                       std::function<void(MeasurementStamped, double, StateId_t)> fun_apply_correction)
    : node_(node),
      est_name_(est_name),
      name_(name),
      ns_frame_id_(ns_frame_id),
      est_type_(est_type),
      ch_(ch),
      ph_(ph),
      fun_get_state_(fun_get_state),
      fun_apply_correction_(fun_apply_correction) {

  clock_ = node->get_clock();

  // | --------------------- load parameters -------------------- |

  std::string msg_type_string;

  /* ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch->nodelet_name) + "/" + getNamespacedName() + "/"); */

  ph->param_loader->loadParam("message/type", msg_type_string);
  if (map_msg_type.find(msg_type_string) == map_msg_type.end()) {
    RCLCPP_ERROR(this->node_->get_logger(), "[%s]: wrong message type: %s of correction %s", getPrintName().c_str(), msg_type_string.c_str(),
                 getName().c_str());
    rclcpp::shutdown();
  }
  msg_type_ = map_msg_type.at(msg_type_string);

  ph->param_loader->loadParam("message/topic", msg_topic_);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;
  ph->param_loader->loadParam("message/limit/delay", msg_delay_limit_);
  msg_delay_warn_limit_ = msg_delay_limit_ / 2;  // maybe specify this as a param?
  ph->param_loader->loadParam("message/limit/time_since_last", time_since_last_msg_limit_);

  int state_id_tmp;
  ph->param_loader->loadParam("state_id", state_id_tmp);
  if (state_id_tmp < n_StateId_t) {
    state_id_ = static_cast<StateId_t>(state_id_tmp);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: wrong state id: %d of correction %s", getPrintName().c_str(), state_id_tmp, getName().c_str());
    rclcpp::shutdown();
  }

  ph->param_loader->loadParam("transform/enabled", transform_to_frame_enabled_);

  if (transform_to_frame_enabled_) {
    ph->param_loader->loadParam("transform/from_frame", transform_from_frame_);
    transform_from_frame_ = ch_->uav_name + "/" + transform_from_frame_;
    ph->param_loader->loadParam("transform/to_frame", transform_to_frame_);
    transform_to_frame_ = ch_->uav_name + "/" + transform_to_frame_;
  }

  if (state_id_ == StateId_t::VELOCITY) {
    ph->param_loader->loadParam("body_frame", is_in_body_frame_);
  }

  if (state_id_ == StateId_t::ACCELERATION) {
    ph->param_loader->loadParam("body_frame", is_in_body_frame_);
    ph->param_loader->loadParam("gravity_norm", gravity_norm_);
  }

  ph->param_loader->loadParam("noise", R_);
  ph->param_loader->loadParam("noise_unhealthy_coeff", R_coeff_);
  default_R_ = R_;

  // | --------------- processors initialization --------------- |
  ph->param_loader->loadParam("processors", processor_names_);

  for (auto proc_name : processor_names_) {
    processors_[proc_name] = createProcessorFromName(proc_name, node->create_sub_node(proc_name));
  }

  // | ------------- initialize dynamic reconfigure ------------- |

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);

  // TODO the parameter name needs fixing to reflect the ROS1 naming
  drs_params_.sensor_noise = R_;
  dynparam_mgr_->register_param(node_->get_sub_namespace() + "/noise", &drs_params_.sensor_noise, drs_params_.sensor_noise,
                                mrs_lib::DynparamMgr::range_t<double>(0.0, 100000.0));

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node               = node;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  switch (msg_type_) {
    case MessageType_t::ODOMETRY: {
      sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, msg_topic_, &Correction::callbackOdometry, this);
      break;
    }
    case MessageType_t::POSE: {
      sh_pose_s_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped>(shopts, msg_topic_, &Correction::callbackPoseStamped, this);
      break;
    }
    case MessageType_t::POSECOV: {
      // TODO implement
      /* sh_pose_wcs_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(shopts, msg_topic_); */
      break;
    }
    case MessageType_t::RANGE: {

      sh_range_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>(shopts, msg_topic_, &Correction::callbackRange, this);

      ss_toggle_range_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
          node_, std::string(node_->get_namespace()) + "/" + std::string(node_->get_name()) + "/" + getNamespacedName() + "/toggle_range",
          std::bind(&Correction::callbackToggleRange, this, std::placeholders::_1, std::placeholders::_2));

      break;
    }
    case MessageType_t::IMU: {
      sh_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, msg_topic_, &Correction::callbackImu, this);
      break;
    }
    case MessageType_t::RTK_GPS: {
      sh_rtk_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::RtkGps>(shopts, msg_topic_, &Correction::callbackRtk, this);
      break;
    }
    case MessageType_t::NAVSATFIX: {
      sh_navsatfix_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, msg_topic_, &Correction::callbackNavSatFix, this);
      break;
    }
    case MessageType_t::MAG_HDG: {
      sh_mag_hdg_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, msg_topic_, &Correction::callbackMagHeading, this);
      break;
    }
    case MessageType_t::MAG_FIELD: {
      sh_mag_field_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, msg_topic_, &Correction::callbackMagField, this);
      ph_mag_field_untilted_ =
          mrs_lib::PublisherHandler<geometry_msgs::msg::PointStamped>(node_, "~/" + est_name_ + "/correction/" + getName() + "/fcu_untilted");

      break;
    }
    case MessageType_t::POINT: {
      sh_point_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::PointStamped>(shopts, msg_topic_, &Correction::callbackPoint, this);
      break;
    }
    case MessageType_t::VECTOR: {
      sh_vector_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped>(shopts, msg_topic_, &Correction::callbackVector, this);
      break;
    }
    case MessageType_t::QUAT: {
      sh_quat_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, msg_topic_);
      break;
    }
    case MessageType_t::UNKNOWN: {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: UNKNOWN message type of correction", getPrintName().c_str());
      break;
    }
    default: {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: unhandled message type", getPrintName().c_str());
    }
  }

  // | ------ subscribe orientation for obtaingin hdg rate ------ |
  /* if (est_type_ == EstimatorType_t::HEADING && state_id_ == StateId_t::VELOCITY) { */
  /*   ph->param_loader->loadParam("message/orientation_topic", orientation_topic_); */
  /*   orientation_topic_ = "/" + ch_->uav_name + "/" + orientation_topic_; */
  /*   sh_orientation_    = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, orientation_topic_); */
  /* } */

  if (est_type_ == EstimatorType_t::HEADING) {
    ph->param_loader->loadParam("init_hdg_in_zero", init_hdg_in_zero_, false);
  }

  // | --------------- initialize publish handlers -------------- |

  if (ch_->debug_topics.correction) {
    ph_correction_raw_  = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection>(node_, "~/" + est_name_ + "/correction/" + getName() + "/raw");
    ph_correction_proc_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection>(node, "~/" + est_name_ + "/correction/" + getName() + "/proc");
  }

  if (ch_->debug_topics.corr_delay) {
    ph_delay_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/" + est_name_ + "/correction/" + getName() + "/delay");
  }

  // | --- check whether all parameters were loaded correctly --- |
  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
    exit(1);
  }

  healthy_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

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

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  default_R_ = drs_params.sensor_noise;

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
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: delay not ok", getPrintName().c_str());
  }

  if (!is_healthy_) {
    if (is_dt_ok_ && is_delay_ok_) {
      if (healthy_time_.seconds() > 10) {
        is_healthy_ = true;
      }
    } else {
      healthy_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
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
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "fusing range corrections is disabled");
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

    case MessageType_t::IMU: {

      if (!sh_imu_.hasMsg()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, " no imu msgs so far");
        return {};
      }

      auto msg                  = sh_imu_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   ROS_ERROR("[%s]: rtk msg delay not ok", ros::this_node::getName().c_str()); */
      /*   return {}; */
      /* } */

      auto res = getCorrectionFromImu(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not get imu correction");
        return {};
      }

      break;
    }

    case MessageType_t::RTK_GPS: {

      if (!sh_rtk_.hasMsg()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, " no rtk msgs so far");
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
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not get rtk correction");
        return {};
      }

      break;
    }

    case MessageType_t::NAVSATFIX: {

      if (!sh_navsatfix_.hasMsg()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "no navsatfix msgs so far");
        return {};
      }

      auto msg                  = sh_navsatfix_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   ROS_ERROR("[%s]: rtk msg delay not ok", ros::this_node::getName().c_str()); */
      /*   return {}; */
      /* } */

      auto res = getCorrectionFromNavSatFix(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "could not get navsatfix correction");
        return {};
      }

      break;
    }

    case MessageType_t::MAG_HDG: {

      if (!sh_mag_hdg_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_mag_hdg_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
      auto res = getCorrectionFromMagHeading(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
        return {};
      }
      break;
    }

    case MessageType_t::MAG_FIELD: {

      if (!sh_mag_field_.hasMsg()) {
        return {};
      }

      auto msg                  = sh_mag_field_.getMsg();
      measurement_stamped.stamp = msg->header.stamp;
      /* checkMsgDelay(measurement_stamped.stamp); */

      /* if (!is_delay_ok_) { */
      /*   return {}; */
      /* } */
      auto res = getCorrectionFromMagField(msg);
      if (res) {
        measurement_stamped.value = res.value();
      } else {
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
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: this type of correction is not implemented in getCorrectionFromMessage()",
                            getPrintName().c_str());
      is_healthy_ = false;
      return {};
    }
  }

  // check for nans
  is_nan_free_ = true;
  for (int i = 0; i < measurement_stamped.value.rows(); i++) {
    if (!std::isfinite(measurement_stamped.value(i))) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaN detected in correction. Total NaNs: %d", getPrintName().c_str(), ++counter_nan_);
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
void Correction<n_measurements>::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromOdometry(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromOdometry() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          if (transform_to_frame_enabled_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = transform_from_frame_;
            auto res                     = getInFrame(msg->pose.pose.position, header, transform_to_frame_);
            if (res) {
              measurement_t measurement;
              measurement(0) = res.value().x;
              measurement(1) = res.value().y;
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not transform vel from odom");
              return {};
            }

          } else {
            measurement(0) = msg->pose.pose.position.x;
            measurement(1) = msg->pose.pose.position.y;
          }
          return measurement;
          break;
        }

        case StateId_t::VELOCITY: {
          if (is_in_body_frame_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = ch_->frames.ns_fcu;  // message in odometry is published in body frame
            auto res                     = getVecInFrame(msg->twist.twist.linear, header, ns_frame_id_ + "_att_only");
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not transform vel from odom", getPrintName().c_str());
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
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
          if (transform_to_frame_enabled_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = transform_from_frame_;
            auto res                     = getInFrame(msg->pose.pose.position, header, transform_to_frame_);
            if (res) {
              measurement_t measurement;
              measurement(0) = res.value().z;
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not transform vel from odom");
              return {};
            }

          } else {
            measurement(0) = msg->pose.pose.position.z;
          }
          return measurement;
          break;
        }

        case StateId_t::VELOCITY: {
          if (is_in_body_frame_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = ch_->frames.ns_fcu;
            auto res                     = getZVelUntilted(msg->twist.twist.linear, header);
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t                               measurement;
          std::unique_ptr<mrs_lib::AttitudeConverter> attitude;

          if (transform_to_frame_enabled_) {

            auto res = ch_->transformer->getTransform(transform_from_frame_, transform_to_frame_, msg->header.stamp);
            if (!res) {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not find transform from '%s' to '%s'", transform_from_frame_.c_str(),
                                   transform_to_frame_.c_str());
              return {};
            }

            // R_to_from = R^to_from
            Eigen::Matrix3d R_to_from = mrs_lib::AttitudeConverter(res.value().transform.rotation);

            Eigen::Matrix3d R_odom = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

            // obtain heading from orientation
            attitude = std::make_unique<mrs_lib::AttitudeConverter>(R_to_from * R_odom);

          } else {
            attitude = std::make_unique<mrs_lib::AttitudeConverter>(msg->pose.pose.orientation);
          }

          try {
            // obtain heading from orientation
            measurement(StateId_t::POSITION) = attitude->getHeading();

            // subtract initial heading to start with zero heading
            if (init_hdg_in_zero_ && got_first_hdg_measurement_) {
              measurement(StateId_t::POSITION) = measurement(StateId_t::POSITION) - first_hdg_measurement_;
            }

            // unwrap heading wrt previous measurement
            if (got_first_hdg_measurement_) {
              measurement(StateId_t::POSITION) =
                  mrs_lib::geometry::radians::unwrap(measurement(StateId_t::POSITION), prev_hdg_measurement_(StateId_t::POSITION));
            } else {
              first_hdg_measurement_     = measurement(StateId_t::POSITION);
              got_first_hdg_measurement_ = true;
            }
            prev_hdg_measurement_ = measurement;
            return measurement;
          }
          catch (...) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: failed to obtain heading", getPrintName().c_str());
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          if (transform_to_frame_enabled_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = transform_from_frame_;
            auto res                     = getInFrame(msg->pose.pose.position, header, transform_to_frame_);
            if (res) {
              measurement_t measurement;
              measurement(0) = res.value().x;
              measurement(1) = res.value().y;
              measurement(2) = res.value().z;
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not transform vel from odom");
              return {};
            }

          } else {
            measurement(0) = msg->pose.pose.position.x;
            measurement(1) = msg->pose.pose.position.y;
            measurement(2) = msg->pose.pose.position.z;
          }
          return measurement;
          break;
        }

        case StateId_t::VELOCITY: {
          if (is_in_body_frame_) {
            std_msgs::msg::Header header = msg->header;
            header.frame_id              = ch_->frames.ns_fcu;  // message in odometry is published in body frame
            auto res                     = getVecInFrame(msg->twist.twist.linear, header, ns_frame_id_ + "_att_only");
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not transform vel from odom", getPrintName().c_str());
              return {};
            }
          } else {
            measurement_t measurement;
            measurement(0) = msg->twist.twist.linear.x;
            measurement(1) = msg->twist.twist.linear.y;
            measurement(2) = msg->twist.twist.linear.z;
            return measurement;
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackPoseStamped() */
template <int n_measurements>
void Correction<n_measurements>::callbackPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromPoseStamped(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromPoseStamped() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromPoseStamped(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
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
            // obtain heading from orientation
            measurement(StateId_t::POSITION) = mrs_lib::AttitudeConverter(msg->pose.orientation).getHeading();
            // unwrap heading wrt previous measurement
            if (got_first_hdg_measurement_) {
              measurement(StateId_t::POSITION) = mrs_lib::geometry::radians::unwrap(measurement(POSITION), prev_hdg_measurement_(POSITION));
            } else {
              got_first_hdg_measurement_ = true;
            }
            prev_hdg_measurement_ = measurement;
            return measurement;
          }
          catch (...) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: failed to obtain heading", getPrintName().c_str());
            return {};
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->pose.position.x;
          measurement(1) = msg->pose.position.y;
          measurement(2) = msg->pose.position.z;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoseStamped() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackRange() */
template <int n_measurements>
void Correction<n_measurements>::callbackRange(const sensor_msgs::msg::Range::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromRange(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromRange() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRange(
    const sensor_msgs::msg::Range::ConstSharedPtr msg) {

  if (!range_enabled_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: fusing range corrections is disabled", getPrintName().c_str());
    return {};
  }

  if (!std::isfinite(msg->range)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: received value: %f. Not using this correction.", getPrintName().c_str(), msg->range);
    return {};
  }

  const double eps = 1e-3;
  if (msg->range <= msg->min_range + eps || msg->range >= msg->max_range - eps) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "range measurement %.2f outside of its valid range (%.2f, %.2f)", msg->range, msg->min_range,
                         msg->max_range);
    return {};
  }

  geometry_msgs::msg::PoseStamped range_point;

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
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not transform range measurement to %s. Not using this correction.",
                          getPrintName().c_str(), ch_->frames.ns_fcu_untilted.c_str());
    return {};
  }
}
/*//}*/

/*//{ callbackImu() */
template <int n_measurements>
void Correction<n_measurements>::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromImu(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain correction from Imu msg", getPrintName().c_str());
  }
}
/*//}*/

/*//{ getCorrectionFromImu() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromImu(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::ACCELERATION: {
          if (is_in_body_frame_) {
            auto res = getVecInFrame(msg->linear_acceleration, msg->header, ns_frame_id_ + "_att_only");
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain IMU acceleration in frame: %s", getPrintName().c_str(),
                                   (ns_frame_id_ + "_att_only").c_str());
              return {};
            }
          } else {
            measurement_t measurement;
            measurement(0) = msg->linear_acceleration.x;
            measurement(1) = msg->linear_acceleration.y;
            return measurement;
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromImu() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::ACCELERATION: {
          if (is_in_body_frame_) {
            auto res = getZVelUntilted(msg->linear_acceleration, msg->header);
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              measurement(0) -= gravity_norm_;
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain IMU Z acceleration", getPrintName().c_str());
              return {};
            }
          } else {
            measurement_t measurement;
            measurement(0) = msg->linear_acceleration.z - gravity_norm_;
            return measurement;
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromImu() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          geometry_msgs::msg::Quaternion orientation;
          auto                           res = ch_->transformer->getTransform(ch_->frames.ns_fcu_untilted, ch_->frames.ns_fcu, clock_->now());
          if (res) {
            orientation = res.value().transform.rotation;
          } else {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain transform from %s to %s. Not using this correction.",
                                  getPrintName().c_str(), ch_->frames.ns_fcu_untilted.c_str(), ch_->frames.ns_fcu.c_str());
            return {};
          }

          measurement_t measurement;
          measurement(0) = mrs_lib::AttitudeConverter(orientation).getHeadingRate(msg->angular_velocity);
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromImu() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::ACCELERATION: {
          if (is_in_body_frame_) {
            auto res = getVecInFrame(msg->linear_acceleration, msg->header, ns_frame_id_ + "_att_only");
            if (res) {
              measurement_t measurement;
              measurement = res.value();
              return measurement;
            } else {
              RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain IMU acceleration in frame: %s", getPrintName().c_str(),
                                   (ns_frame_id_ + "_att_only").c_str());
              return {};
            }
          } else {
            measurement_t measurement;
            measurement(0) = msg->linear_acceleration.x;
            measurement(1) = msg->linear_acceleration.y;
            measurement(2) = msg->linear_acceleration.z;
            return measurement;
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromImu() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }
  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackRtk() */
template <int n_measurements>
void Correction<n_measurements>::callbackRtk(const mrs_msgs::msg::RtkGps::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromRtk(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromRtk() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromRtk(
    const mrs_msgs::msg::RtkGps::ConstSharedPtr msg) {

  geometry_msgs::msg::PoseStamped rtk_pos;

  if (!std::isfinite(msg->gps.latitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in RTK variable \"msg->latitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (!std::isfinite(msg->gps.longitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in RTK variable \"msg->longitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (!std::isfinite(msg->gps.altitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in RTK variable \"msg->altitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (msg->fix_type.fix_type != mrs_msgs::msg::RtkFixType::RTK_FIX) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] %s RTK FIX", getPrintName().c_str(), Support::waiting_for_string.c_str());
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
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: transform to fcu failed", getPrintName().c_str());
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromRtk() switch", getPrintName().c_str());
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
          if (!got_avg_init_z_) {
            getAvgInitZ(measurement(0));
            return {};
          }
          measurement(0) -= init_z_avg_;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromRtk() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    case EstimatorType_t::HEADING: {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: should not be possible to get into this branch of getCorrectionFromRtk() switch",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = rtk_pos.pose.position.x;
          measurement(1) = rtk_pos.pose.position.y;
          measurement(2) = rtk_pos.pose.position.z;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromRtk() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackNavSatFix() */
template <int n_measurements>
void Correction<n_measurements>::callbackNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromNavSatFix(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromNavSatFix() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromNavSatFix(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  geometry_msgs::msg::PointStamped navsatfix_pos;

  if (!std::isfinite(msg->latitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in NavSatFix variable \"msg->latitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (!std::isfinite(msg->longitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in NavSatFix variable \"msg->longitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (!std::isfinite(msg->altitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in NavSatFix variable \"msg->altitude\"!!!", getPrintName().c_str());
    return {};
  }

  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NavSatFix has no GNSS fix!!!", getPrintName().c_str());
    return {};
  }

  navsatfix_pos.header = msg->header;
  mrs_lib::UTM(msg->latitude, msg->longitude, &navsatfix_pos.point.x, &navsatfix_pos.point.y);
  navsatfix_pos.point.x -= ch_->world_origin.x;
  navsatfix_pos.point.y -= ch_->world_origin.y;
  navsatfix_pos.point.z = msg->altitude;

  Correction::measurement_t measurement;

  // TODO transform position from GNSS antenna to FCU

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = navsatfix_pos.point.x;
          measurement(1) = navsatfix_pos.point.y;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromNavSatFix() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = navsatfix_pos.point.z;
          if (!got_avg_init_z_) {
            getAvgInitZ(measurement(0));
            return {};
          }
          measurement(0) -= init_z_avg_;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromNavSatFix() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    case EstimatorType_t::HEADING: {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: should not be possible to get into this branch of getCorrectionFromNavSatFix() switch",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement(0) = navsatfix_pos.point.x;
          measurement(1) = navsatfix_pos.point.y;
          measurement(2) = navsatfix_pos.point.z;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromNavSatFix() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackMagHeading() */
template <int n_measurements>
void Correction<n_measurements>::callbackMagHeading(const mrs_msgs::msg::Float64Stamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromMagHeading(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain correction from Float64Stamped msg", getPrintName().c_str());
  }
}
/*//}*/

/*//{ getCorrectionFromMagHeading() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromMagHeading(
    const mrs_msgs::msg::Float64Stamped::ConstSharedPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::LATERAL in getCorrectionFromMagHeading() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::ALTITUDE in getCorrectionFromMagHeading() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      measurement_t measurement;

      const double mag_hdg = msg->value / 180 * M_PI;

      if (!got_first_mag_hdg_) {
        mag_hdg_previous_  = mag_hdg;
        got_first_mag_hdg_ = true;
      }

      measurement(0) = -mrs_lib::geometry::radians::unwrap(mag_hdg, mag_hdg_previous_) + M_PI / 2;  // may be weirdness of px4 heading (NED vs ENU or something)
      mag_hdg_previous_ = mag_hdg;
      return measurement;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::LATALT in getCorrectionFromMagHeading() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackMagField() */
template <int n_measurements>
void Correction<n_measurements>::callbackMagField(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromMagField(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain correction from sensor_msgs::msg::MagneticField msg",
                         getPrintName().c_str());
  }
}
/*//}*/

/*//{ getCorrectionFromMagField() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromMagField(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::LATERAL in getCorrectionFromMagField() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle altitude estimators
    case EstimatorType_t::ALTITUDE: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::ALTITUDE in getCorrectionFromMagField() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      /* Eigen::Matrix3d rot; */

      /* auto res = ch_->transformer->getTransform(ch_->frames.ns_fcu, ch_->frames.ns_fcu_untilted, msg->header.stamp); */
      /* if (res) { */
      /*   rot = Eigen::Matrix3d(mrs_lib::AttitudeConverter(res.value().transform.rotation)); */
      /* } else { */
      /*   ROS_ERROR_THROTTLE(1.0, "[%s]: Could not obtain transform from %s to %s. Not using this correction.", getPrintName().c_str(), */
      /*                      ch_->frames.ns_fcu_untilted.c_str(), ch_->frames.ns_fcu.c_str()); */
      /*   return {}; */
      /* } */

      geometry_msgs::msg::Vector3 mag_vec;
      mag_vec.x = msg->magnetic_field.x;
      mag_vec.y = msg->magnetic_field.y;
      mag_vec.z = msg->magnetic_field.z;

      if (transform_to_frame_enabled_) {
        std_msgs::msg::Header header = msg->header;
        header.frame_id              = transform_from_frame_;
        auto res                     = transformVecToFrame(mag_vec, header, transform_to_frame_);
        if (res) {
          mag_vec.x = res.value().x;
          mag_vec.y = res.value().y;
          mag_vec.z = res.value().z;
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not transform mag field vector", getPrintName().c_str());
        }
      }

      geometry_msgs::msg::PointStamped mag_vec_msg;
      mag_vec_msg.header.stamp    = msg->header.stamp;
      mag_vec_msg.header.frame_id = transform_to_frame_;
      mag_vec_msg.point.x         = mag_vec.x;
      mag_vec_msg.point.y         = mag_vec.y;
      mag_vec_msg.point.z         = mag_vec.z;
      ph_mag_field_untilted_.publish(mag_vec_msg);
      const double mag_hdg = atan2(mag_vec.y, mag_vec.x);
      /* const Eigen::Vector3d mag_vec(mag_vec_pt.x, mag_vec_pt.y, mag_vec_pt.z); */
      /* const Eigen::Vector3d proj_mag_field = rot * mag_vec; */
      /* mrs_msgs::msg::Float64Stamped hdg_stamped; */
      /* hdg_stamped.header = msg->header; */
      /* hdg_stamped.value = atan2(proj_mag_field.y(), proj_mag_field.x()); */
      /* const double mag_hdg = atan2(proj_mag_field.y(), proj_mag_field.x()); */

      measurement_t measurement;

      /* const double mag_hdg = msg->value / 180 * M_PI; */

      if (!got_first_mag_hdg_) {
        mag_hdg_previous_  = mag_hdg;
        got_first_mag_hdg_ = true;
      }

      measurement(0)    = -mrs_lib::geometry::radians::unwrap(mag_hdg, mag_hdg_previous_);  // may be weirdness of px4 heading (NED vs ENU or something)
      mag_hdg_previous_ = mag_hdg;
      return measurement;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::LATALT in getCorrectionFromMagField() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackPoint() */
template <int n_measurements>
void Correction<n_measurements>::callbackPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromPoint(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  }
}
/*//}*/

/*//{ getCorrectionFromPoint() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromPoint(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {

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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoint() switch", getPrintName().c_str());
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoint() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: EstimatorType_t::Heading in getCorrectionFromPoint() not implemented",
                            getPrintName().c_str());
      return {};
      break;
    }

    // handle latalt estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::POSITION: {
          measurement_t measurement;
          measurement(0) = msg->point.x;
          measurement(1) = msg->point.y;
          measurement(2) = msg->point.z;
          return measurement;
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoint() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    default: {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromPoint() switch", getPrintName().c_str());
      return {};
    }
  }


  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ callbackVector() */
template <int n_measurements>
void Correction<n_measurements>::callbackVector(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  auto res = getCorrectionFromVector(msg);
  if (res) {
    applyCorrection(res.value(), msg->header.stamp);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain correction from Vector3Stamped msg", getPrintName().c_str());
  }
}
/*//}*/

/*//{ getCorrectionFromVector() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromVector(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {

  switch (est_type_) {

    // handle lateral estimators
    case EstimatorType_t::LATERAL: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          auto res = getVecInFrame(msg->vector, msg->header, ns_frame_id_ + "_att_only");
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromVector() switch", getPrintName().c_str());
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
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain untilted Z velocity", getPrintName().c_str());
            return {};
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromVector() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle heading estimators
    case EstimatorType_t::HEADING: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          try {
            if (!sh_orientation_.hasMsg()) {
              RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s orientation on topic: %s", getPrintName().c_str(),
                                   Support::waiting_for_string.c_str(), orientation_topic_.c_str());
              return {};
            }
            measurement_t measurement;
            measurement(0) = mrs_lib::AttitudeConverter(sh_orientation_.getMsg()->quaternion).getHeadingRate(msg->vector);
            return measurement;
          }
          catch (...) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Exception caught during getting heading rate (getCorrectionFromVector())",
                                  getPrintName().c_str());
            return {};
          }
          break;
        }

        default: {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromVector() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }

    // handle lateral estimators
    case EstimatorType_t::LATALT: {

      switch (state_id_) {

        case StateId_t::VELOCITY: {
          auto res = getVecInFrame(msg->vector, msg->header, ns_frame_id_ + "_att_only");
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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromVector() switch", getPrintName().c_str());
          return {};
        }
      }
      break;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ getCorrectionFromQuat() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getCorrectionFromQuat(
    [[maybe_unused]] const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg) {

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
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: unhandled case in getCorrectionFromOdometry() switch", getPrintName().c_str());
      return {};
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "[%s]: FIXME: should not be possible to get into this part of code", getPrintName().c_str());
  return {};
}
/*//}*/

/*//{ applyCorrection() */
template <int n_measurements>
void Correction<n_measurements>::applyCorrection(const measurement_t& meas, const rclcpp::Time& stamp) {

  {
    std::scoped_lock lock(mtx_msg_time_);
    if (first_timestamp_) {
      prev_msg_time_   = stamp - rclcpp::Duration(std::chrono::duration<double>(0.01));
      msg_time_        = stamp;
      healthy_time_    = rclcpp::Time(0, 0, clock_->get_clock_type());
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
bool Correction<n_measurements>::callbackToggleRange(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                                     const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!range_enabled_ && request->data) {
    processors_["saturate"]->toggle(true);
  }

  range_enabled_ = request->data;

  // after enabling range we want to start correcting the altitude slowly

  response->success = true;
  response->message = (range_enabled_ ? "Range enabled" : "Range disabled");

  if (range_enabled_) {

    RCLCPP_INFO(node_->get_logger(), "[%s]: Range enabled.", getPrintName().c_str());

  } else {

    RCLCPP_INFO(node_->get_logger(), "[%s]: Range disabled", getPrintName().c_str());
  }

  return true;
}

//}

/*//{ getZVelUntilted() */
template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getZVelUntilted(const geometry_msgs::msg::Vector3& msg,
                                                                                                              const std_msgs::msg::Header&       header) {

  // untilt the desired vector
  geometry_msgs::msg::PointStamped vel;
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
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Transform from %s to %s failed", getPrintName().c_str(), vel.header.frame_id.c_str(),
                         ch_->frames.ns_fcu_untilted.c_str());
    return {};
  }
}

/*//}*/

/*//{ transformVecToFrame() */
template <int n_measurements>
std::optional<geometry_msgs::msg::Vector3> Correction<n_measurements>::transformVecToFrame(const geometry_msgs::msg::Vector3& vec_in,
                                                                                           const std_msgs::msg::Header&       source_header,
                                                                                           const std::string                  target_frame) {

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header = source_header;
  vec.vector = vec_in;

  auto res = ch_->transformer->transformSingle(vec, target_frame);
  if (res) {
    return res.value().vector;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Transform of vector from %s to %s failed.", getPrintName().c_str(),
                         vec.header.frame_id.c_str(), target_frame.c_str());
    return {};
  }
}

template <int n_measurements>
std::optional<typename Correction<n_measurements>::measurement_t> Correction<n_measurements>::getVecInFrame(const geometry_msgs::msg::Vector3& vec_in,
                                                                                                            const std_msgs::msg::Header&       source_header,
                                                                                                            const std::string                  target_frame) {

  measurement_t measurement;

  auto res = transformVecToFrame(vec_in, source_header, target_frame);
  if (res) {
    measurement(0) = res.value().x;
    measurement(1) = res.value().y;
    if (n_measurements == 3) {
      measurement(2) = res.value().z;
    }
    return measurement;
  } else {
    return {};
  }
}
/*//}*/

/*//{ getInFrame() */
template <int n_measurements>
std::optional<geometry_msgs::msg::Point> Correction<n_measurements>::getInFrame(const geometry_msgs::msg::Point& pt_in,
                                                                                const std_msgs::msg::Header& source_header, const std::string target_frame) {

  geometry_msgs::msg::PointStamped pt;
  pt.header = source_header;
  pt.point  = pt_in;

  geometry_msgs::msg::PointStamped transformed_pt;
  auto                             res = ch_->transformer->transformSingle(pt, target_frame);
  if (res) {
    transformed_pt = res.value();
    geometry_msgs::msg::Point pt_out;
    pt_out = transformed_pt.point;
    return pt_out;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Transform of point from %s to %s failed.", getPrintName().c_str(),
                         pt.header.frame_id.c_str(), target_frame.c_str());
    return {};
  }
}
/*//}*/

/*//{ transformRtkToFcu() */
template <int n_measurements>
std::optional<geometry_msgs::msg::Pose> Correction<n_measurements>::transformRtkToFcu(const geometry_msgs::msg::PoseStamped& pose_in) const {

  geometry_msgs::msg::PoseStamped pose_tmp = pose_in;

  // inject current orientation into rtk pose
  auto res1 = ch_->transformer->getTransform(ch_->frames.ns_fcu_untilted, ch_->frames.ns_fcu, clock_->now());
  if (res1) {
    pose_tmp.pose.orientation = res1.value().transform.rotation;
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not obtain transform from %s to %s. Not using this correction.",
                          getPrintName().c_str(), ch_->frames.ns_fcu_untilted.c_str(), ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform                  tf_utm_to_antenna = Support::tf2FromPose(pose_tmp.pose);
  geometry_msgs::msg::PoseStamped utm_in_antenna;
  utm_in_antenna.pose            = Support::poseFromTf2(tf_utm_to_antenna.inverse());
  utm_in_antenna.header.stamp    = pose_in.header.stamp;
  utm_in_antenna.header.frame_id = ch_->frames.ns_rtk_antenna;

  // transform to fcu
  geometry_msgs::msg::PoseStamped utm_in_fcu;
  utm_in_fcu.header.frame_id = ch_->frames.ns_fcu;
  utm_in_fcu.header.stamp    = pose_in.header.stamp;
  auto res2                  = ch_->transformer->transformSingle(utm_in_antenna, ch_->frames.ns_fcu);

  if (res2) {
    utm_in_fcu = res2.value();
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Could not transform pose to %s. Not using this correction.", getPrintName().c_str(),
                          ch_->frames.ns_fcu.c_str());
    return {};
  }

  // invert tf
  tf2::Transform           tf_fcu_to_utm = Support::tf2FromPose(utm_in_fcu.pose);
  geometry_msgs::msg::Pose fcu_in_utm    = Support::poseFromTf2(tf_fcu_to_utm.inverse());

  return fcu_in_utm;
}
/*//}*/

/*//{ getAvgInitZ() */
template <int n_measurements>
void Correction<n_measurements>::getAvgInitZ(const double z) {

  if (!got_avg_init_z_) {

    double z_avg = init_z_avg_ / got_z_counter_;

    if (got_z_counter_ < 10 || (got_z_counter_ < 300 && std::fabs(z - z_avg) > 0.1)) {

      init_z_avg_ += z;
      got_z_counter_++;
      z_avg = init_z_avg_ / got_z_counter_;
      RCLCPP_INFO(node_->get_logger(), "[%s]: AMSL altitude sample #%d: %.2f; avg: %.2f", getPrintName().c_str(), got_z_counter_, z, z_avg);
      return;

    } else {

      init_z_avg_     = z_avg;
      got_avg_init_z_ = true;
      RCLCPP_INFO(node_->get_logger(), "[%s]: AMSL altitude avg: %f", getPrintName().c_str(), z_avg);
    }
  }
}
/*//}*/

/*//{ checkMsgDelay() */
template <int n_measurements>
void Correction<n_measurements>::checkMsgDelay(const rclcpp::Time& msg_time) {

  const double delay = (clock_->now() - msg_time).seconds();
  if (delay > msg_delay_warn_limit_) {
    if (delay > msg_delay_limit_) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: message too delayed (%.4f s)", getPrintName().c_str(), delay);
      is_delay_ok_ = false;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "[%s]: message delayed (%.4f s)", getPrintName().c_str(), delay);
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

  rclcpp::Time msg_time, prev_msg_time;
  {
    std::scoped_lock lock(mtx_msg_time_);
    msg_time      = msg_time_;
    prev_msg_time = prev_msg_time_;
  }
  const double delta = msg_time.seconds() - prev_msg_time.seconds();

  if (msg_time.seconds() <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: current timestamp non-positive: %f", getPrintName().c_str(), msg_time.seconds());
    return false;
  }

  if (delta <= 0.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: time delta non-positive: %f", getPrintName().c_str(), delta);
    return true;
  }

  if (delta < 0.001) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: time delta too small: %f", getPrintName().c_str(), delta);
    return true;
  }

  if (delta > time_since_last_msg_limit_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: time since last msg too long %f > %f", getPrintName().c_str(), delta,
                          time_since_last_msg_limit_);
    return false;
  }

  return true;
}  // namespace mrs_uav_state_estimators
/*//}*/

/*//{ isMsgComing() */
template <int n_measurements>
bool Correction<n_measurements>::isMsgComing() {

  if (first_timestamp_) {
    return true;
  }

  const rclcpp::Time msg_time = mrs_lib::get_mutexed(mtx_msg_time_, msg_time_);
  const double       delta    = clock_->now().seconds() - msg_time.seconds();

  if (msg_time.seconds() <= 0.0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: current timestamp non-positive: %f", getPrintName().c_str(), msg_time.seconds());
    return false;
  }

  if (delta > time_since_last_msg_limit_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: time since last msg too long %f > %f", getPrintName().c_str(), delta,
                          time_since_last_msg_limit_);
    return false;
  }

  return true;
}  // namespace mrs_uav_state_estimators
/*//}*/

/*//{ createProcessorFromName() */
template <int n_measurements>
std::shared_ptr<Processor<n_measurements>> Correction<n_measurements>::createProcessorFromName(const std::string&             name,
                                                                                               const rclcpp::Node::SharedPtr& subnode) {

  auto proc_ph            = std::make_shared<PrivateHandlers_t>();
  proc_ph->loadConfigFile = ph_->loadConfigFile;
  proc_ph->param_loader   = std::make_unique<mrs_lib::ParamLoader>(subnode);
  proc_ph->param_loader->copyYamls(*ph_->param_loader);
  proc_ph->param_loader->setPrefix(ph_->param_loader->getPrefix());

  if (name == "median_filter") {
    return std::make_shared<ProcMedianFilter<n_measurements>>(subnode, getNamespacedName(), name, ch_, proc_ph);
  } else if (name == "saturate") {
    return std::make_shared<ProcSaturate<n_measurements>>(subnode, getNamespacedName(), name, ch_, proc_ph, state_id_, fun_get_state_);
  } else if (name == "excessive_tilt") {
    return std::make_shared<ProcExcessiveTilt<n_measurements>>(subnode, getNamespacedName(), name, ch_, proc_ph);
  } else if (name == "tf_to_world") {
    return std::make_shared<ProcTfToWorld<n_measurements>>(subnode, getNamespacedName(), name, ch_, proc_ph);
  } else if (name == "mag_declination") {
    return std::make_shared<ProcMagDeclination<n_measurements>>(subnode, getNamespacedName(), name, ch_, proc_ph);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: requested invalid processor %s", getPrintName().c_str(), name.c_str());
    rclcpp::shutdown();
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
       processor_names_) {  // need to access the processors in the specific order from the config (e.g. median filter should go before saturation etc.)
    /* bool is_ok, should_fuse; */
    auto [is_ok, should_fuse] = processors_[proc_name]->process(measurement);
    ok_flag &= is_ok;
    fuse_flag &= should_fuse;
  }
  if (fuse_flag) {
    if (!ok_flag) {
      setR(default_R_ * R_coeff_);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: set R to %.4f", getPrintName().c_str(), default_R_ * R_coeff_);
      return true;
    } else {
      setR(default_R_);
      return true;
    }
  }
  return false;
}
/*//}*/

/*//{ resetProcessors() */
template <int n_measurements>
void Correction<n_measurements>::resetProcessors() {

  for (auto proc_name :
       processor_names_) {  // need to access the processors in the specific order from the config (e.g. median filter should go before saturation etc.)
    /* bool is_ok, should_fuse; */
    processors_[proc_name]->reset();
  }
}
/*//}*/

/*//{ publishCorrection() */
template <int n_measurements>
void Correction<n_measurements>::publishCorrection(const MeasurementStamped&                                      measurement_stamped,
                                                   mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorCorrection>& ph_corr) {

  if (!ch_->debug_topics.correction) {
    return;
  }

  mrs_msgs::msg::EstimatorCorrection msg;
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

  mrs_msgs::msg::Float64Stamped msg;
  msg.header.stamp    = clock_->now();
  msg.header.frame_id = ns_frame_id_;
  msg.value           = delay;

  ph_delay_.publish(msg);
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_CORRECTION_H
