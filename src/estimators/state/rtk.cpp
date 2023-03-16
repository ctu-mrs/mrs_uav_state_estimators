/* includes //{ */

#include "estimators/state/rtk.h"

//}

namespace mrs_uav_state_estimators
{

namespace rtk
{

/* initialize() //{*/
void Rtk::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;
  nh_ = nh;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, getPrintName());

  Support::loadParamFile(ros::package::getPath(package_name_) + "/config/estimators/" + getName() + "/" + getName() + ".yaml", nh_.getNamespace());
  param_loader.setPrefix(getName() + "/");

  std::string topic_orientation;
  param_loader.loadParam("topics/orientation", topic_orientation);
  topic_orientation_ = "/" + ch_->uav_name + "/" + topic_orientation;
  std::string topic_angular_velocity;
  param_loader.loadParam("topics/angular_velocity", topic_angular_velocity);
  topic_angular_velocity_ = "/" + ch_->uav_name + "/" + topic_angular_velocity;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------------ timers initialization ----------------- |
  timer_update_             = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &Rtk::timerUpdate, this, false, false);  // not running after init
  timer_check_health_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &Rtk::timerCheckHealth, this);
  timer_pub_attitude_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &Rtk::timerPubAttitude, this);

  // | --------------- subscribers initialization --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_orient_  = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, topic_orientation_);
  sh_hw_api_ang_vel_ = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, topic_angular_velocity_);

  // | ---------------- publishers initialization --------------- |
  ph_uav_state_        = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, Support::toSnakeCase(getName()) + "/uav_state", 1);
  ph_odom_             = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 1);
  ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/pose_covariance", 1);
  ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/twist_covariance", 1);
  ph_innovation_       = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/innovation", 1);
  ph_diagnostics_      = mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics>(nh, Support::toSnakeCase(getName()) + "/diagnostics", 1);
  ph_attitude_         = mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped>(nh, Support::toSnakeCase(getName()) + "/attitude", 1);

  // | ---------------- estimators initialization --------------- |
  std::vector<double> max_altitudes;

  est_lat_rtk_ = std::make_unique<LatGeneric>(est_lat_name_, frame_id_, getName());
  est_lat_rtk_->initialize(nh, ch_);
  max_altitudes.push_back(est_lat_rtk_->getMaxFlightAltitudeAgl());

  est_alt_rtk_ = std::make_unique<AltGeneric>(est_alt_name_, frame_id_, getName());
  est_alt_rtk_->initialize(nh, ch_);
  max_altitudes.push_back(est_alt_rtk_->getMaxFlightAltitudeAgl());

  est_hdg_hw_api_ = std::make_unique<HdgPassthrough>(est_hdg_name_, frame_id_, getName());
  est_hdg_hw_api_->initialize(nh, ch_);
  max_altitudes.push_back(est_hdg_hw_api_->getMaxFlightAltitudeAgl());

  max_flight_altitude_agl_ = *std::min_element(max_altitudes.begin(), max_altitudes.end());

  // | ------------------ initialize published messages ------------------ |
  uav_state_.header.frame_id = ns_frame_id_;
  uav_state_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_.estimator_horizontal = est_lat_name_;
  uav_state_.estimator_vertical   = est_alt_name_;
  uav_state_.estimator_heading    = est_hdg_name_;

  innovation_.header.frame_id         = ns_frame_id_;
  innovation_.child_frame_id          = ch_->frames.ns_fcu;
  innovation_.pose.pose.orientation.w = 1.0;

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool Rtk::start(void) {


  if (isInState(READY_STATE)) {

    bool est_lat_rtk_start_successful, est_alt_rtk_start_successful, est_hdg_hw_api_start_successful;

    if (est_hdg_hw_api_->isStarted() || est_hdg_hw_api_->isRunning()) {
      est_hdg_hw_api_start_successful = true;
      timer_pub_attitude_.start();
    } else {
      est_hdg_hw_api_start_successful = est_hdg_hw_api_->start();
    }

    if (est_lat_rtk_->isStarted() || est_lat_rtk_->isRunning()) {
      est_lat_rtk_start_successful = true;
    } else {
      est_lat_rtk_start_successful = est_lat_rtk_->start();
    }


    if (est_alt_rtk_->isStarted() || est_alt_rtk_->isRunning()) {
      est_alt_rtk_start_successful = true;
    } else {
      est_alt_rtk_start_successful = est_alt_rtk_->start();
    }

    if (est_lat_rtk_start_successful && est_alt_rtk_start_successful && est_hdg_hw_api_start_successful) {
      timer_update_.start();
      changeState(STARTED_STATE);
      return true;
    }

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    ros::Duration(1.0).sleep();
  }
  return false;

  ROS_ERROR("[%s]: Failed to start", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ pause() */
bool Rtk::pause(void) {

  if (isInState(RUNNING_STATE)) {
    est_lat_rtk_->pause();
    est_alt_rtk_->pause();
    changeState(STOPPED_STATE);
    return true;
  }
  return false;
}
/*//}*/

/*//{ reset() */
bool Rtk::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  est_lat_rtk_->pause();
  est_alt_rtk_->pause();
  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void Rtk::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  if (!sh_hw_api_orient_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!sh_hw_api_ang_vel_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received angular velocity on topic %s yet", getPrintName().c_str(), sh_hw_api_ang_vel_.topicName().c_str());
    return;
  }
  const ros::Time time_now = ros::Time::now();

  {
    std::scoped_lock lock(mtx_uav_state_);

    uav_state_.header.stamp = time_now;

    uav_state_.pose.orientation = sh_hw_api_orient_.getMsg()->quaternion;

    uav_state_.velocity.angular = sh_hw_api_ang_vel_.getMsg()->vector;

    /* uav_state_.pose.position.x = est_lat_rtk_->getState(POSITION, AXIS_X) - utm_origin_x_; */
    /* uav_state_.pose.position.y = est_lat_rtk_->getState(POSITION, AXIS_Y) - utm_origin_y_; */
    uav_state_.pose.position.x = est_lat_rtk_->getState(POSITION, AXIS_X);
    uav_state_.pose.position.y = est_lat_rtk_->getState(POSITION, AXIS_Y);
    uav_state_.pose.position.z = est_alt_rtk_->getState(POSITION) - rtk_avg_init_z_;

    uav_state_.velocity.linear.x = est_lat_rtk_->getState(VELOCITY, AXIS_X);  // in global frame
    uav_state_.velocity.linear.y = est_lat_rtk_->getState(VELOCITY, AXIS_Y);  // in global frame
    uav_state_.velocity.linear.z = est_alt_rtk_->getState(VELOCITY);          // in global frame

    uav_state_.acceleration.linear.x = est_lat_rtk_->getState(ACCELERATION, AXIS_X);  // in global frame
    uav_state_.acceleration.linear.y = est_lat_rtk_->getState(ACCELERATION, AXIS_Y);  // in global frame
    uav_state_.acceleration.linear.z = est_alt_rtk_->getState(ACCELERATION);          // in global frame
  }

  {
    std::scoped_lock lock(mtx_uav_state_, mtx_odom_);
    odom_ = Support::uavStateToOdom(uav_state_, ch_->transformer);
  }

  {
    std::scoped_lock lock(mtx_innovation_);

    innovation_.header.stamp = time_now;

    innovation_.pose.pose.position.x = est_lat_rtk_->getInnovation(POSITION, AXIS_X);
    innovation_.pose.pose.position.y = est_lat_rtk_->getInnovation(POSITION, AXIS_Y);
    innovation_.pose.pose.position.z = est_alt_rtk_->getInnovation(POSITION);
  }

  {
    std::scoped_lock lock(mtx_covariance_);

    pose_covariance_.header.stamp  = time_now;
    twist_covariance_.header.stamp = time_now;

    const int n_states = 6;  // TODO(petrlmat) this should be defined somewhere else
    pose_covariance_.values.resize(n_states * n_states);
    pose_covariance_.values.at(n_states * AXIS_X + AXIS_X) = est_lat_rtk_->getCovariance(POSITION, AXIS_X);
    pose_covariance_.values.at(n_states * AXIS_Y + AXIS_Y) = est_lat_rtk_->getCovariance(POSITION, AXIS_Y);
    pose_covariance_.values.at(n_states * AXIS_Z + AXIS_Z) = est_alt_rtk_->getCovariance(POSITION);

    twist_covariance_.values.resize(n_states * n_states);
    twist_covariance_.values.at(n_states * AXIS_X + AXIS_X) = est_lat_rtk_->getCovariance(VELOCITY, AXIS_X);
    twist_covariance_.values.at(n_states * AXIS_Y + AXIS_Y) = est_lat_rtk_->getCovariance(VELOCITY, AXIS_Y);
    twist_covariance_.values.at(n_states * AXIS_Z + AXIS_Z) = est_alt_rtk_->getCovariance(VELOCITY);
  }

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void Rtk::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (sh_hw_api_orient_.hasMsg() && sh_hw_api_ang_vel_.hasMsg()) {
      if (est_lat_rtk_->isReady() && est_alt_rtk_->isReady()) {
        if (got_rtk_avg_init_z_) {
          changeState(READY_STATE);
        } else {
          getAvgRtkInitZ();
        }
        ROS_INFO("[%s]: Estimator is ready to start", getPrintName().c_str());
      } else {
        ROS_INFO("[%s]: Waiting for subestimators to be ready", getPrintName().c_str());
        return;
      }
    } else {
      ROS_INFO("[%s]: Waiting for msg on topic %s", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
      return;
    }
  }


  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getPrintName().c_str());

    if (est_lat_rtk_->isRunning() && est_alt_rtk_->isRunning()) {
      ROS_INFO("[%s]: Subestimators converged", getPrintName().c_str());
      changeState(RUNNING_STATE);
    } else {
      return;
    }
  }
}
/*//}*/

/* timerPubAttitude() //{*/
void Rtk::timerPubAttitude(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (!sh_hw_api_orient_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
    return;
  }

  const ros::Time time_now = ros::Time::now();

  geometry_msgs::QuaternionStamped att;
  att.header.stamp    = time_now;
  att.header.frame_id = ns_frame_id_ + "_att_only";
  att.quaternion      = sh_hw_api_orient_.getMsg()->quaternion;

  ph_attitude_.publish(att);
}
/*//}*/

/*//{ isConverged() */
bool Rtk::isConverged() {

  // TODO(petrlmat): check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ getUavState() */
mrs_msgs::UavState Rtk::getUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  return uav_state_;
}
/*//}*/

/*//{ getInnovation() */
nav_msgs::Odometry Rtk::getInnovation() const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_;
}
/*//}*/

/*//{ getPoseCovariance() */
std::vector<double> Rtk::getPoseCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return pose_covariance_.values;
}
/*//}*/

/*//{ getTwistCovariance() */
std::vector<double> Rtk::getTwistCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return twist_covariance_.values;
}
/*//}*/

/*//{ setUavState() */
bool Rtk::setUavState(const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ getAvgInitZ() */
void Rtk::getAvgRtkInitZ() {

  double rtk_z = est_alt_rtk_->getState(POSITION);

  if (!got_rtk_avg_init_z_) {

    double rtk_avg = rtk_avg_init_z_ / got_rtk_counter_;

    if (got_rtk_counter_ < 10 || (got_rtk_counter_ < 300 && std::fabs(rtk_z - rtk_avg) > 0.1)) {

      // sometimes RTK publishes 0.0 after initialization
      if (rtk_z == 0.0) {
        ROS_WARN_THROTTLE(1.0, "[%s]: Waiting for RTK to publish other altitude than 0.0", getPrintName().c_str());
        return;
      }

      rtk_avg_init_z_ += rtk_z;
      got_rtk_counter_++;
      rtk_avg = rtk_avg_init_z_ / got_rtk_counter_;
      ROS_INFO("[%s]: RTK ASL altitude sample #%d: %.2f; avg: %.2f", getPrintName().c_str(), got_rtk_counter_, rtk_z, rtk_avg);
      return;

    } else {

      rtk_avg_init_z_     = rtk_avg;
      got_rtk_avg_init_z_ = true;
      ROS_INFO("[%s]: RTK ASL altitude avg: %f", getPrintName().c_str(), rtk_avg);
    }
  }
}
/*//}*/

}  // namespace rtk

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::rtk::Rtk, mrs_uav_managers::StateEstimator)
