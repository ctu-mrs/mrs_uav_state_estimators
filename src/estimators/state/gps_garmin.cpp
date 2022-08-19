#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/state/gps_garmin.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void GpsGarmin::initialize(const ros::NodeHandle &parent_nh, const std::string &uav_name) {

  nh_       = parent_nh;
  uav_name_ = uav_name;

  // TODO load parameters
  mrs_lib::ParamLoader param_loader(nh_, getName());

  // | ----------------------- transformer ---------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>(nh_, getName());
  /* transformer_->setDefaultPrefix(_uav_name_); */
  /* transformer_->retryLookupNewest(true); */

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                           // TODO: parametrize
  timer_update_             = nh_.createTimer(ros::Rate(_update_timer_rate_), &GpsGarmin::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                             // TODO: parametrize
  timer_check_health_       = nh_.createTimer(ros::Rate(_check_health_timer_rate_), &GpsGarmin::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in");

  sh_attitude_command_ = mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>(shopts, "attitude_command_in", &GpsGarmin::callbackAttitudeCommand, this);

  // | ---------------- publishers initialization --------------- |
  ph_uav_state_   = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh_, toSnakeCase(getName()) + "/uav_state", 1);
  ph_innovation_   = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, toSnakeCase(getName()) + "/innovation", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<EstimatorDiagnostics>(nh_, toSnakeCase(getName()) + "/diagnostics", 1);

  // | ---------------- estimators initialization --------------- |
  est_lat_gps_ = std::make_unique<Gps>();
  est_lat_gps_->initialize(nh_, uav_name_);

  est_alt_garmin_ = std::make_unique<Garmin>();
  est_alt_garmin_->initialize(nh_, uav_name_);

  // | ------------------ initialize published messages ------------------ |
  uav_state_.header.frame_id = uav_name_ + "/" + frame_id_;
  uav_state_.child_frame_id  = uav_name_ + "/" + fcu_frame_id_;

  innovation_.header.frame_id = uav_name_ + "/" + frame_id_;
  innovation_.child_frame_id  = uav_name_ + "/" + fcu_frame_id_;
  innovation_.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  innovation_.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  innovation_.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();

  innovation_.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  innovation_.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  innovation_.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  innovation_.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();

  innovation_.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN(); 
  innovation_.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN(); 
  innovation_.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN(); 

  innovation_.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN(); 
  innovation_.twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN(); 
  innovation_.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN(); 

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool GpsGarmin::start(void) {


  if (isInState(READY_STATE)) {

    bool est_lat_gps_start_successful, est_alt_garmin_start_successful;

    if (est_lat_gps_->isStarted() || est_lat_gps_->isRunning()) {
      est_lat_gps_start_successful = true;
    } else {
      est_lat_gps_start_successful = est_lat_gps_->start();
    }


    if (est_alt_garmin_->isStarted() || est_alt_garmin_->isRunning()) {
      est_alt_garmin_start_successful = true;
    } else {
      est_alt_garmin_start_successful = est_alt_garmin_->start();
    }

    if (est_lat_gps_start_successful && est_alt_garmin_start_successful) {
      timer_update_.start();
      changeState(STARTED_STATE);
      return true;
    }

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getName().c_str());
    ros::Duration(1.0).sleep();
  }
  return false;

  ROS_ERROR("[%s]: Failed to start", getName().c_str());
  return false;
}
/*//}*/

/*//{ pause() */
bool GpsGarmin::pause(void) {

  if (isInState(RUNNING_STATE)) {
    est_lat_gps_->pause();
    est_alt_garmin_->pause();
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool GpsGarmin::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getName().c_str());
    return false;
  }

  est_lat_gps_->pause();
  est_alt_garmin_->pause();
  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void GpsGarmin::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  {
    std::scoped_lock lock(mtx_uav_state_);

    uav_state_.header.stamp = ros::Time::now();

    // TODO fill in estimator types

    uav_state_.pose.orientation = sh_mavros_odom_.getMsg()->pose.pose.orientation;

    uav_state_.velocity.angular = sh_mavros_odom_.getMsg()->twist.twist.angular;

    uav_state_.pose.position.x = est_lat_gps_->getState(POSITION, AXIS_X);
    uav_state_.pose.position.y = est_lat_gps_->getState(POSITION, AXIS_Y);
    uav_state_.pose.position.z = est_alt_garmin_->getState(POSITION);

    uav_state_.velocity.linear.x = est_lat_gps_->getState(VELOCITY, AXIS_X);  // in global frame
    uav_state_.velocity.linear.y = est_lat_gps_->getState(VELOCITY, AXIS_Y);  // in global frame
    uav_state_.velocity.linear.z = est_alt_garmin_->getState(VELOCITY);       // in global frame

    uav_state_.acceleration.linear.x = est_lat_gps_->getState(ACCELERATION, AXIS_X);  // in global frame
    uav_state_.acceleration.linear.y = est_lat_gps_->getState(ACCELERATION, AXIS_Y);  // in global frame
    uav_state_.acceleration.linear.z = est_alt_garmin_->getState(ACCELERATION);       // in global frame
  }

  innovation_.header.stamp = ros::Time::now();

  innovation_.pose.pose.position.x = est_lat_gps_->getInnovation(POSITION, AXIS_X);
  innovation_.pose.pose.position.y = est_lat_gps_->getInnovation(POSITION, AXIS_Y);
  innovation_.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN(); // TODO

  publishUavState();
  publishInnovation();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void GpsGarmin::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (sh_mavros_odom_.hasMsg()) {
      if (est_lat_gps_->isReady() && est_alt_garmin_->isReady()) {
        changeState(READY_STATE);
        ROS_INFO("[%s]: Estimator is ready to start", getName().c_str());
      } else {
        ROS_INFO("[%s]: Waiting for subestimators to be ready", getName().c_str());
        return;
      }
    } else {
      ROS_INFO("[%s]: Waiting for msg on topic %s", getName().c_str(), sh_mavros_odom_.topicName().c_str());
      return;
    }
  }


  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getName().c_str());

    if (est_lat_gps_->isRunning() && est_alt_garmin_->isRunning()) {
      ROS_INFO("[%s]: Subestimators converged", getName().c_str());
      changeState(RUNNING_STATE);
    } else {
      return;
    }
  }
}
/*//}*/

/*//{ callbackAttitudeCommand() */
void GpsGarmin::callbackAttitudeCommand(mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand> &wrp) {

  if (!isRunning()) {
    return;
  }

  if (sh_mavros_odom_.hasMsg()) {

    mrs_msgs::AttitudeCommandConstPtr msg = wrp.getMsg();
    // untilt the desired acceleration vector
    geometry_msgs::Vector3Stamped des_acc, des_acc_untilted;
    des_acc.vector.x          = msg->desired_acceleration.x;
    des_acc.vector.y          = msg->desired_acceleration.y;
    des_acc.vector.z          = msg->desired_acceleration.z;
    des_acc.header.frame_id = fcu_frame_id_;
    des_acc.header.stamp    = msg->header.stamp;
    auto response_acc                = transformer_->transformSingle(des_acc, fcu_untilted_frame_id_);
    if (response_acc) {
      des_acc_untilted = response_acc.value();
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: Transform from %s to %s failed", getName().c_str(), des_acc.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
    }

    // rotate the desired acceleration vector to global frame
    const tf2::Vector3 des_acc_global = rotateVecByHdg(des_acc_untilted.vector, mrs_lib::AttitudeConverter(sh_mavros_odom_.getMsg()->pose.pose.orientation).getHeading());

    est_lat_gps_->setInput(des_acc_global.getX(), des_acc_global.getY(), msg->header.stamp);
  }
}
/*//}*/

/*//{ isConverged() */
bool GpsGarmin::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ getUavState() */
mrs_msgs::UavState GpsGarmin::getUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  return uav_state_;
}
/*//}*/

/*//{ setUavState() */
bool GpsGarmin::setUavState(const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", ros::this_node::getName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", ros::this_node::getName().c_str());
  return false;
}
/*//}*/

};  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::GpsGarmin, mrs_uav_state_estimation::StateEstimator)
