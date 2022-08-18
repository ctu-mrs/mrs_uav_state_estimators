#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/state/gps_garmin.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void GpsGarmin::initialize(const ros::NodeHandle &parent_nh) {

  nh_ = parent_nh;

  // TODO load parameters

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

  // | ---------------- publishers initialization --------------- |
  pub_uav_state_   = nh_.advertise<mrs_msgs::UavState>(toSnakeCase(getName()) + "/output", 1);
  pub_diagnostics_ = nh_.advertise<EstimatorDiagnostics>(toSnakeCase(getName()) + "/diagnostics", 1);

  // | ---------------- estimators initialization --------------- |
  est_lat_gps_ = std::make_unique<Gps>();
  est_lat_gps_->initialize(nh_);

  est_alt_garmin_ = std::make_unique<Garmin>();
  est_alt_garmin_->initialize(nh_);

  // | ------------------ initialize uav state ------------------ |
  uav_state_.header.frame_id = frame_id_;
  uav_state_.child_frame_id  = "fcu";

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

  publishUavState();
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
