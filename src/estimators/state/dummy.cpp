/* includes //{ */

#include "estimators/state/dummy.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void Dummy::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                            // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &Dummy::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                        // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &Dummy::timerCheckHealth, this);

  // | ---------------- publishers initialization --------------- |
  ph_uav_state_        = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, Support::toSnakeCase(getName()) + "/uav_state", 1);
  ph_odom_             = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 1);
  ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/pose_covariance", 1);
  ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/twist_covariance", 1);
  ph_innovation_       = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/innovation", 1);
  ph_diagnostics_      = mrs_lib::PublisherHandler<EstimatorDiagnostics>(nh, Support::toSnakeCase(getName()) + "/diagnostics", 1);

  // | ------------------ initialize published messages ------------------ |
  uav_state_.header.frame_id = ns_frame_id_;
  uav_state_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_.estimator_horizontal.name = est_lat_name_;
  uav_state_.estimator_vertical.name   = est_alt_name_;
  uav_state_.estimator_heading.name    = est_hdg_name_;

  uav_state_.pose.position.x = 0.0;
  uav_state_.pose.position.y = 0.0;
  uav_state_.pose.position.z = 0.0;

  uav_state_.pose.orientation.x = 0.0;
  uav_state_.pose.orientation.y = 0.0;
  uav_state_.pose.orientation.z = 0.0;
  uav_state_.pose.orientation.w = 1.0;

  uav_state_.velocity.linear.x = 0.0;
  uav_state_.velocity.linear.y = 0.0;
  uav_state_.velocity.linear.z = 0.0;

  uav_state_.velocity.angular.x = 0.0;
  uav_state_.velocity.angular.y = 0.0;
  uav_state_.velocity.angular.z = 0.0;

  innovation_.header.frame_id      = ns_frame_id_;
  innovation_.child_frame_id       = ch_->frames.ns_fcu;
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
bool Dummy::start(void) {


  if (isInState(READY_STATE)) {

    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

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
bool Dummy::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool Dummy::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void Dummy::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  const ros::Time time_now = ros::Time::now();

  {
    std::scoped_lock lock(mtx_uav_state_);

    uav_state_.header.stamp = time_now;

  }

  {
    std::scoped_lock lock(mtx_uav_state_, mtx_odom_);
    odom_ = Support::uavStateToOdom(uav_state_, ch_->transformer);
  }

  {
    std::scoped_lock lock(mtx_innovation_);

    innovation_.header.stamp = time_now;

    innovation_.pose.pose.position.x = 0.0;
    innovation_.pose.pose.position.y = 0.0;
    innovation_.pose.pose.position.z = 0.0;
  }

  {
    std::scoped_lock lock(mtx_covariance_);

    pose_covariance_.header.stamp  = time_now;
    twist_covariance_.header.stamp = time_now;

    const int n_states = 6;  // TODO this should be defined somewhere else
    pose_covariance_.values.resize(n_states * n_states);
    twist_covariance_.values.resize(n_states * n_states);
  }

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();

}
/*//}*/

/*//{ timerCheckHealth() */
void Dummy::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

      changeState(READY_STATE);
      ROS_INFO("[%s]: Estimator is ready to start", getPrintName().c_str());
  }


  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }
}
/*//}*/

/*//{ isConverged() */
bool Dummy::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ getUavState() */
mrs_msgs::UavState Dummy::getUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  return uav_state_;
}
/*//}*/

/*//{ getInnovation() */
nav_msgs::Odometry Dummy::getInnovation() const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_;
}
/*//}*/

/*//{ getPoseCovariance() */
std::vector<double> Dummy::getPoseCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return pose_covariance_.values;
}
/*//}*/

/*//{ getTwistCovariance() */
std::vector<double> Dummy::getTwistCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return twist_covariance_.values;
}
/*//}*/

/*//{ setUavState() */
bool Dummy::setUavState(const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

};  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::Dummy, mrs_uav_state_estimation::StateEstimator)
