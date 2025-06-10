/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/dummy.h>

//}

namespace mrs_uav_state_estimators
{

namespace dummy
{

/* initialize() //{*/
void Dummy::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  ph->param_loader->addYamlFile(ros::package::getPath(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
  ph->param_loader->addYamlFile(ros::package::getPath(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");

  ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  // | --------------------- load parameters -------------------- |
  ph->param_loader->loadParam("max_flight_z", max_flight_z_);

  // | ------------------ timers initialization ----------------- |
  //
  timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &Dummy::timerUpdate, this, false, false);  // not running after init
  timer_check_health_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &Dummy::timerCheckHealth, this);

  // | ---------------- publishers initialization --------------- |
  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 10);

  if (ch_->debug_topics.state) {
    ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, Support::toSnakeCase(getName()) + "/uav_state", 10);
  }
  if (ch_->debug_topics.covariance) {
    ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/pose_covariance", 10);
    ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/twist_covariance", 10);
  }
  if (ch_->debug_topics.innovation) {
    ph_innovation_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/innovation", 10);
  }
  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics>(nh, Support::toSnakeCase(getName()) + "/diagnostics", 10);
  }

  // | ------------------ initialize published messages ------------------ |
  uav_state_init_.header.frame_id = ns_frame_id_;
  uav_state_init_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_init_.estimator_horizontal = est_lat_name_;
  uav_state_init_.estimator_vertical   = est_alt_name_;
  uav_state_init_.estimator_heading    = est_hdg_name_;

  uav_state_init_.pose.position.x = 0.0;
  uav_state_init_.pose.position.y = 0.0;
  uav_state_init_.pose.position.z = 0.0;

  uav_state_init_.pose.orientation.x = 0.0;
  uav_state_init_.pose.orientation.y = 0.0;
  uav_state_init_.pose.orientation.z = 0.0;
  uav_state_init_.pose.orientation.w = 1.0;

  uav_state_init_.velocity.linear.x = 0.0;
  uav_state_init_.velocity.linear.y = 0.0;
  uav_state_init_.velocity.linear.z = 0.0;

  uav_state_init_.velocity.angular.x = 0.0;
  uav_state_init_.velocity.angular.y = 0.0;
  uav_state_init_.velocity.angular.z = 0.0;

  innovation_init_.header.frame_id         = ns_frame_id_;
  innovation_init_.child_frame_id          = ch_->frames.ns_fcu;
  innovation_init_.pose.pose.orientation.w = 1.0;

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
void Dummy::timerUpdate([[maybe_unused]] const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  updateUavState();

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}  // namespace dummy
/*//}*/

/*//{ timerCheckHealth() */
void Dummy::timerCheckHealth([[maybe_unused]] const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    changeState(READY_STATE);
    ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is ready to start", getPrintName().c_str());
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

/*//{ setUavState() */
bool Dummy::setUavState([[maybe_unused]] const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ updateUavState() */
void Dummy::updateUavState() {
  const ros::Time time_now = ros::Time::now();

  mrs_msgs::UavState uav_state = uav_state_init_;
  uav_state_.header.stamp      = time_now;

  const nav_msgs::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::Odometry innovation = innovation_init_;

  innovation.header.stamp = time_now;

  innovation.pose.pose.position.x = 0.0;
  innovation.pose.pose.position.y = 0.0;
  innovation.pose.pose.position.z = 0.0;

  mrs_msgs::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance_.header.stamp  = time_now;
  twist_covariance_.header.stamp = time_now;

  const int n_states = 6;  // TODO this should be defined somewhere else
  pose_covariance.values.resize(n_states * n_states);
  twist_covariance.values.resize(n_states * n_states);

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);
  mrs_lib::set_mutexed(mtx_innovation_, innovation, innovation_);
  mrs_lib::set_mutexed(mtx_covariance_, pose_covariance, pose_covariance_);
  mrs_lib::set_mutexed(mtx_covariance_, twist_covariance, twist_covariance_);
}
/*//}*/
}  // namespace dummy

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::dummy::Dummy, mrs_uav_managers::StateEstimator)
