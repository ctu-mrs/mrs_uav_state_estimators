/* includes //{ */

#include "estimators/state/ground_truth.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void GroundTruth::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // TODO load parameters
  Support::loadParamFile(ros::package::getPath(ch_->package_name) + "/config/estimators/" + getName() + "/" + getName() + ".yaml", nh.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getName());
  param_loader.loadParam("message/topic", msg_topic_);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                            // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &GroundTruth::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                              // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &GroundTruth::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_gt_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, msg_topic_);

  // | ---------------- publishers initialization --------------- |
  ph_uav_state_        = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, Support::toSnakeCase(getName()) + "/uav_state", 1);
  ph_odom_             = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 1);
  ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/pose_covariance", 1);
  ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/twist_covariance", 1);
  ph_innovation_       = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/innovation", 1);
  ph_diagnostics_      = mrs_lib::PublisherHandler<EstimatorDiagnostics>(nh, Support::toSnakeCase(getName()) + "/diagnostics", 1);

  // | ---------------- estimators initialization --------------- |
  /* est_lat_gps_ = std::make_unique<LatGeneric>(est_lat_name_, frame_id_, getName()); */
  /* est_lat_gps_->initialize(nh, ch_); */

  /* est_alt_garmin_ = std::make_unique<AltGeneric>(est_alt_name_, frame_id_, getName()); */
  /* est_alt_garmin_->initialize(nh, ch_); */

  /* est_hdg_mavros_ = std::make_unique<HdgPassthrough>(est_hdg_name_, frame_id_, getName()); */
  /* est_hdg_mavros_->initialize(nh, ch_); */

  // | ------------------ initialize published messages ------------------ |
  uav_state_.header.frame_id = ns_frame_id_;
  uav_state_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_.estimator_horizontal.name = est_lat_name_;
  uav_state_.estimator_vertical.name   = est_alt_name_;
  uav_state_.estimator_heading.name    = est_hdg_name_;

  innovation_.header.frame_id      = ns_frame_id_;
  innovation_.child_frame_id       = ch_->frames.ns_fcu;
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
    ROS_INFO("[%s]: Estimator initialized", getName().c_str());
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool GroundTruth::start(void) {


  if (isInState(READY_STATE)) {

    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

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
bool GroundTruth::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool GroundTruth::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void GroundTruth::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  const ros::Time time_now = ros::Time::now();

  nav_msgs::OdometryConstPtr msg = sh_gt_odom_.getMsg();

  if (first_iter_) {
    prev_msg_   = msg;
    first_iter_ = false;
  }

  {
    std::scoped_lock lock(mtx_uav_state_);

    uav_state_.header.stamp = time_now;

    // TODO fill in estimator types

    uav_state_.pose.position    = msg->pose.pose.position;
    uav_state_.pose.orientation = msg->pose.pose.orientation;

    uav_state_.velocity.linear  = msg->twist.twist.linear;  // GT twist from Gazebo is in global frame
    uav_state_.velocity.angular = msg->twist.twist.angular;
  }

  {
    std::scoped_lock lock(mtx_uav_state_, mtx_odom_);
    odom_ = Support::uavStateToOdom(uav_state_, ch_->transformer);
  }

  {
    std::scoped_lock lock(mtx_innovation_);

    innovation_.header.stamp = time_now;

    innovation_.pose.pose.position.x = prev_msg_->pose.pose.position.x - msg->pose.pose.position.x;
    innovation_.pose.pose.position.y = prev_msg_->pose.pose.position.y - msg->pose.pose.position.y;
    innovation_.pose.pose.position.z = prev_msg_->pose.pose.position.z - msg->pose.pose.position.z;
  }

  {
    std::scoped_lock lock(mtx_covariance_);

    pose_covariance_.header.stamp  = time_now;
    twist_covariance_.header.stamp = time_now;

    const int n_states = 6;  // TODO this should be defined somewhere else
    pose_covariance_.values.resize(n_states * n_states);
    pose_covariance_.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
    pose_covariance_.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
    pose_covariance_.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;

    twist_covariance_.values.resize(n_states * n_states);
    twist_covariance_.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
    twist_covariance_.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
    twist_covariance_.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;
  }

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();

  prev_msg_ = msg;
}
/*//}*/

/*//{ timerCheckHealth() */
void GroundTruth::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (sh_gt_odom_.hasMsg()) {
      changeState(READY_STATE);
      ROS_INFO("[%s]: Estimator is ready to start", getName().c_str());
    } else {
      ROS_INFO("[%s]: Waiting for msg on topic %s", getName().c_str(), sh_gt_odom_.topicName().c_str());
      return;
    }
  }


  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }
}
/*//}*/

/*//{ isConverged() */
bool GroundTruth::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ getUavState() */
mrs_msgs::UavState GroundTruth::getUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  return uav_state_;
}
/*//}*/

/*//{ getPoseCovariance() */
std::vector<double> GroundTruth::getPoseCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return pose_covariance_.values;
}
/*//}*/

/*//{ getTwistCovariance() */
std::vector<double> GroundTruth::getTwistCovariance() const {
  std::scoped_lock lock(mtx_covariance_);
  return twist_covariance_.values;
}
/*//}*/

/*//{ setUavState() */
bool GroundTruth::setUavState(const mrs_msgs::UavState &uav_state) {

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
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::GroundTruth, mrs_uav_state_estimation::StateEstimator)
