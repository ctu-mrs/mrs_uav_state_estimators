/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/ground_truth.h>

//}

namespace mrs_uav_state_estimators
{

namespace ground_truth
{

/* initialize() //{*/
void GroundTruth::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |
  mrs_lib::ParamLoader param_loader(nh, getPrintName());

  bool success = true;

  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");

  if (!success) {
    ROS_ERROR("[%s]: could not load config file", getPrintName().c_str());
    return;
  }

  param_loader.setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  // | --------------------- load parameters -------------------- |
  param_loader.loadParam("max_flight_z", max_flight_z_);
  param_loader.loadParam("message/topic", msg_topic_);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;

  // | ------------------ timers initialization ----------------- |
  timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &GroundTruth::timerUpdate, this, false, false);  // not running after init
  timer_check_health_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &GroundTruth::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_gt_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, msg_topic_);

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
bool GroundTruth::start(void) {


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
bool GroundTruth::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;
  }
  return false;
}
/*//}*/

/*//{ reset() */
bool GroundTruth::reset(void) {

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

  mrs_msgs::UavState uav_state = uav_state_init_;
  uav_state.header.stamp       = time_now;

  uav_state.pose.position    = msg->pose.pose.position;
  uav_state.pose.orientation = msg->pose.pose.orientation;

  uav_state.velocity.linear = Support::rotateVector(msg->twist.twist.linear, msg->pose.pose.orientation);

  uav_state.velocity.angular = msg->twist.twist.angular;

  const nav_msgs::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::Odometry innovation = innovation_init_;
  innovation.header.stamp       = time_now;

  innovation.pose.pose.position.x = prev_msg_->pose.pose.position.x - msg->pose.pose.position.x;
  innovation.pose.pose.position.y = prev_msg_->pose.pose.position.y - msg->pose.pose.position.y;
  innovation.pose.pose.position.z = prev_msg_->pose.pose.position.z - msg->pose.pose.position.z;

  mrs_msgs::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = time_now;
  twist_covariance.header.stamp = time_now;

  const int n_states = 6;  // TODO this should be defined somewhere else
  pose_covariance.values.resize(n_states * n_states);
  pose_covariance.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
  pose_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
  pose_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;

  twist_covariance.values.resize(n_states * n_states);
  twist_covariance.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
  twist_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
  twist_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);
  mrs_lib::set_mutexed(mtx_innovation_, innovation, innovation_);
  mrs_lib::set_mutexed(mtx_covariance_, pose_covariance, pose_covariance_);
  mrs_lib::set_mutexed(mtx_covariance_, twist_covariance, twist_covariance_);

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();

  prev_msg_ = msg;
}  // namespace ground_truth
/*//}*/

/*//{ timerCheckHealth() */
void GroundTruth::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (sh_gt_odom_.hasMsg()) {
      changeState(READY_STATE);
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is ready to start", getPrintName().c_str());
    } else {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for msg on topic %s", getPrintName().c_str(), sh_gt_odom_.topicName().c_str());
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

/*//{ setUavState() */
bool GroundTruth::setUavState(const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

}  // namespace ground_truth

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::ground_truth::GroundTruth, mrs_uav_managers::StateEstimator)
