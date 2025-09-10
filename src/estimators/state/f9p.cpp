/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/f9p.h>

//}

namespace mrs_uav_state_estimators
{

namespace f9p
{

/* initialize() //{*/
void F9P::initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ros::NodeHandle nh(parent_nh);

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  if (is_core_plugin_) {

    ph_->param_loader->addYamlFile(ros::package::getPath(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    ph_->param_loader->addYamlFile(ros::package::getPath(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
  }

  ph_->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  // | --------------------- load parameters -------------------- |
  ph_->param_loader->loadParam("max_flight_z", max_flight_z_);
  ph_->param_loader->loadParam("message/topic", msg_topic_);
  ph_->param_loader->loadParam("kickoff", kickoff_, false);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;

  ph->param_loader->loadParam("gnss/topic", gnss_topic_);

  gnss_topic_ = "/" + ch->uav_name + "/" + gnss_topic_;

  // | --------------- subscribers initialization --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_f9p_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, msg_topic_, &F9P::callbackF9POdom, this);

  sh_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, gnss_topic_, &F9P::callbackGnss, this);

  // | ------------------ timers initialization ----------------- |
  t_check_hz_last_                 = ros::Time::now();
  prev_avg_hz_                     = 0;
  timer_check_f9p_odom_hz_ = nh.createTimer(ros::Rate(1.0), &F9P::timerCheckF9POdomHz, this);

  // | ---------------- publishers initialization --------------- |
  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 10);  // needed for tf
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
bool F9P::start(void) {


  if (isInState(READY_STATE)) {

    /* timer_update_.start(); */
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
bool F9P::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;
  }
  return false;
}
/*//}*/

/*//{ reset() */
bool F9P::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerCheckF9POdomHz() //{*/
void F9P::timerCheckF9POdomHz([[maybe_unused]] const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    // first wait for a message
    if (!sh_f9p_odom_.hasMsg()) {
      ROS_INFO_THROTTLE(1.0, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(), sh_f9p_odom_.topicName().c_str());
      return;
    }

    // calculate average rate of messages
    if (counter_odom_msgs_ > 0) {
      ros::Time t_now  = ros::Time::now();
      double        dt     = (t_now - t_check_hz_last_).toSec();
      double        avg_hz = counter_odom_msgs_ / dt;
      t_check_hz_last_     = t_now;
      counter_odom_msgs_   = 0;
      ROS_INFO("[%s]: rate of f9p odom: %.2f Hz", getPrintName().c_str(), avg_hz);

      // check message rate stability
      if (abs(avg_hz - prev_avg_hz_) >= 5) {
        ROS_INFO("[%s]: %s stable f9p odometry rate. now: %.2f Hz prev: %.2f Hz", getPrintName().c_str(), Support::waiting_for_string.c_str(), avg_hz,
                 prev_avg_hz_);
        prev_avg_hz_ = avg_hz;
        return;
      }

      // the message rate must be higher than required by the control manager
      if (!kickoff_ && avg_hz < ch_->desired_uav_state_rate * 0.9) {
        ROS_ERROR(
            "[%s]: rate of f9p odom: %.2f Hz is lower than desired uav_state rate: %.2f Hz. Flight not allowed. Provide higher f9p odometry rate "
            "or use a higher-level controller.",
            getPrintName().c_str(), avg_hz, ch_->desired_uav_state_rate);
        // note: might run the publishing asynchronously on the desired rate in this case to still be able to fly
        // the states would then be interpolated by ZOH (bleeh)
        /* timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &F9P::timerUpdate, this); */
        return;
      }
    }

    changeState(READY_STATE);
    ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is ready to start", getPrintName().c_str());
    timer_check_f9p_odom_hz_.stop();
  }
}
/*//}*/

/* timerUpdate() //{*/
void F9P::timerUpdate([[maybe_unused]] const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }

  // If the estimator is active the updateUavState is triggered directly by estimation manager
  if (!is_active_) {
    updateUavState();
  }

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();

}
/*//}*/

/*//{ callbackF9POdom() */
void F9P::callbackF9POdom([[maybe_unused]] const nav_msgs::Odometry::ConstPtr msg) {

  counter_odom_msgs_++;

  if (!isInitialized()) {
    return;
  }

  updateUavState();

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}
/*//}*/

/*//{ callbackGnss() */
void F9P::callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!isInitialized()) {
    return;
  }

  if (got_gnss_) {
    return;
  }

  if (!std::isfinite(msg->latitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->latitude\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->longitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->longitude\"!!!", getPrintName().c_str());
    return;
  }

  mrs_lib::UTM(msg->latitude, msg->longitude, &gnss_x_, &gnss_y_);
  gnss_z_ = msg->altitude;
  ROS_INFO("[%s]: First GNSS obtained: %.2f %.2f %.2f", getPrintName().c_str(), gnss_x_, gnss_y_, gnss_z_);
  got_gnss_ = true;
}
/*//}*/

/*//{ isConverged() */
bool F9P::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ setUavState() */
bool F9P::setUavState([[maybe_unused]] const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ updateUavState() */
void F9P::updateUavState() {

  if (!isInitialized()) {
    return;
  }

  if (!got_gnss_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: Missing GNSS data on topic: %s", getPrintName().c_str(), gnss_topic_.c_str());
    return;
  }

  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }

  nav_msgs::OdometryConstPtr msg = sh_f9p_odom_.getMsg();

  if (first_iter_) {

    gnss_x_                    = (gnss_x_ - msg->pose.pose.position.x) - ch_->world_origin.x;
    gnss_y_                    = (gnss_y_ - msg->pose.pose.position.y) - ch_->world_origin.y;
    /* gnss_z_                    = gnss_z_ - msg->pose.pose.position.z; */ // absolute AMSL altitude
    gnss_z_                    = -msg->pose.pose.position.z; // relative altitude to start

    prev_msg_   = msg;
    first_iter_ = false;
  }

  mrs_msgs::UavState uav_state = uav_state_init_;

  /* const ros::Time time_now = ros::Time::now(); */
  /* uav_state.header.stamp = time_now; */

  // here we are assuming the f9p odometry has correct timestamp
  uav_state.header.stamp = msg->header.stamp;

  uav_state.pose.position.x = msg->pose.pose.position.x + gnss_x_;
  uav_state.pose.position.y = msg->pose.pose.position.y + gnss_y_;
  uav_state.pose.position.z = msg->pose.pose.position.z + gnss_z_;

  uav_state.pose.orientation = msg->pose.pose.orientation;

  // uav_state has velocities in the parent frame
  uav_state.velocity.linear  = Support::rotateVector(msg->twist.twist.linear, msg->pose.pose.orientation);
  uav_state.velocity.angular = msg->twist.twist.angular;

  const nav_msgs::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::Odometry innovation = innovation_init_;
  innovation.header.stamp       = odom.header.stamp;

  innovation.pose.pose.position.x = prev_msg_->pose.pose.position.x - msg->pose.pose.position.x;
  innovation.pose.pose.position.y = prev_msg_->pose.pose.position.y - msg->pose.pose.position.y;
  innovation.pose.pose.position.z = prev_msg_->pose.pose.position.z - msg->pose.pose.position.z;

  mrs_msgs::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = odom.header.stamp;
  twist_covariance.header.stamp = odom.header.stamp;

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

  prev_msg_ = msg;
}
/*//}*/

}  // namespace f9p

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::f9p::F9P, mrs_uav_managers::StateEstimator)
