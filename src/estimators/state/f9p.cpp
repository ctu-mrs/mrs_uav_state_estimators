/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/f9p.h>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_state_estimators
{

namespace f9p
{

/* initialize() //{*/
void F9P::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  if (is_core_plugin_) {

    ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
  }

  // | --------------------- load parameters -------------------- |

  ph_->param_loader->loadParam("max_flight_z", max_flight_z_);
  ph_->param_loader->loadParam("message/topic", msg_topic_);
  ph_->param_loader->loadParam("kickoff", kickoff_, false);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;

  ph_->param_loader->loadParam("gnss/topic", gnss_topic_);
  gnss_topic_ = "/" + ch_->uav_name + "/" + gnss_topic_;

  // | --------------- subscribers initialization --------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = getPrintName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_f9p_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, msg_topic_, &F9P::callbackF9POdom, this);

  sh_gnss_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, gnss_topic_, &F9P::callbackGnss, this);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  t_check_hz_last_ = clock_->now();

  {
    std::function<void()> callback_fcn = std::bind(&F9P::timerCheckF9POdomHz, this);

    timer_check_f9p_odom_hz_ = std::make_shared<TimerType>(opts, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  // | ---------------- publishers initialization --------------- |

  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/" + Support::toSnakeCase(getName()) + "/odom"); // needed for tf
                                                                                                                          //
  if (ch_->debug_topics.state) {
    ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>(node_, "~/" + Support::toSnakeCase(getName()) + "/uav_state");
  }

  if (ch_->debug_topics.covariance) {
    ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, "~/" + Support::toSnakeCase(getName()) + "/pose_covariance");
    ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, "~/" + Support::toSnakeCase(getName()) + "/twist_covariance");
  }

  if (ch_->debug_topics.innovation) {
    ph_innovation_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase("~/" + getName()) + "/innovation");
  }

  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics>(node_, Support::toSnakeCase("~/" + getName()) + "/diagnostics");
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
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool F9P::start(void) {

  if (isInState(READY_STATE)) {

    timer_check_f9p_odom_hz_->start();

    changeState(STARTED_STATE);
    return true;

  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    clock_->sleep_for(1s);
  }
  return false;

  RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ pause() */
bool F9P::pause(void) {

  if (isInState(RUNNING_STATE)) {

    timer_check_f9p_odom_hz_->stop();

    changeState(STOPPED_STATE);

    return true;
  }
  return false;
}
/*//}*/

/*//{ reset() */
bool F9P::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerCheckF9POdomHz() //{*/
void F9P::timerCheckF9POdomHz() {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    // first wait for a message
    if (!sh_f9p_odom_.hasMsg()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(),
                           sh_f9p_odom_.topicName().c_str());
      return;
    }

    // calculate average rate of messages
    if (counter_odom_msgs_ > 0) {

      rclcpp::Time t_now  = clock_->now();
      double       dt     = (t_now - t_check_hz_last_).seconds();
      double       avg_hz = counter_odom_msgs_ / dt;
      t_check_hz_last_    = t_now;
      counter_odom_msgs_  = 0;
      RCLCPP_INFO(node_->get_logger(), "[%s]: rate of f9p odom: %.2f Hz", getPrintName().c_str(), avg_hz);

      // check message rate stability
      if (abs(avg_hz - prev_avg_hz_) >= 5) {
        RCLCPP_INFO(node_->get_logger(), "[%s]: %s stable f9p odometry rate. now: %.2f Hz prev: %.2f Hz", getPrintName().c_str(),
                    Support::waiting_for_string.c_str(), avg_hz, prev_avg_hz_);
        prev_avg_hz_ = avg_hz;
        return;
      }

      // the message rate must be higher than required by the control manager
      if (!kickoff_ && avg_hz < ch_->desired_uav_state_rate * 0.9) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s]: rate of f9p odom: %.2f Hz is lower than desired uav_state rate: %.2f Hz. Flight not allowed. Provide higher f9p odometry "
                     "rate "
                     "or use a higher-level controller.",
                     getPrintName().c_str(), avg_hz, ch_->desired_uav_state_rate);
        // note: might run the publishing asynchronously on the desired rate in this case to still be able to fly
        // the states would then be interpolated by ZOH (bleeh)
        /* timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &F9P::timerUpdate, this); */
        return;
      }
    }

    changeState(READY_STATE);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is ready to start", getPrintName().c_str());
    timer_check_f9p_odom_hz_->stop();
  }
}
/*//}*/

/* timerUpdate() //{*/

void F9P::timerUpdate() {

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

/* updateUavState() //{ */

void F9P::updateUavState() {

  std::scoped_lock lock(mutex_update_uav_state_);

  if (!isInitialized()) {
    return;
  }

  if (!got_gnss_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Missing GNSS data on topic %s", getPrintName().c_str(), sh_f9p_odom_.topicName().c_str());
    return;
  }

  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }

  nav_msgs::msg::Odometry::ConstSharedPtr msg = sh_f9p_odom_.getMsg();

  if (first_iter_) {

    gnss_x_ = (gnss_x_ - msg->pose.pose.position.x) - ch_->world_origin.x;
    gnss_y_ = (gnss_y_ - msg->pose.pose.position.y) - ch_->world_origin.y;
    /* gnss_z_                    = gnss_z_ - msg->pose.pose.position.z; */ // absolute AMSL altitude
    gnss_z_ = -msg->pose.pose.position.z;                                   // relative altitude to start
                                                                            //
    prev_msg_   = msg;
    first_iter_ = false;
  }

  mrs_msgs::msg::UavState uav_state = uav_state_init_;

  /* const rclcpp::Time time_now = clock_->now(); */

  // here we are assuming the f9p odometry has correct timestamp
  const rclcpp::Time time_now = msg->header.stamp;
  uav_state.header.stamp      = time_now;

  uav_state.pose.position.x = msg->pose.pose.position.x + gnss_x_;
  uav_state.pose.position.y = msg->pose.pose.position.y + gnss_y_;
  uav_state.pose.position.z = msg->pose.pose.position.z + gnss_z_;

  uav_state.pose.orientation = msg->pose.pose.orientation;

  // uav_state has velocities in the parent frame
  uav_state.velocity.linear  = Support::rotateVector(msg->twist.twist.linear, msg->pose.pose.orientation);
  uav_state.velocity.angular = msg->twist.twist.angular;

  const nav_msgs::msg::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::msg::Odometry innovation = innovation_init_;
  innovation.header.stamp            = time_now;

  innovation.pose.pose.position.x = prev_msg_->pose.pose.position.x - msg->pose.pose.position.x;
  innovation.pose.pose.position.y = prev_msg_->pose.pose.position.y - msg->pose.pose.position.y;
  innovation.pose.pose.position.z = prev_msg_->pose.pose.position.z - msg->pose.pose.position.z;

  mrs_msgs::msg::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = time_now;
  twist_covariance.header.stamp = time_now;

  const int n_states = 6; // TODO this should be defined somewhere else
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

//}

/*//{ callbackF9POdom() */
void F9P::callbackF9POdom([[maybe_unused]] const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

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
void F9P::callbackGnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!isInitialized()) {
    return;
  }

  if (got_gnss_) {
    return;
  }

  if (!std::isfinite(msg->latitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in GNSS variable \"msg->latitude\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->longitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in GNSS variable \"msg->longitude\"!!!", getPrintName().c_str());
    return;
  }

  mrs_lib::UTM(msg->latitude, msg->longitude, &gnss_x_, &gnss_y_);
  gnss_z_ = msg->altitude;
  RCLCPP_INFO(node_->get_logger(), "[%s]: First GNSS obtained: %.2f %.2f %.2f", getPrintName().c_str(), gnss_x_, gnss_y_, gnss_z_);
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
bool F9P::setUavState([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

} // namespace f9p

} // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::f9p::F9P, mrs_uav_managers::StateEstimator)
