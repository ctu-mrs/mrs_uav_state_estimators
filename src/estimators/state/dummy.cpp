/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/dummy.h>

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

namespace dummy
{

/* initialize() //{*/
void Dummy::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  if (is_core_plugin_) {

    ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
  }

  // | --------------------- load parameters -------------------- |

  ph->param_loader->loadParam("max_flight_z", max_flight_z_);

  // | ------------------ timers initialization ----------------- |
  //
  mrs_lib::TimerHandlerOptions opts_no_autostart;

  opts_no_autostart.node      = node_;
  opts_no_autostart.autostart = false;

  mrs_lib::TimerHandlerOptions opts_autostart;

  opts_autostart.node      = node_;
  opts_autostart.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&Dummy::timerUpdate, this);

    timer_update_ = std::make_shared<TimerType>(opts_no_autostart, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&Dummy::timerCheckHealth, this);

    timer_check_health_ = std::make_shared<TimerType>(opts_autostart, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  // | ---------------- publishers initialization --------------- |

  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/" + Support::toSnakeCase(getName()) + "/odom");  // needed for tf

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
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool Dummy::start(void) {

  if (isInState(READY_STATE)) {

    timer_update_->start();
    timer_check_health_->start();

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
bool Dummy::pause(void) {

  if (isInState(RUNNING_STATE)) {

    timer_update_->stop();
    timer_check_health_->stop();

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
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void Dummy::timerUpdate() {

  if (!isInitialized()) {
    return;
  }

  const rclcpp::Time time_now = clock_->now();

  mrs_msgs::msg::UavState uav_state = uav_state_init_;

  uav_state.header.stamp            = time_now;

  const nav_msgs::msg::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::msg::Odometry innovation = innovation_init_;

  innovation.header.stamp = time_now;

  innovation.pose.pose.position.x = 0.0;
  innovation.pose.pose.position.y = 0.0;
  innovation.pose.pose.position.z = 0.0;

  mrs_msgs::msg::Float64ArrayStamped pose_covariance, twist_covariance;
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

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}  // namespace dummy
/*//}*/

/*//{ timerCheckHealth() */
void Dummy::timerCheckHealth() {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    changeState(READY_STATE);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is ready to start", getPrintName().c_str());
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
bool Dummy::setUavState([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

}  // namespace dummy

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::dummy::Dummy, mrs_uav_managers::StateEstimator)
