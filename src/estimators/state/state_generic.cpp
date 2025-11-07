/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

//}

/* using //{ */

using namespace std::chrono_literals;

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

/* initialize() //{*/
void StateGeneric::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ch_ = ch;
  ph_ = ph;

  if (is_core_plugin_) {

    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
  }

  ph->param_loader->loadParam("estimators/lateral/name", est_lat_name_);
  ph->param_loader->loadParam("estimators/altitude/name", est_alt_name_);
  ph->param_loader->loadParam("estimators/heading/name", est_hdg_name_);
  ph->param_loader->loadParam("estimators/heading/passthrough", is_hdg_passthrough_);

  ph->param_loader->loadParam("override_frame_id/enabled", is_override_frame_id_);
  if (is_override_frame_id_) {
    ph->param_loader->loadParam("override_frame_id/frame_id", frame_id_);
  }

  std::string topic_orientation;
  ph->param_loader->loadParam("topics/orientation", topic_orientation);
  topic_orientation_ = "/" + ch_->uav_name + "/" + topic_orientation;
  std::string topic_angular_velocity;
  ph->param_loader->loadParam("topics/angular_velocity", topic_angular_velocity);
  topic_angular_velocity_ = "/" + ch_->uav_name + "/" + topic_angular_velocity;

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
  }

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&StateGeneric::timerUpdate, this);

    timer_update_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&StateGeneric::timerPubAttitude, this);

    timer_pub_attitude_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  // | --------------- subscribers initialization --------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node;
  shopts.node_name                           = getPrintName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_hw_api_orient_  = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, topic_orientation_);
  sh_hw_api_ang_vel_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped>(shopts, topic_angular_velocity_);

  // | ---------------- publishers initialization --------------- |

  ph_odom_     = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase("~/" + getName()) + "/odom");
  ph_attitude_ = mrs_lib::PublisherHandler<geometry_msgs::msg::QuaternionStamped>(node_, Support::toSnakeCase("~/" + getName()) + "/attitude");

  if (ch_->debug_topics.state) {
    ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>(node_, Support::toSnakeCase("~/" + getName()) + "/uav_state");
  }

  if (ch_->debug_topics.covariance) {
    ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, Support::toSnakeCase("~/" + getName()) + "/pose_covariance");
    ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, Support::toSnakeCase("~/" + getName()) + "/twist_covariance");
  }

  if (ch_->debug_topics.innovation) {
    ph_innovation_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase("~/" + getName()) + "/innovation");
  }

  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics>(node_, Support::toSnakeCase("~/" + getName()) + "/diagnostics");
  }

  // | ---------------- estimators initialization --------------- |

  std::vector<double> max_altitudes;

  {
    rclcpp::Node::SharedPtr subnode = node_->create_sub_node(est_hdg_name_);
    auto                    est_ph  = std::make_shared<PrivateHandlers_t>();
    est_ph->loadConfigFile          = ph_->loadConfigFile;
    est_ph->param_loader            = std::make_unique<mrs_lib::ParamLoader>(subnode);
    est_ph->param_loader->copyYamls(*ph_->param_loader);
    est_ph->param_loader->setPrefix(ph_->param_loader->getPrefix());

    if (is_hdg_passthrough_) {
      est_hdg_ = std::make_unique<HdgPassthrough>(est_hdg_name_, frame_id_, getName(), is_core_plugin_);
    } else {
      est_hdg_ = std::make_unique<HdgGeneric>(est_hdg_name_, frame_id_, getName(), is_core_plugin_);
    }
    est_hdg_->initialize(subnode, ch_, est_ph);
    max_altitudes.push_back(est_hdg_->getMaxFlightZ());
  }

  {
    rclcpp::Node::SharedPtr subnode = node_->create_sub_node(est_lat_name_);
    auto                    est_ph  = std::make_shared<PrivateHandlers_t>();
    est_ph->loadConfigFile          = ph_->loadConfigFile;
    est_ph->param_loader            = std::make_unique<mrs_lib::ParamLoader>(subnode);
    est_ph->param_loader->copyYamls(*ph_->param_loader);
    est_ph->param_loader->setPrefix(ph_->param_loader->getPrefix());

    est_lat_ = std::make_unique<LatGeneric>(est_lat_name_, frame_id_, getName(), is_core_plugin_, [this](void) { return this->getHeading(); });
    est_lat_->initialize(subnode, ch_, est_ph);
    max_altitudes.push_back(est_lat_->getMaxFlightZ());
  }

  {
    rclcpp::Node::SharedPtr subnode = node_->create_sub_node(est_alt_name_);
    auto                    est_ph  = std::make_shared<PrivateHandlers_t>();
    est_ph->loadConfigFile          = ph_->loadConfigFile;
    est_ph->param_loader            = std::make_unique<mrs_lib::ParamLoader>(subnode);
    est_ph->param_loader->copyYamls(*ph_->param_loader);
    est_ph->param_loader->setPrefix(ph_->param_loader->getPrefix());

    est_alt_ = std::make_unique<AltGeneric>(est_alt_name_, frame_id_, getName(), is_core_plugin_);
    est_alt_->initialize(subnode, ch_, est_ph);
    max_altitudes.push_back(est_alt_->getMaxFlightZ());
  }

  max_flight_z_ = *std::min_element(max_altitudes.begin(), max_altitudes.end());

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
bool StateGeneric::start(void) {

  if (isInState(READY_STATE)) {

    bool est_lat_start_successful, est_alt_start_successful, est_hdg_start_successful;

    if (est_lat_->isStarted() || est_lat_->isRunning()) {
      est_lat_start_successful = true;
    } else {
      est_lat_start_successful = est_lat_->start();
    }

    if (est_alt_->isStarted() || est_alt_->isRunning()) {
      est_alt_start_successful = true;
    } else {
      est_alt_start_successful = est_alt_->start();
    }

    if (est_hdg_->isStarted() || est_hdg_->isRunning()) {
      timer_pub_attitude_->start();
      est_hdg_start_successful = true;
    } else {
      est_hdg_start_successful = est_hdg_->start();
    }

    if (est_lat_start_successful && est_alt_start_successful && est_hdg_start_successful) {

      timer_update_->start();
      timer_pub_attitude_->start();

      changeState(STARTED_STATE);
      return true;
    }

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
bool StateGeneric::pause(void) {

  if (isInState(RUNNING_STATE)) {

    est_lat_->pause();
    est_alt_->pause();
    est_hdg_->pause();

    timer_update_->stop();
    timer_pub_attitude_->stop();

    changeState(STOPPED_STATE);

    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool StateGeneric::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);
  est_lat_->reset();
  est_alt_->reset();
  est_hdg_->reset();
  changeState(INITIALIZED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void StateGeneric::timerUpdate() {


  if (!isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "StateGeneric::timerUpdate", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  switch (getCurrentSmState()) {

  case UNINITIALIZED_STATE: {
    break;
  }
  case INITIALIZED_STATE: {

    if (sh_hw_api_orient_.hasMsg() && sh_hw_api_ang_vel_.hasMsg()) {
      if (est_lat_->isInitialized() && est_alt_->isInitialized() && est_hdg_->isInitialized()) {
        changeState(READY_STATE);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is ready to start", getPrintName().c_str());
      } else {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s subestimators to be initialized", getPrintName().c_str(),
                             Support::waiting_for_string.c_str());
        return;
      }
    } else {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(),
                           sh_hw_api_orient_.topicName().c_str());
      return;
    }

    break;
  }

  case READY_STATE: {
    break;
  }

  case STARTED_STATE: {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s convergence of LKF", getPrintName().c_str(), Support::waiting_for_string.c_str());

    if (est_lat_->isError() || est_alt_->isError() || est_hdg_->isError()) {
      changeState(ERROR_STATE);
    }

    if (est_lat_->isRunning() && est_alt_->isRunning() && est_hdg_->isRunning()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Subestimators converged", getPrintName().c_str());
      changeState(RUNNING_STATE);
    } else {
      return;
    }
    break;
  }

  case RUNNING_STATE: {
    if (est_lat_->isError() || est_alt_->isError() || est_hdg_->isError()) {
      changeState(ERROR_STATE);
    }
    break;
  }

  case STOPPED_STATE: {
    break;
  }

  case ERROR_STATE: {
    if ((est_lat_->isReady() || est_lat_->isRunning()) && (est_alt_->isReady() || est_alt_->isRunning()) && (est_hdg_->isReady() || est_hdg_->isRunning())) {
      changeState(READY_STATE);
    }
    break;
  }
  }

  if (!isRunning() && !isStarted()) {
    return;
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

/* timerPubAttitude() //{*/
void StateGeneric::timerPubAttitude() {

  if (!isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "StateGeneric::timerPubAttitude", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (!sh_hw_api_orient_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(),
                         sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!est_hdg_->isRunning() && !isError()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: cannot publish attitude, heading estimator is not running", getPrintName().c_str());
    return;
  }

  scope_timer.checkpoint("checks");

  const rclcpp::Time time_now = clock_->now();

  geometry_msgs::msg::QuaternionStamped att;
  att.header.stamp    = time_now;
  att.header.frame_id = ns_frame_id_ + "_att_only";

  double hdg;
  if (isError()) {
    hdg = est_hdg_->getLastValidHdg();
  } else {
    hdg = est_hdg_->getState(POSITION);
  }

  auto res = rotateQuaternionByHeading(sh_hw_api_orient_.getMsg()->quaternion, hdg);
  if (res) {
    att.quaternion = res.value();
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not rotate orientation by heading", getPrintName().c_str());
    return;
  }

  scope_timer.checkpoint("rotate");

  if (!Support::noNans(att.quaternion)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: NaNs in timerPubAttitude quaternion", getPrintName().c_str());
    return;
  }

  scope_timer.checkpoint("nan check");

  ph_attitude_.publish(att);
  scope_timer.checkpoint("publish");
}
/*//}*/

/*//{ isConverged() */
bool StateGeneric::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ updateUavState() */
void StateGeneric::updateUavState() {

  std::scoped_lock lock(mutex_update_uav_state_);

  if (!sh_hw_api_orient_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(),
                         sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!sh_hw_api_ang_vel_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received angular velocity on topic %s yet", getPrintName().c_str(),
                         sh_hw_api_ang_vel_.topicName().c_str());
    return;
  }
  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer(node_, "StateGeneric::updateUavState", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  const rclcpp::Time time_now = clock_->now();

  mrs_msgs::msg::UavState uav_state = uav_state_init_;
  uav_state.header.stamp            = time_now;

  // do not rotate orientation if passthrough hdg
  if (est_hdg_name_ == "hdg_passthrough") {
    uav_state.pose.orientation = sh_hw_api_orient_.getMsg()->quaternion;
  } else {
    double hdg;
    if (isError()) {
      hdg = est_hdg_->getLastValidHdg();
    } else {
      hdg = est_hdg_->getState(POSITION);
    }

    auto res = rotateQuaternionByHeading(sh_hw_api_orient_.getMsg()->quaternion, hdg);
    if (res) {
      uav_state.pose.orientation = res.value();
    } else {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not rotate orientation by heading", getPrintName().c_str());
      return;
    }
  }

  scope_timer.checkpoint("rotate orientation");

  uav_state.velocity.angular = sh_hw_api_ang_vel_.getMsg()->vector;

  uav_state.pose.position.x = est_lat_->getState(POSITION, AXIS_X);
  uav_state.pose.position.y = est_lat_->getState(POSITION, AXIS_Y);
  uav_state.pose.position.z = est_alt_->getState(POSITION);

  uav_state.velocity.linear.x = est_lat_->getState(VELOCITY, AXIS_X); // in global frame
  uav_state.velocity.linear.y = est_lat_->getState(VELOCITY, AXIS_Y); // in global frame
  uav_state.velocity.linear.z = est_alt_->getState(VELOCITY);         // in global frame

  uav_state.acceleration.linear.x = est_lat_->getState(ACCELERATION, AXIS_X); // in global frame
  uav_state.acceleration.linear.y = est_lat_->getState(ACCELERATION, AXIS_Y); // in global frame
  uav_state.acceleration.linear.z = est_alt_->getState(ACCELERATION);         // in global frame

  scope_timer.checkpoint("fill uav state");

  const nav_msgs::msg::Odometry odom = Support::uavStateToOdom(uav_state);
  scope_timer.checkpoint("uav state to odom");

  nav_msgs::msg::Odometry innovation = innovation_init_;
  innovation.header.stamp            = time_now;

  innovation.pose.pose.position.x = est_lat_->getInnovation(POSITION, AXIS_X);
  innovation.pose.pose.position.y = est_lat_->getInnovation(POSITION, AXIS_Y);
  innovation.pose.pose.position.z = est_alt_->getInnovation(POSITION);

  is_mitigating_jump_ = est_alt_->isMitigatingJump() || est_lat_->isMitigatingJump() || est_hdg_->isMitigatingJump();

  scope_timer.checkpoint("innovation");

  mrs_msgs::msg::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = time_now;
  twist_covariance.header.stamp = time_now;

  const int n_states = 6; // TODO this should be defined somewhere else
  pose_covariance.values.resize(n_states * n_states);
  pose_covariance.values.at(n_states * AXIS_X + AXIS_X) = est_lat_->getCovariance(POSITION, AXIS_X);
  pose_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = est_lat_->getCovariance(POSITION, AXIS_Y);
  pose_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = est_alt_->getCovariance(POSITION);

  twist_covariance.values.resize(n_states * n_states);
  twist_covariance.values.at(n_states * AXIS_X + AXIS_X) = est_lat_->getCovariance(VELOCITY, AXIS_X);
  twist_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = est_lat_->getCovariance(VELOCITY, AXIS_Y);
  twist_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = est_alt_->getCovariance(VELOCITY);

  scope_timer.checkpoint("covariance");

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);
  mrs_lib::set_mutexed(mtx_innovation_, innovation, innovation_);
  mrs_lib::set_mutexed(mtx_covariance_, pose_covariance, pose_covariance_);
  mrs_lib::set_mutexed(mtx_covariance_, twist_covariance, twist_covariance_);
}

/*//}*/

/*//{ getHeading() */
std::optional<double> StateGeneric::getHeading() const {
  if (!est_hdg_->isRunning()) {
    return {};
  }
  return est_hdg_->getState(POSITION);
}
/*//}*/

/*//{ setUavState() */
bool StateGeneric::setUavState([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

} // namespace mrs_uav_state_estimators
