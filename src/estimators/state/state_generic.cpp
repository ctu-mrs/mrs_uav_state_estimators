/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

//}

namespace mrs_uav_state_estimators
{

/* initialize() //{*/
void StateGeneric::initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ros::NodeHandle nh(parent_nh);

  if (is_core_plugin_) {
    bool success = true;
    success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
    if (!success) {
      ROS_ERROR("[%s]: could not load config file", getPrintName().c_str());
      ros::shutdown();
    }
  }

  mrs_lib::ParamLoader param_loader(nh, getPrintName());

  param_loader.setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  param_loader.loadParam("estimators/lateral/name", est_lat_name_);
  param_loader.loadParam("estimators/altitude/name", est_alt_name_);
  param_loader.loadParam("estimators/heading/name", est_hdg_name_);
  param_loader.loadParam("estimators/heading/passthrough", is_hdg_passthrough_);

  param_loader.loadParam("override_frame_id/enabled", is_override_frame_id_);
  if (is_override_frame_id_) {
    param_loader.loadParam("override_frame_id/frame_id", frame_id_);
  }

  std::string topic_orientation;
  param_loader.loadParam("topics/orientation", topic_orientation);
  topic_orientation_ = "/" + ch_->uav_name + "/" + topic_orientation;
  std::string topic_angular_velocity;
  param_loader.loadParam("topics/angular_velocity", topic_angular_velocity);
  topic_angular_velocity_ = "/" + ch_->uav_name + "/" + topic_angular_velocity;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | ------------------ timers initialization ----------------- |
  timer_update_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &StateGeneric::timerUpdate, this);  // not running after init
  /* timer_check_health_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &StateGeneric::timerCheckHealth, this); */
  timer_pub_attitude_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &StateGeneric::timerPubAttitude, this);

  // | --------------- subscribers initialization --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_orient_  = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, topic_orientation_);
  sh_hw_api_ang_vel_ = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, topic_angular_velocity_);

  // | ---------------- publishers initialization --------------- |
  ph_odom_     = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, Support::toSnakeCase(getName()) + "/odom", 10);
  ph_attitude_ = mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped>(nh, Support::toSnakeCase(getName()) + "/attitude", 10);

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

  // | ---------------- estimators initialization --------------- |
  std::vector<double> max_altitudes;

  if (is_hdg_passthrough_) {
    est_hdg_ = std::make_unique<HdgPassthrough>(est_hdg_name_, frame_id_, getName(), is_core_plugin_);
  } else {
    est_hdg_ = std::make_unique<HdgGeneric>(est_hdg_name_, frame_id_, getName(), is_core_plugin_);
  }
  est_hdg_->initialize(nh, ch_, ph_);
  max_altitudes.push_back(est_hdg_->getMaxFlightZ());

  est_lat_ = std::make_unique<LatGeneric>(est_lat_name_, frame_id_, getName(), is_core_plugin_, [this](void) { return this->getHeading(); });
  est_lat_->initialize(nh, ch_, ph_);
  max_altitudes.push_back(est_lat_->getMaxFlightZ());

  est_alt_ = std::make_unique<AltGeneric>(est_alt_name_, frame_id_, getName(), is_core_plugin_);
  est_alt_->initialize(nh, ch_, ph_);
  max_altitudes.push_back(est_alt_->getMaxFlightZ());

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
    ROS_INFO("[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getPrintName().c_str());
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
      timer_pub_attitude_.start();
      est_hdg_start_successful = true;
    } else {
      est_hdg_start_successful = est_hdg_->start();
    }

    if (est_lat_start_successful && est_alt_start_successful && est_hdg_start_successful) {
      /* timer_update_.start(); */
      changeState(STARTED_STATE);
      return true;
    }

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
bool StateGeneric::pause(void) {

  if (isInState(RUNNING_STATE)) {
    est_lat_->pause();
    est_alt_->pause();
    est_hdg_->pause();
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
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  est_lat_->pause();
  est_alt_->pause();
  est_hdg_->pause();
  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void StateGeneric::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("StateGeneric::timerUpdate", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      break;
    }
    case INITIALIZED_STATE: {

      if (sh_hw_api_orient_.hasMsg() && sh_hw_api_ang_vel_.hasMsg()) {
        if (est_lat_->isInitialized() && est_alt_->isInitialized() && est_hdg_->isInitialized()) {
          changeState(READY_STATE);
          ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is ready to start", getPrintName().c_str());
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: %s subestimators to be initialized", getPrintName().c_str(), Support::waiting_for_string.c_str());
          return;
        }
      } else {
        ROS_INFO_THROTTLE(1.0, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(), sh_hw_api_orient_.topicName().c_str());
        return;
      }

      break;
    }

    case READY_STATE: {
      break;
    }

    case STARTED_STATE: {

      ROS_INFO_THROTTLE(1.0, "[%s]: %s convergence of LKF", getPrintName().c_str(), Support::waiting_for_string.c_str());

      if (est_lat_->isRunning() && est_alt_->isRunning() && est_hdg_->isRunning()) {
        ROS_INFO_THROTTLE(1.0, "[%s]: Subestimators converged", getPrintName().c_str());
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
      if (est_lat_->isReady() && est_alt_->isReady() && est_hdg_->isReady()) {
        changeState(READY_STATE);
      }
      break;
    }
  }

  if (!isRunning() && !isStarted()) {
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

/*//{ timerCheckHealth() */
void StateGeneric::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("StateGeneric::timerCheckHealth", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      break;
    }
    case INITIALIZED_STATE: {

      if (sh_hw_api_orient_.hasMsg() && sh_hw_api_ang_vel_.hasMsg()) {
        if (est_lat_->isInitialized() && est_alt_->isInitialized() && est_hdg_->isInitialized()) {
          changeState(READY_STATE);
          ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is ready to start", getPrintName().c_str());
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: %s subestimators to be initialized", getPrintName().c_str(), Support::waiting_for_string.c_str());
          return;
        }
      } else {
        ROS_INFO_THROTTLE(1.0, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(), sh_hw_api_orient_.topicName().c_str());
        return;
      }

      break;
    }

    case READY_STATE: {
      break;
    }

    case STARTED_STATE: {

      ROS_INFO_THROTTLE(1.0, "[%s]: %s for convergence of LKF", getPrintName().c_str(), Support::waiting_for_string.c_str());

      if (est_lat_->isRunning() && est_alt_->isRunning() && est_hdg_->isRunning()) {
        ROS_INFO_THROTTLE(1.0, "[%s]: Subestimators converged", getPrintName().c_str());
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
      if (est_lat_->isReady() && est_alt_->isReady() && est_hdg_->isReady()) {
        changeState(READY_STATE);
      }
      break;
    }
  }
}
/*//}*/

/* timerPubAttitude() //{*/
void StateGeneric::timerPubAttitude(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("StateGeneric::timerPubAttitude", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  if (!sh_hw_api_orient_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!est_hdg_->isRunning() && !isError()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: cannot publish attitude, heading estimator is not running", getPrintName().c_str());
    return;
  }

  scope_timer.checkpoint("checks");

  const ros::Time time_now = ros::Time::now();

  geometry_msgs::QuaternionStamped att;
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
    ROS_ERROR_THROTTLE(1.0, "[%s]: could not rotate orientation by heading", getPrintName().c_str());
    return;
  }

  scope_timer.checkpoint("rotate");

  if (!Support::noNans(att.quaternion)) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: NaNs in timerPubAttitude quaternion", getPrintName().c_str());
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

  if (!sh_hw_api_orient_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!sh_hw_api_ang_vel_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: has not received angular velocity on topic %s yet", getPrintName().c_str(), sh_hw_api_ang_vel_.topicName().c_str());
    return;
  }
  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("StateGeneric::updateUavState", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  const ros::Time time_now = ros::Time::now();

  mrs_msgs::UavState uav_state = uav_state_init_;
  uav_state.header.stamp       = time_now;

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
      ROS_ERROR_THROTTLE(1.0, "[%s]: could not rotate orientation by heading", getPrintName().c_str());
      return;
    }
  }

  scope_timer.checkpoint("rotate orientation");

  uav_state.velocity.angular = sh_hw_api_ang_vel_.getMsg()->vector;

  uav_state.pose.position.x = est_lat_->getState(POSITION, AXIS_X);
  uav_state.pose.position.y = est_lat_->getState(POSITION, AXIS_Y);
  uav_state.pose.position.z = est_alt_->getState(POSITION);

  uav_state.velocity.linear.x = est_lat_->getState(VELOCITY, AXIS_X);  // in global frame
  uav_state.velocity.linear.y = est_lat_->getState(VELOCITY, AXIS_Y);  // in global frame
  uav_state.velocity.linear.z = est_alt_->getState(VELOCITY);          // in global frame

  uav_state.acceleration.linear.x = est_lat_->getState(ACCELERATION, AXIS_X);  // in global frame
  uav_state.acceleration.linear.y = est_lat_->getState(ACCELERATION, AXIS_Y);  // in global frame
  uav_state.acceleration.linear.z = est_alt_->getState(ACCELERATION);          // in global frame

  scope_timer.checkpoint("fill uav state");

  const nav_msgs::Odometry odom = Support::uavStateToOdom(uav_state);
  scope_timer.checkpoint("uav state to odom");

  nav_msgs::Odometry innovation = innovation_init_;
  innovation.header.stamp       = time_now;

  innovation.pose.pose.position.x = est_lat_->getInnovation(POSITION, AXIS_X);
  innovation.pose.pose.position.y = est_lat_->getInnovation(POSITION, AXIS_Y);
  innovation.pose.pose.position.z = est_alt_->getInnovation(POSITION);

  is_mitigating_jump_ = est_alt_->isMitigatingJump() || est_lat_->isMitigatingJump() || est_hdg_->isMitigatingJump();

  scope_timer.checkpoint("innovation");

  mrs_msgs::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = time_now;
  twist_covariance.header.stamp = time_now;

  const int n_states = 6;  // TODO this should be defined somewhere else
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
bool StateGeneric::setUavState(const mrs_msgs::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    ROS_WARN("[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  ROS_WARN("[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

}  // namespace mrs_uav_state_estimators

