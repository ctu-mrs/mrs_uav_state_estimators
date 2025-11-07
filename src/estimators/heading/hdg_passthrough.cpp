#define VERSION "0.0.6.0"

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/heading/hdg_passthrough.h>

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
void HdgPassthrough::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch,
                                const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  hdg_state_      = states_t::Zero();
  hdg_covariance_ = covariance_t::Zero();

  innovation_ << 0, 0;

  // | --------------- param loader initialization --------------- |

  if (is_core_plugin_) {

    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + parent_state_est_name_ + "/" + getName() +
                                  ".yaml");
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + parent_state_est_name_ + "/" + getName() +
                                  ".yaml");
  }

  // | --------------------- load parameters -------------------- |
  ph->param_loader->loadParam("max_flight_z", max_flight_z_);

  ph->param_loader->loadParam("topics/orientation", orient_topic_);
  ph->param_loader->loadParam("topics/angular_velocity", ang_vel_topic_);

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
  }

  // | ------------------ timers initialization ----------------- |

  {

    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = false;

    std::function<void()> callback_fcn = std::bind(&HdgPassthrough::timerUpdate, this);

    timer_update_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);

    timer_update_last_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = true;

    std::function<void()> callback_fcn = std::bind(&HdgPassthrough::timerCheckHealth, this);

    timer_check_health_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);
  }

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node               = node_;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  sh_orientation_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, "/" + ch_->uav_name + "/" + orient_topic_,
                                                                                      &HdgPassthrough::callbackOrientation, this);

  sh_ang_vel_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped>(shopts, "/" + ch_->uav_name + "/" + ang_vel_topic_,
                                                                               &HdgPassthrough::callbackAngularVelocity, this);

  // | ---------------- publishers initialization --------------- |
  if (ch_->debug_topics.output) {
    ph_output_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorOutput>(node_, "~/" + getNamespacedName() + "/output");
  }
  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics>(node_, "~/" + getNamespacedName() + "/diagnostics");
  }

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized, version %s", getPrintName().c_str(), VERSION);
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool HdgPassthrough::start(void) {

  if (isInState(READY_STATE)) {

    timer_update_->start();
    timer_check_health_->start();

    changeState(STARTED_STATE);

    return true;

  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool HdgPassthrough::pause(void) {

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
bool HdgPassthrough::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);
  // nothing to do during reset of passthrough estimator
  changeState(INITIALIZED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/*//{ callbackOrientation() */
void HdgPassthrough::callbackOrientation(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg) {

  if (!isInitialized()) {
    return;
  }

  double hdg;
  try {
    hdg = mrs_lib::AttitudeConverter(msg->quaternion).getHeading();
  }
  catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: failed getting heading", getPrintName().c_str());
  }

  setState(hdg, POSITION);

  if (!isError()) {
    mrs_lib::set_mutexed(mutex_last_valid_hdg_, hdg, last_valid_hdg_);
  }
}
/*//}*/

/*//{ callbackAngularVelocity() */
void HdgPassthrough::callbackAngularVelocity(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {

  if (!isInitialized() || !sh_orientation_.hasMsg()) {
    return;
  }

  double hdg_rate;
  try {
    hdg_rate = mrs_lib::AttitudeConverter(sh_orientation_.getMsg()->quaternion).getHeadingRate(msg->vector);
  }
  catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: failed getting heading", getPrintName().c_str());
  }

  setState(hdg_rate, VELOCITY);
}
/*//}*/

/* timerUpdate() //{*/
void HdgPassthrough::timerUpdate() {


  if (!isInitialized()) {
    return;
  }

  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void HdgPassthrough::timerCheckHealth() {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE) && is_orient_ready_ && is_ang_vel_ready_) {

    changeState(READY_STATE);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Ready to start", getPrintName().c_str());
  }

  if (isInState(STARTED_STATE)) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator Running", getPrintName().c_str());
    changeState(RUNNING_STATE);
  }

  if (sh_orientation_.hasMsg()) {
    is_orient_ready_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received orientation yet", getPrintName().c_str());
  }

  if (sh_ang_vel_.hasMsg()) {
    is_ang_vel_ready_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received angular velocity yet", getPrintName().c_str());
  }
}
/*//}*/

/*//{ getState() */
double HdgPassthrough::getState(const int &state_id_in, const int &axis_in) const {
  return getState(stateIdToIndex(state_id_in, axis_in));
}

double HdgPassthrough::getState(const int &state_idx_in) const {
  return mrs_lib::get_mutexed(mtx_hdg_state_, hdg_state_(state_idx_in));
}
/*//}*/

/*//{ setState() */
void HdgPassthrough::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, axis_in));
}

void HdgPassthrough::setState(const double &state_in, const int &state_idx_in) {

  const double prev_hdg_state   = mrs_lib::get_mutexed(mtx_hdg_state_, hdg_state_(state_idx_in));
  prev_hdg_state_(state_idx_in) = prev_hdg_state;
  mrs_lib::set_mutexed(mtx_hdg_state_, state_in, hdg_state_(state_idx_in));

  const double innovation = mrs_lib::geometry::radians::dist(mrs_lib::geometry::radians(state_in), mrs_lib::geometry::radians(prev_hdg_state));
  mrs_lib::set_mutexed(mtx_innovation_, innovation, innovation_(state_idx_in));
}
/*//}*/

/*//{ getStates() */
HdgPassthrough::states_t HdgPassthrough::getStates(void) const {
  return mrs_lib::get_mutexed(mtx_hdg_state_, hdg_state_);
}
/*//}*/

/*//{ setStates() */
void HdgPassthrough::setStates(const states_t &states_in) {
  mrs_lib::set_mutexed(mtx_hdg_state_, states_in, hdg_state_);
}
/*//}*/

/*//{ getCovariance() */
double HdgPassthrough::getCovariance(const int &state_id_in, const int &axis_in) const {
  return getCovariance(stateIdToIndex(state_id_in, axis_in));
}

double HdgPassthrough::getCovariance(const int &state_idx_in) const {
  return mrs_lib::get_mutexed(mtx_hdg_covariance_, hdg_covariance_(state_idx_in, state_idx_in));
}
/*//}*/

/*//{ getCovarianceMatrix() */
HdgPassthrough::covariance_t HdgPassthrough::getCovarianceMatrix(void) const {
  return mrs_lib::get_mutexed(mtx_hdg_covariance_, hdg_covariance_);
}
/*//}*/

/*//{ setCovarianceMatrix() */
void HdgPassthrough::setCovarianceMatrix(const covariance_t &cov_in) {
  mrs_lib::set_mutexed(mtx_hdg_covariance_, cov_in, hdg_covariance_);
}
/*//}*/

/*//{ getInnovation() */
double HdgPassthrough::getInnovation(const int &state_idx) const {
  return mrs_lib::get_mutexed(mtx_innovation_, innovation_(state_idx));
}

double HdgPassthrough::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(state_id_in, axis_in));
}
/*//}*/

/*//{ getLastValidHdg() */
double HdgPassthrough::getLastValidHdg() const {
  return mrs_lib::get_mutexed(mutex_last_valid_hdg_, last_valid_hdg_);
}
/*//}*/

/*//{ getNamespacedName() */
std::string HdgPassthrough::getNamespacedName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

/*//{ getPrintName() */
std::string HdgPassthrough::getPrintName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

}; // namespace mrs_uav_state_estimators
