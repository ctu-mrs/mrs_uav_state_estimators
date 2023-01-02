#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/heading/hdg_passthrough.h"

//}

namespace mrs_uav_state_estimation

{

/* initialize() //{*/
void HdgPassthrough::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  {
    std::scoped_lock lock(mtx_hdg_state_);
    hdg_state_      = states_t::Zero();
    hdg_covariance_ = covariance_t::Zero();
  }

  // | --------------- param loader initialization --------------- |
  Support::loadParamFile(ros::package::getPath(ch_->package_name) + "/config/estimators/" + getNamespacedName() + ".yaml", nh.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getNamespacedName());
  param_loader.setPrefix(getNamespacedName() + "/");

  // | --------------------- load parameters -------------------- |
  param_loader.loadParam("max_flight_altitude_agl", max_flight_altitude_agl_);

  param_loader.loadParam("message/topic", odom_topic_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getNamespacedName().c_str());
    ros::shutdown();
  }

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                               // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &HdgPassthrough::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                                 // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &HdgPassthrough::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getNamespacedName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + ch_->uav_name + "/" + odom_topic_);

  // | ---------------- publishers initialization --------------- |
  ph_output_      = mrs_lib::PublisherHandler<EstimatorOutput>(nh, getNamespacedName() + "/output", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<EstimatorDiagnostics>(nh, getNamespacedName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getNamespacedName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getNamespacedName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool HdgPassthrough::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getNamespacedName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool HdgPassthrough::pause(void) {

  if (isInState(RUNNING_STATE)) {
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
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getNamespacedName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getNamespacedName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void HdgPassthrough::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  auto msg = sh_odom_.getMsg();
  double   hdg      = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
  double   hdg_rate = 0;
  auto     res      = getHeadingRate(msg);
  if (res) {
    hdg_rate = res.value();
  } else {
    ROS_ERROR("[%s]: could not get heading rate", getNamespacedName().c_str());
  }

  {
    std::scoped_lock lock(mtx_innovation_);

    innovation_(0) = hdg - getState(POSITION);
    innovation_(1) = hdg_rate - getState(VELOCITY);

    if (innovation_(0) > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - hdg: %.2f", getNamespacedName().c_str(), innovation_(0));
    }
    if (innovation_(1) > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - hdg_rate: %.2f", getNamespacedName().c_str(), innovation_(1));
    }
  }

  setState(hdg, 0);
  setState(hdg_rate, 1);

  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void HdgPassthrough::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    changeState(READY_STATE);
    ROS_INFO("[%s]: Ready to start", getNamespacedName().c_str());
  }

  if (isInState(STARTED_STATE)) {

    ROS_INFO("[%s]: Estimator Running", getNamespacedName().c_str());
    changeState(RUNNING_STATE);
  }

  if (sh_odom_.hasMsg()) {
    is_odom_ready_ = true;
  }
}
/*//}*/

/*//{ getState() */
double HdgPassthrough::getState(const int &state_id_in, const int &axis_in) const {

  return getState(stateIdToIndex(state_id_in, 0));
}

double HdgPassthrough::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mtx_hdg_state_);
  return hdg_state_(state_idx_in);
}
/*//}*/

/*//{ setState() */
void HdgPassthrough::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, 0));
}

void HdgPassthrough::setState(const double &state_in, const int &state_idx_in) {
  std::scoped_lock lock(mtx_hdg_state_);
  hdg_state_(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
HdgPassthrough::states_t HdgPassthrough::getStates(void) const {
  std::scoped_lock lock(mtx_hdg_state_);
  return hdg_state_;
}
/*//}*/

/*//{ setStates() */
void HdgPassthrough::setStates(const states_t &states_in) {
  std::scoped_lock lock(mtx_hdg_state_);
  hdg_state_ = states_in;
}
/*//}*/

/*//{ getCovariance() */
double HdgPassthrough::getCovariance(const int &state_id_in, const int &axis_in) const {

  return getCovariance(stateIdToIndex(state_id_in, 0));
}

double HdgPassthrough::getCovariance(const int &state_idx_in) const {
  std::scoped_lock lock(mtx_hdg_covariance_);
  return hdg_covariance_(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
HdgPassthrough::covariance_t HdgPassthrough::getCovarianceMatrix(void) const {
  std::scoped_lock lock(mtx_hdg_covariance_);
  return hdg_covariance_;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void HdgPassthrough::setCovarianceMatrix(const covariance_t &cov_in) {
  std::scoped_lock lock(mtx_hdg_covariance_);
  hdg_covariance_ = cov_in;
}
/*//}*/

/*//{ getInnovation() */
double HdgPassthrough::getInnovation(const int &state_idx) const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_(state_idx);
}

double HdgPassthrough::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(state_id_in, 0));
}
/*//}*/

/*//{ getNamespacedName() */
  std::string HdgPassthrough::getNamespacedName() const {
    return parent_state_est_name_ + "/" + getName();
  }
/*//}*/

};  // namespace mrs_uav_state_estimation
