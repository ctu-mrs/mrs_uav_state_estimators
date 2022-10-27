#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/heading/hdg_generic.h"

//}

namespace mrs_uav_state_estimation

{

/* initialize() //{*/
void HdgGeneric::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // TODO load parameters

  // clang-format off
    dt_ = 0.01;
    input_coeff_ = 0.1;
    default_input_coeff_ = 0.1;

    generateA();
    generateB();

    H_ <<
      1, 0;

    Q_ <<
      1, 0,
      0, 1;

    R_ <<
      0.1;

  // clang-format on

  // | --------------- corrections initialization --------------- |
  Support::loadParamFile(ros::package::getPath(ch_->package_name) + "/config/estimators/heading/" + getName() + ".yaml", nh.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getName());
  param_loader.setPrefix(getName() + "/");
  param_loader.loadParam("corrections", correction_names_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
    ros::shutdown();
  }

  for (auto corr_name : correction_names_) {
    corrections_.push_back(std::make_shared<Correction<hdg_generic::n_measurements>>(nh, getName(), corr_name, ns_frame_id_, EstimatorType_t::HEADING, ch_));
  }

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                            // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &HdgGeneric::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                              // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &HdgGeneric::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_attitude_command_ = mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>(shopts, "attitude_command_in");

  // | ---------------- publishers initialization --------------- |
  ph_output_      = mrs_lib::PublisherHandler<EstimatorOutput>(nh, getName() + "/output", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<EstimatorDiagnostics>(nh, getName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool HdgGeneric::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool HdgGeneric::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool HdgGeneric::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  // Initialize LKF state and covariance
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  // Instantiate the LKF itself
  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  ROS_INFO("[%s]: Estimator reset", getName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void HdgGeneric::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  // prediction step
  u_t u;
  if (is_input_ready_) {

    auto res = getHeadingRate(sh_attitude_command_.getMsg());
    if (res) {
      setInputCoeff(default_input_coeff_);
      u(0) = res.value();
    } else {
      setInputCoeff(0);
      u = u_t::Zero();
    }

  } else {
    setInputCoeff(0);
    u = u_t::Zero();
  }

  setDt((event.current_real - event.last_real).toSec());

  try {
    // Apply the prediction step
    {
      std::scoped_lock lock(mutex_lkf_);
      sc_ = lkf_->predict(sc_, u, Q_, dt_);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF prediction failed: %s", getName().c_str(), e.what());
  }

  for (auto correction : corrections_) {
    z_t z;
    if (correction->getCorrection(z)) {

      // TODO processing, median filter, gating etc.
      doCorrection(z, correction->getR(), correction->getStateId());
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: correction is not valid", ros::this_node::getName().c_str());
    }
  }

  /* if (is_new_odom_message) { */

  /*   z_t z = z_t::Zero(); */

  /*   nav_msgs::Odometry::ConstPtr odom_msg = sh_odom_.getMsg(); */

  /*   /1* z(0) = odom_msg->twist.twist.linear.z; *1/ */
  /*   z(0) = odom_msg->pose.pose.position.z; */

  /*   bool health_flag = true; */

  /*   if (!std::isfinite(z(0))) { */
  /*     ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in position z correction", getName().c_str()); */
  /*     health_flag = false; */
  /*   } */

  /*   if (health_flag) { */

  /*     { */
  /*       std::scoped_lock lock(mtx_innovation_); */

  /*       innovation_(0) = z(0) - getState(POSITION); */

  /*       if (innovation_(0) > 1.0) { */
  /*         ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - z: %.2f", getName().c_str(), innovation_(0)); */
  /*       } */
  /*     } */

  /*     try { */
  /*       // Apply the correction step */
  /*       { */
  /*         std::scoped_lock lock(mutex_lkf_); */
  /*         sc_ = lkf_->correct(sc_, z, R_); */
  /*       } */
  /*     } */
  /*     catch (const std::exception &e) { */
  /*       // In case of error, alert the user */
  /*       ROS_ERROR("[%s]: LKF correction failed: %s", getName().c_str(), e.what()); */
  /*     } */
  /*   } */
  /* } */

  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void HdgGeneric::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    changeState(READY_STATE);
    ROS_INFO("[%s]: Ready to start", getName().c_str());
  }

  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getName().c_str());

    if (isConverged()) {
      ROS_INFO("[%s]: LKF converged", getName().c_str());
      changeState(RUNNING_STATE);
    }
  }

  if (sh_attitude_command_.newMsg()) {
    is_input_ready_ = true;
  }

  // check age of input
  if (is_input_ready_ && (ros::Time::now() - sh_attitude_command_.lastMsgTime()).toSec() > 0.1) {
    ROS_WARN("[%s]: input too old (%.4f), using zero input instead", getName().c_str(), (ros::Time::now() - sh_attitude_command_.lastMsgTime()).toSec());
    is_input_ready_ = false;
  }
}
/*//}*/

/*//{ doCorrection() */
void HdgGeneric::doCorrection(const z_t &z, const double R, const StateId_t &H_idx) {

  {
    std::scoped_lock lock(mtx_innovation_);

    innovation_(0) = z(0) - getState(POSITION);

    if (innovation_(0) > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - hdg: %.2f", getName().c_str(), innovation_(0));
    }
  }

  try {
    // Apply the correction step
    {
      std::scoped_lock lock(mutex_lkf_);
      H_        = H_t::Zero();
      H_(H_idx) = 1;
      lkf_->H   = H_;
      sc_       = lkf_->correct(sc_, z, R_t::Ones() * R);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getName().c_str(), e.what());
  }
}
/*//}*/

/*//{ isConverged() */
bool HdgGeneric::isConverged() {

  // TODO: check convergence by rate of change of determinant

  return true;
}
/*//}*/

/*//{ getState() */
double HdgGeneric::getState(const int &state_id_in, const int &axis_in) const {

  return getState(stateIdToIndex(state_id_in, 0));
}

double HdgGeneric::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void HdgGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, 0));
}

void HdgGeneric::setState(const double &state_in, const int &state_idx_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
HdgGeneric::states_t HdgGeneric::getStates(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.x;
}
/*//}*/

/*//{ setStates() */
void HdgGeneric::setStates(const states_t &states_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x = states_in;
}
/*//}*/

/*//{ getCovariance() */
double HdgGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {

  return getCovariance(stateIdToIndex(state_id_in, 0));
}

double HdgGeneric::getCovariance(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
HdgGeneric::covariance_t HdgGeneric::getCovarianceMatrix(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void HdgGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.P = cov_in;
}
/*//}*/

/*//{ getInnovation() */
double HdgGeneric::getInnovation(const int &state_idx) const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_(0);
}

double HdgGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(0, 0));
}
/*//}*/

/*//{ setDt() */
void HdgGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
}
/*//}*/

/*//{ setInputCoeff() */
void HdgGeneric::setInputCoeff(const double &input_coeff) {
  input_coeff_ = input_coeff;
  generateA();
  generateB();
}
/*//}*/

/*//{ generateA() */
void HdgGeneric::generateA() {

  // clang-format off
    A_ <<
      1, dt_,
      0, 1-input_coeff_;
  // clang-format on
}
/*//}*/

/*//{ generateB() */
void HdgGeneric::generateB() {

  // clang-format off
    B_ <<
      0,
      input_coeff_;
  // clang-format on
}
/*//}*/
};  // namespace mrs_uav_state_estimation
