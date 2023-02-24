#define VERSION "0.0.0.1"

/* includes //{ */

#include "estimators/lateral/lat_generic.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void LatGeneric::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // clang-format off
  dt_ = 0.01;
  input_coeff_ = 0.1;
  default_input_coeff_ = 0.1;

  generateA();
  generateB();

  H_ <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0;

  // clang-format on

  // | --------------- initialize parameter loader -------------- |
  Support::loadParamFile(ros::package::getPath(ch_->package_name) + "/config/estimators/" + getNamespacedName() + ".yaml", nh.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getNamespacedName());
  param_loader.setPrefix(getNamespacedName() + "/");

  // | --------------------- load parameters -------------------- |
  param_loader.loadParam("hdg_source_topic", hdg_source_topic_);
  param_loader.loadParam("max_flight_altitude_agl", max_flight_altitude_agl_);
  param_loader.loadParam("repredictor/enabled", is_repredictor_enabled_);
  if (is_repredictor_enabled_) {
    param_loader.loadParam("repredictor/buffer_size", rep_buffer_size_);
  }

  // | --------------- corrections initialization --------------- |
  param_loader.loadParam("corrections", correction_names_);

  for (auto corr_name : correction_names_) {
    corrections_.push_back(std::make_shared<Correction<lat_generic::n_measurements>>(nh, getNamespacedName(), corr_name, ns_frame_id_, EstimatorType_t::LATERAL,
                                                                                     ch_, [this](int a, int b) { return this->getState(a, b); }));
  }

  // | ------- check if all parameters loaded successfully ------ |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getNamespacedName().c_str());
    ros::shutdown();
  }

  // | ----------- initialize process noise covariance ---------- |
  Q_ = Q_t::Zero();
  double tmp_noise;
  param_loader.loadParam("process_noise/pos", tmp_noise);
  Q_(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(POSITION, AXIS_Y), stateIdToIndex(POSITION, AXIS_Y)) = tmp_noise;
  param_loader.loadParam("process_noise/vel", tmp_noise);
  Q_(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(VELOCITY, AXIS_Y), stateIdToIndex(VELOCITY, AXIS_Y)) = tmp_noise;
  param_loader.loadParam("process_noise/acc", tmp_noise);
  Q_(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(ACCELERATION, AXIS_Y), stateIdToIndex(ACCELERATION, AXIS_Y)) = tmp_noise;

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_             = std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), getNamespacedName());
  drmgr_->config.pos = Q_(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X));
  drmgr_->config.vel = Q_(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X));
  drmgr_->config.acc = Q_(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X));
  drmgr_->update_config(drmgr_->config);

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    for (int i = 0; i < lat_generic::n_states; i++) {
      H_t H                = H_t::Zero();
      H(AXIS_X, i * 2)     = 1;
      H(AXIS_Y, i * 2 + 1) = 1;
      models_.push_back(std::make_shared<lkf_t>(A_, B_, H));
    }

    const u_t       u0 = u_t::Zero();
    const ros::Time t0 = ros::Time::now();
    lkf_rep_           = std::make_unique<mrs_lib::Repredictor<lkf_t>>(x0, P0, u0, Q_, t0, lkf_, rep_buffer_size_);
  }

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                           // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &LatGeneric::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                             // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &LatGeneric::timerCheckHealth, this);

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

  sh_attitude_command_ = mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>(shopts, "attitude_command_in");
  sh_hdg_state_        = mrs_lib::SubscribeHandler<mrs_uav_state_estimation::EstimatorOutput>(
      shopts, hdg_source_topic_);  // for transformation of desired accelerations from body to global frame

  // | ---------------- publishers initialization --------------- |
  ph_input_       = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, getNamespacedName() + "/input", 1);
  ph_output_      = mrs_lib::PublisherHandler<mrs_uav_state_estimation::EstimatorOutput>(nh, getNamespacedName() + "/output", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_uav_state_estimation::EstimatorDiagnostics>(nh, getNamespacedName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getNamespacedName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getNamespacedName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool LatGeneric::start(void) {

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
bool LatGeneric::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool LatGeneric::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getNamespacedName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  // Initialize LKF state and covariance
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  // Instantiate the LKF itself
  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    const u_t       u0 = u_t::Zero();
    const ros::Time t0 = ros::Time(0);
    lkf_rep_           = std::make_unique<mrs_lib::Repredictor<lkf_t>>(x0, P0, u0, Q_, t0, lkf_, rep_buffer_size_);
  }

  ROS_INFO("[%s]: Estimator reset", getNamespacedName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void LatGeneric::timerUpdate(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (first_iter_) {
    first_iter_ = false;
    return;
  }

  // prediction step
  u_t       u;
  ros::Time input_stamp;
  if (is_input_ready_ && is_hdg_state_ready_) {
    const tf2::Vector3 des_acc_global = getAccGlobal(sh_attitude_command_.getMsg(), sh_hdg_state_.getMsg()->state[0]);
    input_stamp                       = sh_attitude_command_.getMsg()->header.stamp;
    setInputCoeff(default_input_coeff_);
    u(0) = des_acc_global.getX();
    u(1) = des_acc_global.getY();
  } else {
    input_stamp = ros::Time::now();
    setInputCoeff(0);
    u = u_t::Zero();
  }

  setDt((event.current_real - event.last_real).toSec());

  try {
    // Apply the prediction step
    std::scoped_lock lock(mutex_lkf_);
    if (is_repredictor_enabled_) {
      lkf_rep_->addInputChangeWithNoise(u, Q_, input_stamp, lkf_);
      /* sc_ = lkf_rep_->predictTo(ros::Time::now()); */
    } else {
      sc_ = lkf_->predict(sc_, u, getQ(), dt_);
    }
  }
  catch (const std::exception &e) {
    ROS_ERROR("[%s]: LKF prediction failed: %s", getNamespacedName().c_str(), e.what());
    changeState(ERROR_STATE);
  }

  // go through available corrections and apply them
  for (auto correction : corrections_) {
    auto res = correction->getProcessedCorrection();
    if (res) {
      auto measurement_stamped = res.value();
      doCorrection(measurement_stamped.value, correction->getR(), correction->getStateId(), measurement_stamped.stamp);
    }
  }

  // publishing
  publishInput(u);
  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void LatGeneric::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for initialization", getNamespacedName().c_str());
      break;
    }

    case READY_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for estimator start", getNamespacedName().c_str());
      break;
    }

    case INITIALIZED_STATE: {
      // initialize the estimator with current corrections
      for (auto correction : corrections_) {
        auto res = correction->getRawCorrection();
        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(AXIS_X), correction->getStateId(), AXIS_X);
          setState(measurement_stamped.value(AXIS_Y), correction->getStateId(), AXIS_Y);
          ROS_INFO("[%s]: Setting initial state to: %.2f %.2f", getNamespacedName().c_str(), measurement_stamped.value(AXIS_X),
                   measurement_stamped.value(AXIS_Y));
        } else {
          ROS_INFO("[%s]: Waiting for correction %s", getNamespacedName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      ROS_INFO("[%s]: Ready to start", getNamespacedName().c_str());
      changeState(READY_STATE);
      break;
    }

    case STARTED_STATE: {
      ROS_INFO("[%s]: Waiting for convergence of LKF", getNamespacedName().c_str());
      if (isConverged()) {
        ROS_INFO("[%s]: LKF converged", getNamespacedName().c_str());
        changeState(RUNNING_STATE);
      }
      break;
    }

    case RUNNING_STATE: {
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getNamespacedName().c_str(), correction->getNamespacedName().c_str());
          changeState(ERROR_STATE);
        }
      }
      break;
    }

    case STOPPED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is stopped", getNamespacedName().c_str());
      break;
    }

    case ERROR_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is in ERROR state", getNamespacedName().c_str());
      bool all_corrections_healthy = true;
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getNamespacedName().c_str(), correction->getNamespacedName().c_str());
          all_corrections_healthy = false;
        }
      }
      // initialize the estimator again if corrections become healthy
      if (all_corrections_healthy) {
        changeState(INITIALIZED_STATE);
      }
      break;
    }
  }

  if (sh_attitude_command_.newMsg()) {
    is_input_ready_ = true;
  }

  // check age of input
  if (is_input_ready_ && (ros::Time::now() - sh_attitude_command_.lastMsgTime()).toSec() > 0.1) {
    ROS_WARN("[%s]: input too old (%.4f s), using zero input instead", getNamespacedName().c_str(),
             (ros::Time::now() - sh_attitude_command_.lastMsgTime()).toSec());
    is_input_ready_ = false;
  }

  if (sh_hdg_state_.newMsg()) {
    is_hdg_state_ready_ = true;
  }


  // check age of heading
  if (is_hdg_state_ready_ && (ros::Time::now() - sh_hdg_state_.lastMsgTime()).toSec() > 0.1) {
    ROS_WARN("[%s]: hdg state too old (%.4f s), using zero input", getNamespacedName().c_str(), (ros::Time::now() - sh_hdg_state_.lastMsgTime()).toSec());
    is_hdg_state_ready_ = false;
  }
}
/*//}*/

/*//{ doCorrection() */
void LatGeneric::doCorrection(const z_t &z, const double R, const StateId_t &state_id, const ros::Time &meas_stamp) {

  // for position state check the innovation
  if (state_id == POSITION) {
    {
      std::scoped_lock lock(mtx_innovation_);

      innovation_(0) = z(0) - getState(POSITION, AXIS_X);
      innovation_(1) = z(1) - getState(POSITION, AXIS_Y);

      if (innovation_(0) > 1.0 || innovation_(0) < -1.0) {
        ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - x: %.2f", getNamespacedName().c_str(), innovation_(0));
      }
      if (innovation_(1) > 1.0 || innovation_(1) < -1.0) {
        ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - y: %.2f", getNamespacedName().c_str(), innovation_(1));
      }
    }
  }

  try {
    // Apply the correction step
    std::scoped_lock lock(mutex_lkf_);
    H_                           = H_t::Zero();
    H_(AXIS_X, state_id * 2)     = 1;
    H_(AXIS_Y, state_id * 2 + 1) = 1;
    lkf_->H                      = H_;

    if (is_repredictor_enabled_) {

      lkf_rep_->addMeasurement(z, R_t::Ones() * R, meas_stamp, models_[state_id]);
      sc_ = lkf_rep_->predictTo(ros::Time::now());
    } else {
      sc_ = lkf_->correct(sc_, z, R_t::Ones() * R);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getNamespacedName().c_str(), e.what());
  }
}
/*//}*/

/*//{ isConverged() */
bool LatGeneric::isConverged() {

  // TODO: check convergence by rate of change of determinant

  return true;
}
/*//}*/

/*//{ getState() */
double LatGeneric::getState(const int &state_id_in, const int &axis_in) const {

  return getState(stateIdToIndex(state_id_in, axis_in));
}

double LatGeneric::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void LatGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, axis_in));
}

void LatGeneric::setState(const double &state_in, const int &state_idx_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
LatGeneric::states_t LatGeneric::getStates(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.x;
}
/*//}*/

/*//{ setStates() */
void LatGeneric::setStates(const states_t &states_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x = states_in;
}
/*//}*/

/*//{ getCovariance() */
double LatGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {

  return getCovariance(stateIdToIndex(state_id_in, axis_in));
}

double LatGeneric::getCovariance(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
LatGeneric::covariance_t LatGeneric::getCovarianceMatrix(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void LatGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.P = cov_in;
}
/*//}*/

/*//{ getInnovation() */
double LatGeneric::getInnovation(const int &state_idx) const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_(state_idx);
}

double LatGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(state_id_in, axis_in));
}
/*//}*/

/*//{ setDt() */
void LatGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
}
/*//}*/

/*//{ setInputCoeff() */
void LatGeneric::setInputCoeff(const double &input_coeff) {
  input_coeff_ = input_coeff;
  generateA();
  generateB();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
  lkf_->B = B_;
}
/*//}*/

/*//{ generateA() */
void LatGeneric::generateA() {

  // clang-format off
    A_ <<
      1, 0, dt_, 0, std::pow(dt_, 2)/2, 0,
      0, 1, 0, dt_, 0, std::pow(dt_, 2)/2,
      0, 0, 1, 0, dt_, 0,
      0, 0, 0, 1, 0, dt_,
      0, 0, 0, 0, 1-input_coeff_, 0,
      0, 0, 0, 0, 0, 1-input_coeff_;
  // clang-format on
}
/*//}*/

/*//{ generateB() */
void LatGeneric::generateB() {

  // clang-format off
    B_ <<
      0, 0,
      0, 0,
      0, 0,
      0, 0,
      input_coeff_, 0,
      0, input_coeff_;
  // clang-format on
}
/*//}*/

/*//{ getQ() */
LatGeneric::Q_t LatGeneric::getQ() {
  std::scoped_lock lock(mtx_Q_);
  Q_(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X))         = drmgr_->config.pos;
  Q_(stateIdToIndex(POSITION, AXIS_Y), stateIdToIndex(POSITION, AXIS_Y))         = drmgr_->config.pos;
  Q_(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X))         = drmgr_->config.vel;
  Q_(stateIdToIndex(VELOCITY, AXIS_Y), stateIdToIndex(VELOCITY, AXIS_Y))         = drmgr_->config.vel;
  Q_(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X)) = drmgr_->config.acc;
  Q_(stateIdToIndex(ACCELERATION, AXIS_Y), stateIdToIndex(ACCELERATION, AXIS_Y)) = drmgr_->config.acc;
  return Q_;
}
/*//}*/

/*//{ getNamespacedName() */
std::string LatGeneric::getNamespacedName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

};  // namespace mrs_uav_state_estimation
