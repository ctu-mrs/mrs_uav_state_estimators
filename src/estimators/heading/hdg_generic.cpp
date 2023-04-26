#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/heading/hdg_generic.h"

//}

namespace mrs_uav_state_estimators

{

/* initialize() //{*/
void HdgGeneric::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // clang-format off
  dt_ = 0.01;
  input_coeff_ = 0.2;
  default_input_coeff_ = 0.2;

  generateA();
  generateB();

  H_ <<
    1, 0;

  // clang-format on

  // | --------------- initialize parameter loader -------------- |
  Support::loadParamFile(ros::package::getPath(package_name_) + "/config/estimators/" + getNamespacedName() + ".yaml", nh.getNamespace());

  mrs_lib::ParamLoader param_loader(nh, getPrintName());
  param_loader.setPrefix(getNamespacedName() + "/");

  // | --------------------- load parameters -------------------- |
  param_loader.loadParam("max_flight_altitude_agl", max_flight_altitude_agl_);
  param_loader.loadParam("repredictor/enabled", is_repredictor_enabled_);
  if (is_repredictor_enabled_) {
    param_loader.loadParam("repredictor/buffer_size", rep_buffer_size_);
  }

  // | --------------- corrections initialization --------------- |
  param_loader.loadParam("corrections", correction_names_);

  for (auto corr_name : correction_names_) {
    corrections_.push_back(std::make_shared<Correction<hdg_generic::n_measurements>>(
        nh, getNamespacedName(), corr_name, ns_frame_id_, EstimatorType_t::HEADING, ch_, [this](int a, int b) { return this->getState(a, b); },
        [this](const Correction<hdg_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t state) {
          return this->doCorrection(meas, R, state);
        }));
  }

  // | ----------- initialize process noise covariance ---------- |
  Q_ = Q_t::Zero();
  double tmp_noise;
  param_loader.loadParam("process_noise/pos", tmp_noise);
  Q_(POSITION, POSITION) = tmp_noise;
  param_loader.loadParam("process_noise/vel", tmp_noise);
  Q_(VELOCITY, VELOCITY) = tmp_noise;

  // | ------- check if all parameters loaded successfully ------ |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_ =
      std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), true, getPrintName(), boost::bind(&HdgGeneric::callbackReconfigure, this, _1, _2));
  drmgr_->config.pos = Q_(POSITION, POSITION);
  drmgr_->config.vel = Q_(VELOCITY, VELOCITY);
  drmgr_->update_config(drmgr_->config);

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    for (int i = 0; i < hdg_generic::n_states; i++) {
      H_t H = H_t::Zero();
      H(i)  = 1;
      models_.push_back(std::make_shared<lkf_t>(A_, B_, H));
    }

    const u_t       u0 = u_t::Zero();
    const ros::Time t0 = ros::Time::now();
    lkf_rep_           = std::make_unique<mrs_lib::Repredictor<lkf_t>>(x0, P0, u0, Q_, t0, lkf_, rep_buffer_size_);

    setDt(1.0 / ch_->desired_uav_state_rate);
  }

  // | ------------------ timers initialization ----------------- |
  timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &HdgGeneric::timerUpdate, this);  // not running after init
  /* timer_check_health_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &HdgGeneric::timerCheckHealth, this); */

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_input_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput>(shopts, "control_input_in");

  // | ---------------- publishers initialization --------------- |
  if (ch_->debug_topics.input) {
    ph_input_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, getNamespacedName() + "/input", 10);
  }
  if (ch_->debug_topics.output) {
    ph_output_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorOutput>(nh, getNamespacedName() + "/output", 10);
  }
  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics>(nh, getNamespacedName() + "/diagnostics", 10);
  }

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getPrintName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool HdgGeneric::start(void) {

  if (isInState(READY_STATE)) {
    /* timer_update_.start(); */
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
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
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
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

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void HdgGeneric::timerUpdate(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for initialization", getPrintName().c_str());
      break;
    }

    case READY_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for estimator start", getPrintName().c_str());
      break;
    }

    case INITIALIZED_STATE: {
      // initialize the estimator with current corrections
      for (auto correction : corrections_) {
        auto res = correction->getProcessedCorrection();
        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(AXIS_X), correction->getStateId(), AXIS_X);
          ROS_INFO_THROTTLE(1.0, "[%s]: Setting initial state to: %.2f", getPrintName().c_str(), measurement_stamped.value(AXIS_X));
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for correction %s", getPrintName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      ROS_INFO_THROTTLE(1.0, "[%s]: Ready to start", getPrintName().c_str());
      changeState(READY_STATE);
      break;
    }

    case STARTED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for convergence of LKF", getPrintName().c_str());
      if (isConverged()) {
        ROS_INFO_THROTTLE(1.0, "[%s]: LKF converged", getPrintName().c_str());
        changeState(RUNNING_STATE);
      }
      break;
    }

    case RUNNING_STATE: {
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getPrintName().c_str(), correction->getNamespacedName().c_str());
          changeState(ERROR_STATE);
        }
      }
      break;
    }

    case STOPPED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is stopped", getPrintName().c_str());
      break;
    }

    case ERROR_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is in ERROR state", getPrintName().c_str());
      bool all_corrections_healthy = true;
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getPrintName().c_str(), correction->getNamespacedName().c_str());
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

  if (sh_control_input_.newMsg()) {
    is_input_ready_ = true;
  }

  // check age of input
  if (is_input_ready_ && (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec() > 0.1) {
    ROS_WARN_THROTTLE(1.0, "[%s]: input too old (%.4f), using zero input instead", getPrintName().c_str(),
                      (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec());
    is_input_ready_ = false;
  }

  if (!isRunning() && !isStarted()) {
    return;
  }

  if (first_iter_) {
    first_iter_ = false;
    return;
  }

  double dt = (event.current_real - event.last_real).toSec();
  if (dt <= 0.0) {
    return;
  }

  /* if (!is_repredictor_enabled_) { // repredictor requires constant dt */
  /*   setDt(dt); */
  /* } */

  // go through available corrections and apply them
  /* for (auto correction : corrections_) { */
  /*   auto res = correction->getProcessedCorrection(); */
  /*   if (res) { */
  /*     auto measurement_stamped = res.value(); */
  /*     doCorrection(measurement_stamped.value, correction->getR(), correction->getStateId(), measurement_stamped.stamp); */
  /*   } */
  /* } */

  // prediction step
  u_t       u;
  ros::Time input_stamp;
  if (is_input_ready_) {
    input_stamp = sh_control_input_.getMsg()->header.stamp;
    setInputCoeff(default_input_coeff_);
    u(0) = sh_control_input_.getMsg()->control_hdg_rate;
  } else {
    input_stamp = ros::Time::now();
    setInputCoeff(0);
    u = u_t::Zero();
  }

  statecov_t sc = mrs_lib::get_mutexed(mutex_sc_, sc_);
  Q_t        Q  = mrs_lib::get_mutexed(mtx_Q_, Q_);
  try {
    // Apply the prediction step
    std::scoped_lock lock(mutex_lkf_);
    if (is_repredictor_enabled_) {
      lkf_rep_->addInputChangeWithNoise(u, Q, input_stamp, lkf_);
      sc = lkf_rep_->predictTo(ros::Time::now());
    } else {
      sc = lkf_->predict(sc, u, Q, dt_);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF prediction failed: %s", getPrintName().c_str(), e.what());
  }

  mrs_lib::set_mutexed(mutex_sc_, sc, sc_);

  if (!isError()) {
    mrs_lib::set_mutexed(mutex_last_valid_hdg_, sc.x(POSITION), last_valid_hdg_);
  }

  // publishing
  publishInput(u, input_stamp);
  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void HdgGeneric::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for initialization", getPrintName().c_str());
      break;
    }

    case READY_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for estimator start", getPrintName().c_str());
      break;
    }

    case INITIALIZED_STATE: {
      // initialize the estimator with current corrections
      for (auto correction : corrections_) {
        auto res = correction->getProcessedCorrection();
        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(AXIS_X), correction->getStateId(), AXIS_X);
          ROS_INFO_THROTTLE(1.0, "[%s]: Setting initial state to: %.2f", getPrintName().c_str(), measurement_stamped.value(AXIS_X));
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for correction %s", getPrintName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      ROS_INFO_THROTTLE(1.0, "[%s]: Ready to start", getPrintName().c_str());
      changeState(READY_STATE);
      break;
    }

    case STARTED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for convergence of LKF", getPrintName().c_str());
      if (isConverged()) {
        ROS_INFO_THROTTLE(1.0, "[%s]: LKF converged", getPrintName().c_str());
        changeState(RUNNING_STATE);
      }
      break;
    }

    case RUNNING_STATE: {
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getPrintName().c_str(), correction->getNamespacedName().c_str());
          changeState(ERROR_STATE);
        }
      }
      break;
    }

    case STOPPED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is stopped", getPrintName().c_str());
      break;
    }

    case ERROR_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is in ERROR state", getPrintName().c_str());
      bool all_corrections_healthy = true;
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          ROS_ERROR_THROTTLE(1.0, "[%s]: Correction %s is not healthy!", getPrintName().c_str(), correction->getNamespacedName().c_str());
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

  if (sh_control_input_.newMsg()) {
    is_input_ready_ = true;
  }

  // check age of input
  if (is_input_ready_ && (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec() > 0.1) {
    ROS_WARN_THROTTLE(1.0, "[%s]: input too old (%.4f), using zero input instead", getPrintName().c_str(),
                      (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec());
    is_input_ready_ = false;
  }
}
/*//}*/

/*//{ doCorrection() */
void HdgGeneric::doCorrection(const Correction<hdg_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t &state_id) {
  doCorrection(meas.value, R, state_id, meas.stamp);
}
/*//}*/

/*//{ doCorrection() */
void HdgGeneric::doCorrection(const z_t &z, const double R, const StateId_t &H_idx, const ros::Time &meas_stamp) {

  if (!isInitialized()) {
    return;
  }

  // for position state check the innovation
  if (H_idx == POSITION) {
    std::scoped_lock lock(mtx_innovation_);

    innovation_(0) = mrs_lib::geometry::radians::dist(mrs_lib::geometry::radians(z(0)), mrs_lib::geometry::radians(getState(POSITION)));

    if (innovation_(0) > 1.0 || innovation_(0) < -1.0) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - hdg: %.2f", getPrintName().c_str(), innovation_(0));
    }
  }

  statecov_t sc = mrs_lib::get_mutexed(mutex_sc_, sc_);
  try {
    // Apply the correction step
    {
      std::scoped_lock lock(mutex_lkf_);
      H_        = H_t::Zero();
      H_(H_idx) = 1;
      lkf_->H   = H_;
      if (is_repredictor_enabled_) {

        lkf_rep_->addMeasurement(z, R_t::Ones() * R, meas_stamp, models_[H_idx]);
      } else {
        sc = lkf_->correct(sc, z, R_t::Ones() * R);
      }
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getPrintName().c_str(), e.what());
  }

  mrs_lib::set_mutexed(mutex_sc_, sc, sc_);
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
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void HdgGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, 0));
}

void HdgGeneric::setState(const double &state_in, const int &state_idx_in) {
  mrs_lib::set_mutexed(mutex_sc_, state_in, sc_.x(state_idx_in));
}
/*//}*/

/*//{ getStates() */
HdgGeneric::states_t HdgGeneric::getStates(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x;
}
/*//}*/

/*//{ setStates() */
void HdgGeneric::setStates(const states_t &states_in) {
  mrs_lib::set_mutexed(mutex_sc_, states_in, sc_.x);
}
/*//}*/

/*//{ getCovariance() */
double HdgGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {
  return getCovariance(stateIdToIndex(state_id_in, 0));
}

double HdgGeneric::getCovariance(const int &state_idx_in) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
HdgGeneric::covariance_t HdgGeneric::getCovarianceMatrix(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void HdgGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  mrs_lib::set_mutexed(mutex_sc_, cov_in, sc_.P);
}
/*//}*/

/*//{ getInnovation() */
double HdgGeneric::getInnovation(const int &state_idx) const {
  return mrs_lib::get_mutexed(mtx_innovation_, innovation_)(state_idx);
}

double HdgGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(0, 0));
}
/*//}*/

/*//{ setDt() */
void HdgGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
}
/*//}*/

/*//{ setInputCoeff() */
void HdgGeneric::setInputCoeff(const double &input_coeff) {
  input_coeff_ = input_coeff;
  generateA();
  generateB();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
  lkf_->B = B_;
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

/*//{ getLastValidHdg() */
double HdgGeneric::getLastValidHdg() const {
  return mrs_lib::get_mutexed(mutex_last_valid_hdg_, last_valid_hdg_);
}
/*//}*/

/*//{ callbackReconfigure() */
void HdgGeneric::callbackReconfigure(HeadingEstimatorConfig &config, [[maybe_unused]] uint32_t level) {

  if (!isInitialized()) {
    return;
  }

  Q_t Q;
  Q(POSITION, POSITION) = config.pos;
  Q(VELOCITY, VELOCITY) = config.vel;
  mrs_lib::set_mutexed(mtx_Q_, Q, Q_);
}
/*//}*/

/*//{ getNamespacedName() */
std::string HdgGeneric::getNamespacedName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

/*//{ getPrintName() */
std::string HdgGeneric::getPrintName() const {
  return ch_->nodelet_name + "/" + parent_state_est_name_ + "/" + getName();
}
/*//}*/

};  // namespace mrs_uav_state_estimators
