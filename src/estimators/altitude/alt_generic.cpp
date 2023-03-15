#define VERSION "0.0.6.0"

/* includes //{ */

#include "estimators/altitude/alt_generic.h"

//}

namespace mrs_uav_state_estimators

{

/* initialize() //{*/
void AltGeneric::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // clang-format off
  dt_ = 0.01;
  input_coeff_ = 0.1;
  default_input_coeff_ = 0.1;

  generateA();
  generateB();

  H_ <<
    1, 0, 0;

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
    corrections_.push_back(std::make_shared<Correction<alt_generic::n_measurements>>(
        nh, getNamespacedName(), corr_name, ns_frame_id_, EstimatorType_t::ALTITUDE, ch_, [this](int a, int b) { return this->getState(a, b); }));
  }

  // | ----------- initialize process noise covariance ---------- |
  Q_ = Q_t::Zero();
  double tmp_noise;
  param_loader.loadParam("process_noise/pos", tmp_noise);
  Q_(POSITION, POSITION) = tmp_noise;
  param_loader.loadParam("process_noise/vel", tmp_noise);
  Q_(VELOCITY, VELOCITY) = tmp_noise;
  param_loader.loadParam("process_noise/acc", tmp_noise);
  Q_(ACCELERATION, ACCELERATION) = tmp_noise;

  // | ------- check if all parameters loaded successfully ------ |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_             = std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), getPrintName());
  drmgr_->config.pos = Q_(POSITION, POSITION);
  drmgr_->config.vel = Q_(VELOCITY, VELOCITY);
  drmgr_->config.acc = Q_(ACCELERATION, ACCELERATION);
  drmgr_->update_config(drmgr_->config);

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    for (int i = 0; i < alt_generic::n_states; i++) {
      H_t H = H_t::Zero();
      H(i)  = 1;
      models_.push_back(std::make_shared<lkf_t>(A_, B_, H));
    }

    const u_t       u0 = u_t::Zero();
    const ros::Time t0 = ros::Time::now();
    lkf_rep_           = std::make_unique<mrs_lib::Repredictor<lkf_t>>(x0, P0, u0, Q_, t0, lkf_, rep_buffer_size_);
  }

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                           // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &AltGeneric::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                             // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &AltGeneric::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_input_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput>(shopts, "control_input_in", &AltGeneric::timeoutCallback, this);

  // | ---------------- publishers initialization --------------- |
  ph_input_       = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, getNamespacedName() + "/input", 1);
  ph_output_      = mrs_lib::PublisherHandler<mrs_msgs::EstimatorOutput>(nh, getNamespacedName() + "/output", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics>(nh, getNamespacedName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getPrintName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool AltGeneric::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool AltGeneric::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool AltGeneric::reset(void) {

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
void AltGeneric::timerUpdate(const ros::TimerEvent &event) {


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
  if (is_input_ready_) {
    mrs_msgs::EstimatorInputConstPtr msg            = sh_control_input_.getMsg();
    const tf2::Vector3                 des_acc_global = getAccGlobal(msg, 0);  // we don't care about heading
    input_stamp                                       = msg->header.stamp;
    setInputCoeff(default_input_coeff_);
    u(0) = des_acc_global.getZ();
  } else {
    input_stamp = ros::Time::now();
    setInputCoeff(0);
    u = u_t::Zero();
  }

  try {
    // Apply the prediction step
    std::scoped_lock lock(mutex_lkf_);
    if (is_repredictor_enabled_) {
      lkf_rep_->addInputChangeWithNoise(u, Q_, input_stamp, lkf_);
      /* sc_ = lkf_rep_->predictTo(ros::Time::now()); */
    } else {
      /* ROS_INFO_ONCE("[%s]: before first pred %.2f", getPrintName().c_str(), sc_.x(0)); */
      sc_ = lkf_->predict(sc_, u, getQ(), dt_);
      /* ROS_INFO_ONCE("[%s]: after first pred %.2f", getPrintName().c_str(), sc_.x(0)); */
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF prediction failed: %s", getPrintName().c_str(), e.what());
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
void AltGeneric::timerCheckHealth(const ros::TimerEvent &event) {

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
        auto res = correction->getRawCorrection();
        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(0), correction->getStateId());
          ROS_INFO("[%s]: Setting initial state to: %.2f", getPrintName().c_str(), measurement_stamped.value(0));
        } else {
          ROS_INFO("[%s]: Waiting for correction %s", getPrintName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      changeState(READY_STATE);
      ROS_INFO("[%s]: Ready to start", getPrintName().c_str());
      break;
    }

    case STARTED_STATE: {
      ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getPrintName().c_str());

      if (isConverged()) {
        ROS_INFO("[%s]: LKF converged", getPrintName().c_str());
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
    ROS_WARN("[%s]: input too old (%.4f), using zero input instead", getPrintName().c_str(), (ros::Time::now() - sh_control_input_.lastMsgTime()).toSec());
    is_input_ready_ = false;
  }
}
/*//}*/

/*//{ timeoutCallback() */
void AltGeneric::timeoutCallback(const std::string &topic, const ros::Time &last_msg, const int n_pubs) {
  ROS_WARN_THROTTLE(5.0, "[%s]: Did not receive message from topic '%s' for %.2f seconds (%d publishers on topic)", getPrintName().c_str(), topic.c_str(),
                    (ros::Time::now() - last_msg).toSec(), n_pubs);
}
/*//}*/

/*//{ doCorrection() */
void AltGeneric::doCorrection(const z_t &z, const double R, const StateId_t &H_idx, const ros::Time &meas_stamp) {

  // for position state check the innovation
  if (H_idx == POSITION) {
    std::scoped_lock lock(mtx_innovation_);

    innovation_(0) = z(0) - getState(POSITION);

    if (innovation_(0) > 1.0 || innovation_(0) < -1.0) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - z: %.2f", getPrintName().c_str(), innovation_(0));
    }
  }

  try {
    // Apply the correction step
    {
      std::scoped_lock lock(mutex_lkf_);
      H_        = H_t::Zero();
      H_(H_idx) = 1;
      lkf_->H   = H_;
      if (is_repredictor_enabled_) {

        lkf_rep_->addMeasurement(z, R_t::Ones() * R, meas_stamp, models_[H_idx]);
        sc_ = lkf_rep_->predictTo(ros::Time::now());
      } else {
        sc_ = lkf_->correct(sc_, z, R_t::Ones() * R);
      }
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getPrintName().c_str(), e.what());
  }
}
/*//}*/

/*//{ isConverged() */
bool AltGeneric::isConverged() {

  // TODO: check convergence by rate of change of determinant

  return true;
}
/*//}*/

/*//{ getState() */
double AltGeneric::getState(const int &state_id_in, const int &axis_in) const {

  return getState(stateIdToIndex(state_id_in, 0));
}

double AltGeneric::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  /* ROS_INFO("[%s]: returning state[%d]: %.2f", getPrintName().c_str(), state_idx_in, sc_.x(state_idx_in)); */
  return sc_.x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void AltGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, 0));
}

void AltGeneric::setState(const double &state_in, const int &state_idx_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
AltGeneric::states_t AltGeneric::getStates(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.x;
}
/*//}*/

/*//{ setStates() */
void AltGeneric::setStates(const states_t &states_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x = states_in;
}
/*//}*/

/*//{ getCovariance() */
double AltGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {

  return getCovariance(stateIdToIndex(state_id_in, 0));
}

double AltGeneric::getCovariance(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
AltGeneric::covariance_t AltGeneric::getCovarianceMatrix(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void AltGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.P = cov_in;
}
/*//}*/

/*//{ getInnovation() */
double AltGeneric::getInnovation(const int &state_idx) const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_(0);
}

double AltGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(0, 0));
}
/*//}*/

/*//{ setDt() */
void AltGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
}
/*//}*/

/*//{ setInputCoeff() */
void AltGeneric::setInputCoeff(const double &input_coeff) {
  input_coeff_ = input_coeff;
  generateA();
  generateB();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
  lkf_->B = B_;
}
/*//}*/

/*//{ generateA() */
void AltGeneric::generateA() {

  // clang-format off
    A_ <<
      1, dt_, std::pow(dt_, 2)/2,
      0, 1, dt_,
      0, 0, 1-input_coeff_;
  // clang-format on
}
/*//}*/

/*//{ generateB() */
void AltGeneric::generateB() {

  // clang-format off
    B_ <<
      0,
      0,
      input_coeff_;
  // clang-format on
}
/*//}*/

/*//{ getQ() */
AltGeneric::Q_t AltGeneric::getQ() {
  std::scoped_lock lock(mtx_Q_);
  Q_(POSITION, POSITION)         = drmgr_->config.pos;
  Q_(VELOCITY, VELOCITY)         = drmgr_->config.vel;
  Q_(ACCELERATION, ACCELERATION) = drmgr_->config.acc;
  return Q_;
}
/*//}*/

/*//{ getNamespacedName() */
std::string AltGeneric::getNamespacedName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

/*//{ getPrintName() */
std::string AltGeneric::getPrintName() const {
  return ch_->nodelet_name + "/" + parent_state_est_name_ + "/" + getName();
}
/*//}*/

};  // namespace mrs_uav_state_estimators
