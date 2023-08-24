#define VERSION "0.0.6.0"

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/altitude/alt_generic.h>

//}

namespace mrs_uav_state_estimators

{

/* initialize() //{*/
void AltGeneric::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // clang-format off
  dt_ = 0.01;
  input_coeff_ = 10;
  default_input_coeff_ = 10;

  generateA();
  generateB();

  H_ <<
    1, 0, 0;

  // clang-format on

  // | --------------- initialize parameter loader -------------- |
  /* Support::loadParamFile(ros::package::getPath(package_name_) + "/config/estimators/" + getNamespacedName() + ".yaml", nh.getNamespace()); */
  bool success = true;

  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/private/" + getNamespacedName() + ".yaml");
  success *= ph_->loadConfigFile(ros::package::getPath(package_name_) + "/config/public/" + getNamespacedName() + ".yaml");

  if (!success) {
    ROS_ERROR("[%s]: could not load config file", getPrintName().c_str());
    return;
  }

  mrs_lib::ParamLoader param_loader(nh, getPrintName());
  param_loader.setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getNamespacedName() + "/");

  // | --------------------- load parameters -------------------- |
  param_loader.loadParam("max_flight_z", max_flight_z_);
  param_loader.loadParam("innovation/limit", pos_innovation_limit_);
  param_loader.loadParam("innovation/action", exc_innovation_action_name_);
  exc_innovation_action_ = map_exc_inno_action.at(exc_innovation_action_name_);
  param_loader.loadParam("repredictor/enabled", is_repredictor_enabled_);
  if (is_repredictor_enabled_) {
    param_loader.loadParam("repredictor/buffer_size", rep_buffer_size_);
  }

  // | --------------- corrections initialization --------------- |
  param_loader.loadParam("corrections", correction_names_);

  for (auto corr_name : correction_names_) {
    corrections_.push_back(std::make_shared<Correction<alt_generic::n_measurements>>(
        nh, getNamespacedName(), corr_name, ns_frame_id_, EstimatorType_t::ALTITUDE, ch_, [this](int a, int b) { return this->getState(a, b); },
        [this](const Correction<alt_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t state) {
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
  param_loader.loadParam("process_noise/acc", tmp_noise);
  Q_(ACCELERATION, ACCELERATION) = tmp_noise;

  // | ------- check if all parameters loaded successfully ------ |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------- initialize dynamic reconfigure ------------- |
  drmgr_ =
      std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), true, getPrintName(), boost::bind(&AltGeneric::callbackReconfigure, this, _1, _2));
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

    setDt(1.0 / ch_->desired_uav_state_rate);
  }

  // | ------------------ timers initialization ----------------- |
  timer_update_ = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &AltGeneric::timerUpdate, this);  // not running after init
  /* timer_check_health_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &AltGeneric::timerCheckHealth, this); */

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
bool AltGeneric::start(void) {

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
          setState(measurement_stamped.value(0), correction->getStateId());
          ROS_INFO_THROTTLE(1.0, "[%s]: Setting initial state to: %.2f", getPrintName().c_str(), measurement_stamped.value(0));
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for correction %s", getPrintName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      changeState(READY_STATE);
      ROS_INFO_THROTTLE(1.0, "[%s]: Ready to start", getPrintName().c_str());
      break;
    }

    case STARTED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is waiting for convergence of LKF", getPrintName().c_str());

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
      if (all_corrections_healthy && innovation_ok_) {
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

  if (!is_repredictor_enabled_) { // repredictor requires constant dt TODO: how to handle repredictor + variable rate?
    setDt(dt);
  }

  // prediction step
  u_t       u;
  ros::Time input_stamp;
  if (is_input_ready_) {
    mrs_msgs::EstimatorInputConstPtr msg            = sh_control_input_.getMsg();
    const tf2::Vector3               des_acc_global = getAccGlobal(msg, 0);  // we don't care about heading
    input_stamp                                     = msg->header.stamp;
    setInputCoeff(default_input_coeff_);
    u(0) = des_acc_global.getZ();
  } else {
    ROS_DEBUG_THROTTLE(1.0, "[%s]: not receiving control input, estimation suboptimal, potentially unstable", getPrintName().c_str());
    input_stamp = ros::Time::now();
    setInputCoeff(0);
    u = u_t::Zero();
  }

  // go through available corrections and apply them
  /* for (auto correction : corrections_) { */
  /*   auto res = correction->getProcessedCorrection(); */
  /*   if (res) { */
  /*     auto measurement_stamped = res.value(); */
  /*     doCorrection(measurement_stamped.value, correction->getR(), correction->getStateId(), measurement_stamped.stamp); */
  /*   } */
  /* } */

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

  // publishing
  publishInput(u, input_stamp);
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
        auto res = correction->getProcessedCorrection();
        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(0), correction->getStateId());
          ROS_INFO_THROTTLE(1.0, "[%s]: Setting initial state to: %.2f", getPrintName().c_str(), measurement_stamped.value(0));
        } else {
          ROS_INFO_THROTTLE(1.0, "[%s]: Waiting for correction %s", getPrintName().c_str(), correction->getNamespacedName().c_str());
          return;
        }
      }
      changeState(READY_STATE);
      ROS_INFO_THROTTLE(1.0, "[%s]: Ready to start", getPrintName().c_str());
      break;
    }

    case STARTED_STATE: {
      ROS_INFO_THROTTLE(1.0, "[%s]: Estimator is waiting for convergence of LKF", getPrintName().c_str());

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
void AltGeneric::doCorrection(const Correction<alt_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t &state_id) {
  doCorrection(meas.value, R, state_id, meas.stamp);
}
/*//}*/

/*//{ doCorrection() */
void AltGeneric::doCorrection(const z_t &z, const double R, const StateId_t &H_idx, const ros::Time &meas_stamp) {

  if (!isInitialized()) {
    return;
  }

  // for position state check the innovation
  if (H_idx == POSITION) {
    std::scoped_lock lock(mtx_innovation_);

    is_mitigating_jump_ = false;
    innovation_(0)      = z(0) - getState(POSITION);

    if (fabs(innovation_(0)) > pos_innovation_limit_) {
      ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - [%.2f] lim: %.2f", getPrintName().c_str(), innovation_(0), pos_innovation_limit_);
      innovation_ok_ = false;
      switch (exc_innovation_action_) {
        case ExcInnoAction_t::ELAND: {
          ROS_WARN_THROTTLE(1.0, "[%s]: large innovation should trigger eland in control manager", ros::this_node::getName().c_str());
          changeState(ERROR_STATE);
          break;
        }
        case ExcInnoAction_t::SWITCH: {
          ROS_WARN_THROTTLE(1.0, "[%s]: innovation should trigger estimator switch but no eland", ros::this_node::getName().c_str());
          innovation_(0) = 0.0;  // this is quite hacky but is there other way to switch estimators and not trigger eland by the large innovation?
          changeState(ERROR_STATE);
          break;
        }
        case ExcInnoAction_t::MITIGATE: {
          ROS_WARN_THROTTLE(1.0, "[%s]: large innovation should trigger estimate jump mitigation", ros::this_node::getName().c_str());
          innovation_(0)      = 0.0;  // this is quite hacky but is there other way to switch estimators and not trigger eland by the large innovation?
          is_mitigating_jump_ = true;
          setState(z(0), POSITION);
          break;
        }
        case ExcInnoAction_t::NONE: {
          ROS_WARN_THROTTLE(1.0, "[%s]: large innovation ignored", ros::this_node::getName().c_str());
          break;
        }
      }
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
    innovation_ok_ = true;
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getPrintName().c_str(), e.what());
  }

  mrs_lib::set_mutexed(mutex_sc_, sc, sc_);
}  // namespace mrs_uav_state_estimators
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
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void AltGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, 0));
}

void AltGeneric::setState(const double &state_in, const int &state_idx_in) {
  mrs_lib::set_mutexed(mutex_sc_, state_in, sc_.x(state_idx_in));
}
/*//}*/

/*//{ getStates() */
AltGeneric::states_t AltGeneric::getStates(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x;
}
/*//}*/

/*//{ setStates() */
void AltGeneric::setStates(const states_t &states_in) {
  mrs_lib::set_mutexed(mutex_sc_, states_in, sc_.x);
}
/*//}*/

/*//{ getCovariance() */
double AltGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {
  return getCovariance(stateIdToIndex(state_id_in, 0));
}

double AltGeneric::getCovariance(const int &state_idx_in) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
AltGeneric::covariance_t AltGeneric::getCovarianceMatrix(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void AltGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  mrs_lib::set_mutexed(mutex_sc_, cov_in, sc_.P);
}
/*//}*/

/*//{ getInnovation() */
double AltGeneric::getInnovation(const int &state_idx) const {
  return mrs_lib::get_mutexed(mtx_innovation_, innovation_)(state_idx);
}

double AltGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(0, 0));
}
/*//}*/

/*//{ setDt() */
void AltGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
  generateB();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
  lkf_->B = B_;
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
      1, dt_, 0.5 * dt_ * dt_,
      0, 1, dt_,
      0, 0, 1-(input_coeff_ * dt_);
  // clang-format on
}
/*//}*/

/*//{ generateB() */
void AltGeneric::generateB() {

  // clang-format off
    B_ <<
      0,
      0,
      (input_coeff_ * dt_);
  // clang-format on
}
/*//}*/

/*//{ callbackReconfigure() */
void AltGeneric::callbackReconfigure(AltitudeEstimatorConfig &config, [[maybe_unused]] uint32_t level) {

  if (!isInitialized()) {
    return;
  }

  Q_t Q;
  Q(POSITION, POSITION)         = config.pos;
  Q(VELOCITY, VELOCITY)         = config.vel;
  Q(ACCELERATION, ACCELERATION) = config.acc;
  mrs_lib::set_mutexed(mtx_Q_, Q, Q_);
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
