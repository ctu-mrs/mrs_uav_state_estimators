#define VERSION "0.0.0.1"

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/lateral/lat_generic.h>

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
void LatGeneric::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_ = node->create_sub_node(getNamespacedName());

  RCLCPP_INFO(node_->get_logger(), "initializing %s %s %s %s", getNamespacedName().c_str(), node_->get_name(), node_->get_namespace(),
              node_->get_sub_namespace().c_str());

  clock_ = node->get_clock();

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
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0;

  innovation_ <<
    0, 0;

  // clang-format on

  // | --------------- initialize parameter loader -------------- |

  if (is_core_plugin_) {

    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + parent_state_est_name_ + "/" + getName() +
                                  ".yaml");
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + parent_state_est_name_ + "/" + getName() +
                                  ".yaml");
  }

  ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getNamespacedName() + "/");

  // | --------------------- load parameters -------------------- |
  ph->param_loader->loadParam("hdg_source_topic", hdg_source_topic_);
  ph->param_loader->loadParam("max_flight_z", max_flight_z_);
  ph->param_loader->loadParam("innovation/limit", pos_innovation_limit_);
  ph->param_loader->loadParam("innovation/action", exc_innovation_action_name_);
  exc_innovation_action_ = map_exc_inno_action.at(exc_innovation_action_name_);
  ph->param_loader->loadParam("repredictor/enabled", is_repredictor_enabled_);
  if (is_repredictor_enabled_) {
    ph->param_loader->loadParam("repredictor/buffer_size", rep_buffer_size_);
  }

  // | --------------- corrections initialization --------------- |
  ph->param_loader->loadParam("corrections", correction_names_);

  for (auto corr_name : correction_names_) {
    corrections_.push_back(std::make_shared<Correction<lat_generic::n_measurements>>(
        node_, getNamespacedName(), corr_name, ns_frame_id_, EstimatorType_t::LATERAL, ch_, ph_, [this](int a, int b) { return this->getState(a, b); },
        [this](const Correction<lat_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t state) {
          return this->doCorrection(meas, R, state);
        }));
  }

  // | ------- check if all parameters loaded successfully ------ |
  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
  }

  ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getNamespacedName() + "/");

  // | ----------- initialize process noise covariance ---------- |
  Q_ = Q_t::Zero();
  double tmp_noise;
  ph->param_loader->loadParam("process_noise/pos", tmp_noise);
  Q_(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(POSITION, AXIS_Y), stateIdToIndex(POSITION, AXIS_Y)) = tmp_noise;
  ph->param_loader->loadParam("process_noise/vel", tmp_noise);
  Q_(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(VELOCITY, AXIS_Y), stateIdToIndex(VELOCITY, AXIS_Y)) = tmp_noise;
  ph->param_loader->loadParam("process_noise/acc", tmp_noise);
  Q_(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X)) = tmp_noise;
  Q_(stateIdToIndex(ACCELERATION, AXIS_Y), stateIdToIndex(ACCELERATION, AXIS_Y)) = tmp_noise;

  // | ------------- initialize dynamic reconfigure ------------- |

  // original DRS from ROS1
  /* drmgr_ = */
  /*     std::make_unique<drmgr_t>(ros::NodeHandle("~/" + getNamespacedName()), true, getPrintName(), boost::bind(&LatGeneric::callbackReconfigure, this, _1,
   * _2)); */
  /* drmgr_->config.pos = Q_(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X)); */
  /* drmgr_->config.vel = Q_(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X)); */
  /* drmgr_->config.acc = Q_(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X)); */
  /* drmgr_->update_config(drmgr_->config); */

  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    rcl_interfaces::msg::FloatingPointRange range;

    range.from_value = 0.0;
    range.to_value   = 100000.0;

    param_desc.floating_point_range = {range};

    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    node_->declare_parameter(node_->get_sub_namespace() + "/pos", 0.0, param_desc);
  }

  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    rcl_interfaces::msg::FloatingPointRange range;

    range.from_value = 0.0;
    range.to_value   = 100000.0;

    param_desc.floating_point_range = {range};

    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    node_->declare_parameter(node_->get_sub_namespace() + "/vel", 0.0, param_desc);
  }

  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    rcl_interfaces::msg::FloatingPointRange range;

    range.from_value = 0.0;
    range.to_value   = 100000.0;

    param_desc.floating_point_range = {range};

    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    node_->declare_parameter(node_->get_sub_namespace() + "/acc", 0.0, param_desc);
  }

  // | --------------- Kalman filter intialization -------------- |

  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    generateRepredictorModels(input_coeff_);

    const u_t          u0 = u_t::Zero();
    const rclcpp::Time t0 = clock_->now();
    lkf_rep_              = std::make_unique<mrs_lib::Repredictor<varstep_lkf_t>>(x0, P0, u0, Q_, t0, models_.at(0), rep_buffer_size_);

    setDt(1.0 / ch_->desired_uav_state_rate);
  }

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node      = node_;
  opts.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&LatGeneric::timerUpdate, this);

    timer_update_ = std::make_shared<TimerType>(opts, rclcpp::Rate(ch_->desired_uav_state_rate, clock_), callback_fcn);

    timer_update_last_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }

  // | --------------- subscribers initialization --------------- |
  // subscriber to odometry
  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node               = node_;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  sh_control_input_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimatorInput>(shopts, "control_input_in");

  // for transformation of desired accelerations from body to global frame
  sh_hdg_state_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimatorOutput>(shopts, hdg_source_topic_);

  // | ---------------- publishers initialization --------------- |
  if (ch_->debug_topics.input) {
    ph_input_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, "~/" + getNamespacedName() + "/input");
  }
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
bool LatGeneric::start(void) {

  if (isInState(READY_STATE)) {
    /* timer_update_.start(); */
    changeState(STARTED_STATE);
    return true;

  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
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
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  // reset processors of corrections
  for (auto correction : corrections_) {
    correction->resetProcessors();
  }

  // Initialize LKF state and covariance
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  // Instantiate the LKF itself
  lkf_ = std::make_shared<lkf_t>(A_, B_, H_);
  if (is_repredictor_enabled_) {

    const u_t          u0 = u_t::Zero();
    const rclcpp::Time t0 = rclcpp::Time(0);
    lkf_rep_              = std::make_unique<mrs_lib::Repredictor<varstep_lkf_t>>(x0, P0, u0, Q_, t0, models_[0], rep_buffer_size_);
  }

  changeState(INITIALIZED_STATE);
  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void LatGeneric::timerUpdate() {

  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s initialization", getPrintName().c_str(), Support::waiting_for_string.c_str());
      break;
    }

    case READY_STATE: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s estimator start", getPrintName().c_str(), Support::waiting_for_string.c_str());
      break;
    }

    case INITIALIZED_STATE: {

      // initialize the estimator with current corrections
      for (auto correction : corrections_) {

        auto res = correction->getProcessedCorrection();

        if (res) {
          auto measurement_stamped = res.value();
          setState(measurement_stamped.value(AXIS_X), correction->getStateId(), AXIS_X);
          setState(measurement_stamped.value(AXIS_Y), correction->getStateId(), AXIS_Y);
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Setting initial state to: %.2f %.2f", getPrintName().c_str(),
                               measurement_stamped.value(AXIS_X), measurement_stamped.value(AXIS_Y));
        } else {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s correction %s", getPrintName().c_str(), Support::waiting_for_string.c_str(),
                               correction->getNamespacedName().c_str());
          return;
        }
      }

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Ready to start", getPrintName().c_str());
      changeState(READY_STATE);
      break;
    }

    case STARTED_STATE: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s convergence of LKF", getPrintName().c_str(), Support::waiting_for_string.c_str());
      if (isConverged()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: LKF converged", getPrintName().c_str());
        changeState(RUNNING_STATE);
      }
      break;
    }

    case RUNNING_STATE: {
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Correction %s is not healthy!", getPrintName().c_str(),
                                correction->getNamespacedName().c_str());
          changeState(ERROR_STATE);
        }
      }
      break;
    }

    case STOPPED_STATE: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is stopped", getPrintName().c_str());
      break;
    }

    case ERROR_STATE: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is in ERROR state", getPrintName().c_str());

      rclcpp::Time t_now = clock_->now();
      if (is_error_state_first_time_) {
        prev_time_in_error_state_  = t_now;
        is_error_state_first_time_ = false;
        error_state_duration_      = 0;
      }
      error_state_duration_ += t_now.seconds() - prev_time_in_error_state_.seconds();


      // check if all corrections are healthy now
      bool all_corrections_healthy = true;
      for (auto correction : corrections_) {
        if (!correction->isHealthy()) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Correction %s is not healthy!", getPrintName().c_str(),
                                correction->getNamespacedName().c_str());
          all_corrections_healthy = false;
        }
      }

      if (all_corrections_healthy && innovation_ok_) {
        // initialize the estimator again if corrections become healthy
        if (error_state_duration_ > 5.0) {
          RCLCPP_INFO(node_->get_logger(), "[%s]: corrections healthy for %.2f s", getPrintName().c_str(), error_state_duration_);
          changeState(INITIALIZED_STATE);
          is_error_state_first_time_ = true;
        }
      } else {
        is_error_state_first_time_ = true;
      }

      prev_time_in_error_state_ = t_now;

      break;
    }
  }

  if (sh_control_input_.newMsg()) {
    is_input_ready_ = true;
  }

  // check age of input
  if (is_input_ready_ && (clock_->now() - sh_control_input_.lastMsgTime()).seconds() > 0.1) {  // TODO: parametrize, if older than say 1 second, eland
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: input too old (%.4f s), using zero input instead", getPrintName().c_str(),
                         (clock_->now() - sh_control_input_.lastMsgTime()).seconds());
    is_input_ready_ = false;
  }

  if (fun_get_hdg_()) {
    is_hdg_state_ready_ = true;
  }

  if (!isRunning() && !isStarted()) {
    return;
  }

  if (first_iter_) {
    first_iter_ = false;
    return;
  }

  // obtain dt for state prediction
  double dt = (clock_->now() - timer_update_last_time_).seconds();

  timer_update_last_time_ = clock_->now();

  if (dt <= 0.0 || dt > 1.0) {  // sometimes the timer ticks twice simultaneously in simulation - we ignore the second tick, in case of stopping and starting
                                // the timer, the last_real is 0
    return;
  }

  if (!is_repredictor_enabled_) {  // repredictor calculates dt on its own
    setDt(dt);
  }

  // obtain unbiased desired control acceleration in the estimator frame that will be used as input to the estimator
  u_t          u;
  rclcpp::Time input_stamp;
  if (is_input_ready_ && is_hdg_state_ready_) {
    auto res = fun_get_hdg_();
    if (!res) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not obtain heading", getPrintName().c_str());
      return;
    }

    const tf2::Vector3 des_acc_global = getAccGlobal(sh_control_input_.getMsg(), res.value());
    input_stamp                       = sh_control_input_.getMsg()->header.stamp;
    if (input_coeff_ != default_input_coeff_) {
      setInputCoeff(default_input_coeff_);
    }
    u(0) = des_acc_global.getX();
    u(1) = des_acc_global.getY();

  } else {  // this is ok before the controller starts controlling but bad during actual flight (causes delayed estimated acceleration and velocity)
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: not receiving control input, estimation suboptimal, potentially unstable",
                          getPrintName().c_str());
    input_stamp = clock_->now();
    if (input_coeff_ != 0) {
      setInputCoeff(0);
    }
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

  // get current state, covariance, and process noise
  statecov_t sc = mrs_lib::get_mutexed(mutex_sc_, sc_);
  Q_t        Q  = mrs_lib::get_mutexed(mtx_Q_, Q_);

  // prediction step
  try {
    // Apply the prediction step
    std::scoped_lock lock(mutex_lkf_);
    if (is_repredictor_enabled_) {
      lkf_rep_->addInputChangeWithNoise(u, Q, input_stamp, models_[0]);
      sc = lkf_rep_->predictTo(clock_->now());
    } else {
      sc = lkf_->predict(sc, u, Q, dt_);
    }
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: LKF prediction failed: %s", getPrintName().c_str(), e.what());
    changeState(ERROR_STATE);
  }

  // update the state and covariance variable that is queried by the estimation manager
  mrs_lib::set_mutexed(mutex_sc_, sc, sc_);

  // publishing
  publishInput(u, input_stamp);
  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ doCorrection() */
void LatGeneric::doCorrection(const Correction<lat_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t &state_id) {
  doCorrection(meas.value, R, state_id, meas.stamp);
}
/*//}*/

/*//{ doCorrection() */
void LatGeneric::doCorrection(const z_t &z, const double R, const StateId_t &state_id, const rclcpp::Time &meas_stamp) {

  if (!isInitialized()) {
    return;
  }

  // we do not want to perform corrections until the estimator is initialized
  if (!(isInState(SMStates_t::READY_STATE) || isInState(SMStates_t::RUNNING_STATE) || isInState(SMStates_t::STARTED_STATE))) {
    return;
  }

  // for position state check the innovation
  if (state_id == POSITION) {
    {
      std::scoped_lock lock(mtx_innovation_);

      is_mitigating_jump_ = false;
      innovation_(0)      = z(0) - getState(POSITION, AXIS_X);
      innovation_(1)      = z(1) - getState(POSITION, AXIS_Y);

      if (fabs(innovation_(0)) > pos_innovation_limit_ || fabs(innovation_(1)) > pos_innovation_limit_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: innovation too large - [%.2f %.2f] lim: %.2f", getPrintName().c_str(), innovation_(0),
                             innovation_(1), pos_innovation_limit_);
        innovation_ok_ = false;
        switch (exc_innovation_action_) {
          case ExcInnoAction_t::ELAND: {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: large innovation should trigger eland in control manager", getPrintName().c_str());
            changeState(ERROR_STATE);
            break;
          }
          case ExcInnoAction_t::SWITCH: {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: innovation should trigger estimator switch but no eland", getPrintName().c_str());
            innovation_(0) = 0.0;  // this is quite hacky but is there other way to switch estimators and not trigger eland by the large innovation?
            innovation_(1) = 0.0;
            changeState(ERROR_STATE);
            break;
          }
          case ExcInnoAction_t::MITIGATE: {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: large innovation should trigger estimate jump mitigation", getPrintName().c_str());
            innovation_(0)      = 0.0;  // this is quite hacky but is there other way to switch estimators and not trigger eland by the large innovation?
            innovation_(1)      = 0.0;
            is_mitigating_jump_ = true;
            setState(z(0), POSITION, AXIS_X);
            setState(z(1), POSITION, AXIS_Y);
            break;
          }
          case ExcInnoAction_t::NONE: {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: large innovation ignored", getPrintName().c_str());
            break;
          }
        }
      }
      innovation_ok_ = true;
    }
  }

  statecov_t sc = mrs_lib::get_mutexed(mutex_sc_, sc_);
  try {
    // Apply the correction step
    std::scoped_lock lock(mutex_lkf_);
    H_                           = H_t::Zero();
    H_(AXIS_X, state_id * 2)     = 1;
    H_(AXIS_Y, state_id * 2 + 1) = 1;
    lkf_->H                      = H_;

    if (is_repredictor_enabled_) {

      lkf_rep_->addMeasurement(z, R_t::Ones() * R, meas_stamp, models_[state_id]);
    } else {
      sc = lkf_->correct(sc, z, R_t::Ones() * R);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    RCLCPP_ERROR(node_->get_logger(), "[%s]: LKF correction failed: %s", getPrintName().c_str(), e.what());
  }

  mrs_lib::set_mutexed(mutex_sc_, sc, sc_);
}  // namespace mrs_uav_state_estimators
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
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void LatGeneric::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, axis_in));
}

void LatGeneric::setState(const double &state_in, const int &state_idx_in) {
  mrs_lib::set_mutexed(mutex_sc_, state_in, sc_.x(state_idx_in));
}
/*//}*/

/*//{ getStates() */
LatGeneric::states_t LatGeneric::getStates(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).x;
}
/*//}*/

/*//{ setStates() */
void LatGeneric::setStates(const states_t &states_in) {
  mrs_lib::set_mutexed(mutex_sc_, states_in, sc_.x);
}
/*//}*/

/*//{ getCovariance() */
double LatGeneric::getCovariance(const int &state_id_in, const int &axis_in) const {
  return getCovariance(stateIdToIndex(state_id_in, axis_in));
}

double LatGeneric::getCovariance(const int &state_idx_in) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
LatGeneric::covariance_t LatGeneric::getCovarianceMatrix(void) const {
  return mrs_lib::get_mutexed(mutex_sc_, sc_).P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void LatGeneric::setCovarianceMatrix(const covariance_t &cov_in) {
  mrs_lib::set_mutexed(mutex_sc_, cov_in, sc_.P);
}
/*//}*/

/*//{ getInnovation() */
double LatGeneric::getInnovation(const int &state_idx) const {
  return mrs_lib::get_mutexed(mtx_innovation_, innovation_)(state_idx);
}

double LatGeneric::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(state_id_in, axis_in));
}
/*//}*/

/*//{ setDt() */
void LatGeneric::setDt(const double &dt) {
  dt_ = dt;
  generateA();
  generateB();
  std::scoped_lock lock(mutex_lkf_);
  lkf_->A = A_;
  lkf_->B = B_;
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

  if (is_repredictor_enabled_) {
    models_.clear();
    generateRepredictorModels(input_coeff_);
  }
}
/*//}*/

/*//{ generateRepredictorModels() */
void LatGeneric::generateRepredictorModels(const double input_coeff) {
  for (int i = 0; (int)(i < lat_generic::n_states / 2); i++) {

    auto lambda_generateA = [input_coeff](const double dt) {
      A_t A;
      // clang-format off
        A <<
          1, 0, dt, 0, 0.5 * dt * dt, 0,
          0, 1, 0, dt, 0, 0.5 * dt * dt,
          0, 0, 1, 0, dt, 0,
          0, 0, 0, 1, 0, dt,
          0, 0, 0, 0, 1-(input_coeff * dt), 0,
          0, 0, 0, 0, 0, 1-(input_coeff * dt);
      // clang-format on
      return A;
    };

    auto lambda_generateB = [input_coeff]([[maybe_unused]] const double dt) {
      B_t B = B.Zero();
      // clang-format off
          B <<
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            input_coeff * dt, 0,
            0, input_coeff * dt;
      // clang-format on

      return B;
    };

    H_t H                = H.Zero();
    H(AXIS_X, i * 2)     = 1;
    H(AXIS_Y, i * 2 + 1) = 1;
    models_.push_back(std::make_shared<varstep_lkf_t>(lambda_generateA, lambda_generateB, H));
  }
}
/*//}*/

/*//{ generateA() */
void LatGeneric::generateA() {

  // clang-format off
    A_ <<
      1, 0, dt_, 0, 0.5 * dt_ * dt_, 0,
      0, 1, 0, dt_, 0, 0.5 * dt_ * dt_,
      0, 0, 1, 0, dt_, 0,
      0, 0, 0, 1, 0, dt_,
      0, 0, 0, 0, 1-(input_coeff_ * dt_), 0,
      0, 0, 0, 0, 0, 1-(input_coeff_ * dt_);
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
      input_coeff_ * dt_, 0,
      0, input_coeff_ * dt_;
  // clang-format on
}
/*//}*/

/* callbackParameters() //{ */

rcl_interfaces::msg::SetParametersResult LatGeneric::callbackParameters(std::vector<rclcpp::Parameter> parameters) {

  rcl_interfaces::msg::SetParametersResult result;

  // Note that setting a parameter to a nonsensical value (such as setting the `param_namespace.floating_number` parameter to `hello`)
  // doesn't have any effect - it doesn't even call this callback.
  for (auto &param : parameters) {

    RCLCPP_INFO_STREAM(node_->get_logger(), "got parameter: '" << param.get_name() << "' with value '" << param.value_to_string() << "'");

    if (param.get_name() == "pos") {

      auto Q = mrs_lib::get_mutexed(mtx_Q_, Q_);

      Q(stateIdToIndex(POSITION, AXIS_X), stateIdToIndex(POSITION, AXIS_X)) = param.as_double();
      Q(stateIdToIndex(POSITION, AXIS_Y), stateIdToIndex(POSITION, AXIS_Y)) = param.as_double();

      mrs_lib::set_mutexed(mtx_Q_, Q, Q_);

    } else if (param.get_name() == "vel") {

      auto Q = mrs_lib::get_mutexed(mtx_Q_, Q_);

      Q(stateIdToIndex(VELOCITY, AXIS_X), stateIdToIndex(VELOCITY, AXIS_X)) = param.as_double();
      Q(stateIdToIndex(VELOCITY, AXIS_Y), stateIdToIndex(VELOCITY, AXIS_Y)) = param.as_double();

      mrs_lib::set_mutexed(mtx_Q_, Q, Q_);

    } else if (param.get_name() == "acc") {

      auto Q = mrs_lib::get_mutexed(mtx_Q_, Q_);

      Q(stateIdToIndex(ACCELERATION, AXIS_X), stateIdToIndex(ACCELERATION, AXIS_X)) = param.as_double();
      Q(stateIdToIndex(ACCELERATION, AXIS_Y), stateIdToIndex(ACCELERATION, AXIS_Y)) = param.as_double();

      mrs_lib::set_mutexed(mtx_Q_, Q, Q_);

    } else {

      RCLCPP_WARN_STREAM(node_->get_logger(), "parameter: '" << param.get_name() << "' is not dynamically reconfigurable!");
      result.successful = false;
      result.reason     = "Parameter '" + param.get_name() + "' is not dynamically reconfigurable!";
      return result;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "params updated");
  result.successful = true;
  result.reason     = "OK";

  return result;
}

//}

/*//{ getNamespacedName() */
std::string LatGeneric::getNamespacedName() const {
  return parent_state_est_name_ + "/" + getName();
}
/*//}*/

/*//{ getPrintName() */
std::string LatGeneric::getPrintName() const {
  return ch_->nodelet_name + "/" + parent_state_est_name_ + "/" + getName();
}
/*//}*/
};  // namespace mrs_uav_state_estimators
