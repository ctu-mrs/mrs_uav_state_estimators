/* includes //{ */

#include <mrs_uav_state_estimators/estimators/agl/garmin_agl.h>

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

namespace garmin_agl
{

/* initialize() //{*/
void GarminAgl::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------------- load parameters -------------------- |

  if (is_core_plugin_) {

    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() + ".yaml");
    ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() + ".yaml");
  }

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    rclcpp::shutdown();
  }

  // | ------------------ timers initialization ----------------- |

  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node           = node_;
    opts.autostart      = false;
    opts.callback_group = cbkgrp_timers_;

    std::function<void()> callback_fcn = std::bind(&GarminAgl::timerUpdate, this);

    // TODO: parametrize the rate
    timer_update_ = std::make_shared<TimerType>(opts, rclcpp::Rate(100, clock_), callback_fcn);
  }

  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node           = node_;
    opts.autostart      = true;
    opts.callback_group = cbkgrp_timers_;

    std::function<void()> callback_fcn = std::bind(&GarminAgl::timerCheckHealth, this);

    // TODO: parametrize the rate
    timer_check_health_ = std::make_shared<TimerType>(opts, rclcpp::Rate(1, clock_), callback_fcn);
  }

  // | ---------------- publishers initialization --------------- |

  ph_agl_height_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, Support::toSnakeCase("~/" + getName()) + "/agl_height");

  if (ch_->debug_topics.covariance) {
    ph_agl_height_cov_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, Support::toSnakeCase("~/" + getName()) + "/agl_height_cov");
  }

  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics>(node_, Support::toSnakeCase("~/" + getName()) + "/diagnostics");
  }

  // | ---------------- estimators initialization --------------- |

  est_agl_garmin_ = std::make_unique<AltGeneric>(est_agl_name_, frame_id_, getName(), is_core_plugin_);
  est_agl_garmin_->initialize(node_, ch_, ph_);

  max_flight_z_ = est_agl_garmin_->getMaxFlightZ();

  // | ------------------ initialize published messages ------------------ |

  agl_height_init_.header.frame_id     = ns_frame_id_;
  agl_height_cov_init_.header.frame_id = ns_frame_id_;

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool GarminAgl::start(void) {


  if (isInState(READY_STATE)) {

    bool est_agl_garmin_start_successful;

    if (est_agl_garmin_->isStarted() || est_agl_garmin_->isRunning()) {
      est_agl_garmin_start_successful = true;
    } else {
      est_agl_garmin_start_successful = est_agl_garmin_->start();
    }

    if (est_agl_garmin_start_successful) {

      timer_update_->start();
      timer_check_health_->start();

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
bool GarminAgl::pause(void) {

  if (isInState(RUNNING_STATE)) {

    est_agl_garmin_->pause();

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
bool GarminAgl::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  est_agl_garmin_->pause();
  changeState(STOPPED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void GarminAgl::timerUpdate() {

  if (!isInitialized()) {
    return;
  }

  const rclcpp::Time time_now = clock_->now();

  mrs_msgs::msg::Float64Stamped agl_height = agl_height_init_;
  agl_height.header.stamp                  = time_now;
  agl_height.value                         = est_agl_garmin_->getState(POSITION);

  mrs_msgs::msg::Float64ArrayStamped agl_height_cov;
  agl_height_cov.header.stamp = time_now;

  const int n_states = 2; // TODO this should be defined somewhere else
  agl_height_cov.values.resize(n_states * n_states);
  agl_height_cov.values.at(n_states * POSITION + POSITION) = est_agl_garmin_->getCovariance(POSITION);
  agl_height_cov.values.at(n_states * VELOCITY + VELOCITY) = est_agl_garmin_->getCovariance(VELOCITY);

  mrs_lib::set_mutexed(mtx_agl_height_, agl_height, agl_height_);
  mrs_lib::set_mutexed(mtx_agl_height_cov_, agl_height_cov, agl_height_cov_);

  publishAglHeight();
  publishCovariance();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void GarminAgl::timerCheckHealth() {

  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

  case UNINITIALIZED_STATE: {
    break;
  }

  case INITIALIZED_STATE: {

    if (est_agl_garmin_->isInitialized()) {
      changeState(READY_STATE);
      RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator is ready to start", getPrintName().c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "[%s]: %s subestimators to be ready", getPrintName().c_str(), Support::waiting_for_string.c_str());
      return;
    }
    break;
  }

  case READY_STATE: {
    break;
  }

  case STARTED_STATE: {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s for convergence of LKF", getPrintName().c_str(), Support::waiting_for_string.c_str());

    if (est_agl_garmin_->isError()) {
      changeState(ERROR_STATE);
    }

    if (est_agl_garmin_->isRunning()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Subestimators converged", getPrintName().c_str());
      changeState(RUNNING_STATE);
    } else {
      return;
    }
    break;
  }

  case RUNNING_STATE: {
    if (est_agl_garmin_->isError()) {
      changeState(ERROR_STATE);
    }
    break;
  }

  case STOPPED_STATE: {
    break;
  }

  case ERROR_STATE: {
    if (est_agl_garmin_->isReady()) {
      changeState(READY_STATE);
    }
    break;
  }
  }
}
/*//}*/

/*//{ isConverged() */
bool GarminAgl::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ getUavAglHeight() */
mrs_msgs::msg::Float64Stamped GarminAgl::getUavAglHeight() const {
  return mrs_lib::get_mutexed(mtx_agl_height_, agl_height_);
}
/*//}*/

/*//{ getHeightCovariance() */
std::vector<double> GarminAgl::getHeightCovariance() const {
  return mrs_lib::get_mutexed(mtx_agl_height_cov_, agl_height_cov_.values);
}
/*//}*/

} // namespace garmin_agl

} // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::garmin_agl::GarminAgl, mrs_uav_managers::AglEstimator)
