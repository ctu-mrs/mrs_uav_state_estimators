/* includes //{ */

#include <mrs_uav_state_estimators/estimators/agl/garmin_agl.h>

//}

namespace mrs_uav_state_estimators
{

namespace garmin_agl
{

/* initialize() //{*/
void GarminAgl::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;
  nh_ = nh;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, getName());

  Support::loadParamFile(ros::package::getPath(package_name_) + "/config/estimators/" + getName() + "/" + getName() + ".yaml", nh_.getNamespace());
  param_loader.setPrefix(getName() + "/");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                          // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &GarminAgl::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                            // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &GarminAgl::timerCheckHealth, this);

  // | --------------- subscribers initialization --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // | ---------------- publishers initialization --------------- |
  ph_agl_height_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh, Support::toSnakeCase(getName()) + "/agl_height", 10);
  if (ch_->debug_topics.covariance) {
    ph_agl_height_cov_ = mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped>(nh, Support::toSnakeCase(getName()) + "/agl_height_cov", 10);
  }
  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::EstimatorDiagnostics>(nh, Support::toSnakeCase(getName()) + "/diagnostics", 10);
  }

  // | ---------------- estimators initialization --------------- |

  est_agl_garmin_ = std::make_unique<AltGeneric>(est_agl_name_, frame_id_, getName());
  est_agl_garmin_->initialize(nh, ch_);

  max_flight_z_ = est_agl_garmin_->getMaxFlightZ();

  // | ------------------ initialize published messages ------------------ |
  agl_height_init_.header.frame_id     = ns_frame_id_;
  agl_height_cov_init_.header.frame_id = ns_frame_id_;

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getPrintName().c_str());
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
      timer_update_.start();
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
bool GarminAgl::pause(void) {

  if (isInState(RUNNING_STATE)) {
    est_agl_garmin_->pause();
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
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  est_agl_garmin_->pause();
  changeState(STOPPED_STATE);

  ROS_INFO("[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void GarminAgl::timerUpdate(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  const ros::Time time_now = ros::Time::now();

  mrs_msgs::Float64Stamped agl_height = agl_height_init_;
  agl_height.header.stamp             = time_now;
  agl_height.value                    = est_agl_garmin_->getState(POSITION);

  mrs_msgs::Float64ArrayStamped agl_height_cov;
  agl_height_cov.header.stamp = time_now;

  const int n_states = 2;  // TODO this should be defined somewhere else
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
void GarminAgl::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (est_agl_garmin_->isReady()) {
      changeState(READY_STATE);
      ROS_INFO("[%s]: Estimator is ready to start", getPrintName().c_str());
    } else {
      ROS_INFO("[%s]: Waiting for subestimators to be ready", getPrintName().c_str());
      return;
    }
  }


  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getPrintName().c_str());

    if (est_agl_garmin_->isRunning()) {
      ROS_INFO("[%s]: Subestimators converged", getPrintName().c_str());
      changeState(RUNNING_STATE);
    } else {
      return;
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
mrs_msgs::Float64Stamped GarminAgl::getUavAglHeight() const {
  return mrs_lib::get_mutexed(mtx_agl_height_, agl_height_);
}
/*//}*/

/*//{ getHeightCovariance() */
std::vector<double> GarminAgl::getHeightCovariance() const {
  return mrs_lib::get_mutexed(mtx_agl_height_cov_, agl_height_cov_.values);
}
/*//}*/

}  // namespace garmin_agl

}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::garmin_agl::GarminAgl, mrs_uav_managers::AglEstimator)
