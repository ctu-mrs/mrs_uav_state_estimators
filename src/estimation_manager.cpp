#define VERSION "0.0.0.1"
#include "estimation_manager.h"

namespace mrs_uav_state_estimation
{

/*//{ onInit() */
void EstimationManager::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getName().c_str());

  ch_ = std::make_shared<CommonHandlers_t>();

  ch_->package_name = "mrs_uav_state_estimation";

  mrs_lib::ParamLoader param_loader(nh, getName());

  // load common parameters into the common handlers structure
  param_loader.loadParam("uav_name", ch_->uav_name);
  param_loader.loadParam("frame_id/fcu", ch_->frames.fcu);
  ch_->frames.ns_fcu = ch_->uav_name + "/" + ch_->frames.fcu;

  param_loader.loadParam("frame_id/fcu_untilted", ch_->frames.fcu_untilted);
  ch_->frames.ns_fcu_untilted = ch_->uav_name + "/" + ch_->frames.fcu_untilted;

  param_loader.loadParam("frame_id/rtk_antenna", ch_->frames.rtk_antenna);
  ch_->frames.ns_rtk_antenna = ch_->uav_name + "/" + ch_->frames.rtk_antenna;

  ch_->transformer = std::make_shared<mrs_lib::Transformer>(nh, getName());
  ch_->transformer->retryLookupNewest(true);

  /*//{ check version */
  param_loader.loadParam("version", version_);

  if (version_ != VERSION) {

    ROS_ERROR("[%s]: the version of the binary (%s) does not match the config file (%s), please build me!", getName().c_str(), VERSION, version_.c_str());
    ros::shutdown();
  }
  /*//}*/

  /*//{ load estimators */
  param_loader.loadParam("state_estimators", estimator_names_);

  estimator_loader_ = std::make_unique<pluginlib::ClassLoader<mrs_uav_state_estimation::StateEstimator>>("mrs_uav_state_estimation",
                                                                                                         "mrs_uav_state_estimation::StateEstimator");

  for (int i = 0; i < int(estimator_names_.size()); i++) {

    const std::string estimator_name = estimator_names_[i];

    // load the estimator parameters
    std::string address;
    param_loader.loadParam(estimator_name + "/address", address);

    try {
      ROS_INFO("[%s]: loading the estimator '%s'", getName().c_str(), address.c_str());
      estimator_list_.push_back(estimator_loader_->createInstance(address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[%s]: CreateClassException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[%s]: PluginlibException for the estimator '%s'", getName().c_str(), address.c_str());
      ROS_ERROR("[%s]: Error: %s", getName().c_str(), ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[%s]: estimators were loaded", getName().c_str());
  /*//}*/

  /*//{ check whether initial estimator was loaded */
  param_loader.loadParam("initial_estimator", initial_estimator_name_);
  bool initial_estimator_found = false;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == initial_estimator_name_) {
      initial_estimator_      = estimator;
      initial_estimator_found = true;
      break;
    }
  }

  if (!initial_estimator_found) {
    ROS_ERROR("[%s]: initial estimator %s could not be found among loaded estimators. shutting down", getName().c_str(), initial_estimator_name_.c_str());
    ros::shutdown();
  }
  /*//}*/

  /*//{ initialize estimators */
  for (auto estimator : estimator_list_) {

    try {
      ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), estimator->getName().c_str());
      estimator->initialize(nh, ch_);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[%s]: exception caught during estimator initialization: '%s'", getName().c_str(), ex.what());
    }
  }

  ROS_INFO("[%s]: estimators were initialized", getName().c_str());

  /*//}*/

  /*//{ initialize publishers */
  ph_uav_state_   = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, "uav_state_out", 1);
  ph_odom_main_   = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, "odom_main_out", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_uav_state_estimation::Diagnostics>(nh, "diagnostics_out", 1);
  /*//}*/

  /*//{ initialize timers */
  param_loader.loadParam("rate/uav_state", timer_rate_publish_);
  timer_publish_ = nh.createTimer(ros::Rate(timer_rate_publish_), &EstimationManager::timerPublish, this);

  param_loader.loadParam("rate/health", timer_rate_check_health_);
  timer_check_health_ = nh.createTimer(ros::Rate(timer_rate_check_health_), &EstimationManager::timerCheckHealth, this);
  /*//}*/

  /*//{ initialize service clients */

  srvch_failsafe_.initialize(nh, "failsafe_out");

  /*//}*/

  /*//{ initialize service servers */
  srvs_change_estimator_ = nh.advertiseService("change_estimator_in", &EstimationManager::callbackChangeEstimator, this);
  /*//}*/

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getName().c_str());
    ros::shutdown();
  }

  sm_.changeState(StateMachine::INITIALIZED_STATE);

  ROS_INFO("[%s]: initialized", getName().c_str());
}
/*//}*/

/*//{ timerPublish() */
void EstimationManager::timerPublish(const ros::TimerEvent& event) {

  if (!sm_.isInitialized()) {
    return;
  }

  if (!sm_.isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    mrs_uav_state_estimation::Diagnostics diagnostics;
    diagnostics.header.stamp                = ros::Time::now();
    diagnostics.current_sm_state            = sm_.getCurrentStateString();
    diagnostics.running_state_estimators    = estimator_names_;
    diagnostics.switchable_state_estimators = switchable_estimator_names_;
    if (active_estimator_ && active_estimator_->isInitialized()) {
      diagnostics.header.frame_id         = active_estimator_->getFrameId();
      diagnostics.current_state_estimator = active_estimator_->getName();
    } else {
      diagnostics.header.frame_id         = "NONE";
      diagnostics.current_state_estimator = "NONE";
    }
    ph_diagnostics_.publish(diagnostics);
  }

  if (sm_.isInPublishableState()) {

    mrs_msgs::UavState uav_state = active_estimator_->getUavState();

    uav_state.estimator_iteration = estimator_switch_count_;

    // TODO state health checks

    ph_uav_state_.publish(uav_state);

    nav_msgs::Odometry odom_main = Support::uavStateToOdom(uav_state, ch_->transformer);

    const std::vector<double> pose_covariance = active_estimator_->getPoseCovariance();
    for (size_t i = 0; i < pose_covariance.size(); i++) {
      odom_main.pose.covariance[i] = pose_covariance[i];
    }

    const std::vector<double> twist_covariance = active_estimator_->getTwistCovariance();
    for (size_t i = 0; i < twist_covariance.size(); i++) {
      odom_main.twist.covariance[i] = twist_covariance[i];
    }

    ph_odom_main_.publish(odom_main);

  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: not publishing uav state in %s", getName().c_str(), sm_.getCurrentStateString().c_str());
  }
}
/*//}*/

/*//{ timerCheckHealth() */
void EstimationManager::timerCheckHealth(const ros::TimerEvent& event) {

  if (!sm_.isInitialized()) {
    return;
  }

  /*//{ start ready estimators, check switchable estimators */
  switchable_estimator_names_.clear();
  for (auto estimator : estimator_list_) {

    if (estimator->isReady()) {
      try {
        ROS_INFO("[%s]: starting the estimator '%s'", getName().c_str(), estimator->getName().c_str());
        estimator->start();
      }
      catch (std::runtime_error& ex) {
        ROS_ERROR("[%s]: exception caught during estimator starting: '%s'", getName().c_str(), ex.what());
      }
    }

    if (estimator->isRunning() && estimator->getName() != "dummy" && estimator->getName() != "ground_truth") {
      switchable_estimator_names_.push_back(estimator->getName());
    }
  }

  /*//}*/

  if (!callbacks_disabled_by_service_ && (sm_.isInState(StateMachine::FLYING_STATE) || sm_.isInState(StateMachine::HOVER_STATE))) {
    callbacks_enabled_ = true;
  } else {
    callbacks_enabled_ = false;
  }

  // TODO fuj if, zmenit na switch
  // activate initial estimator
  if (sm_.isInState(StateMachine::INITIALIZED_STATE) && initial_estimator_->isRunning()) {
    std::scoped_lock lock(mutex_active_estimator_);
    ROS_INFO("[%s]: activating the initial estimator %s", getName().c_str(), initial_estimator_->getName().c_str());
    active_estimator_ = initial_estimator_;
    if (active_estimator_->getName() == "dummy") {
      sm_.changeState(StateMachine::DUMMY_STATE);
    } else {
      sm_.changeState(StateMachine::READY_FOR_TAKEOFF_STATE);
    }
  }

  // active estimator is in faulty state, we need to switch to healthy estimator
  if (sm_.isInTheAir() && active_estimator_->isError()) {
    sm_.changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
  }

  if (sm_.isInState(StateMachine::ESTIMATOR_SWITCHING_STATE)) {
    if (switchToHealthyEstimator()) {
      sm_.changeToPreSwitchState();
    } else {  // cannot switch to healthy estimator - failsafe necessary
      ROS_ERROR("[%s]: Cannot switch to any healthy estimator. Triggering failsafe.", getName().c_str());
      if (callFailsafeService()) {
        ROS_INFO("[%s]: failsafe called successfully", getName().c_str());
        sm_.changeState(StateMachine::FAILSAFE_STATE);
      }
    }
  }

  if (sm_.isInState(StateMachine::READY_FOR_TAKEOFF_STATE)) {
    sm_.changeState(StateMachine::TAKING_OFF_STATE);
  }

  if (sm_.isInState(StateMachine::TAKING_OFF_STATE)) {
    sm_.changeState(StateMachine::FLYING_STATE);
  }
}
/*//}*/

/*//{ callbackChangeEstimator() */
bool EstimationManager::callbackChangeEstimator(mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!sm_.isInitialized()) {
    return false;
  }

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[%s]: Ignoring service call. Callbacks are disabled.", getName().c_str());
    return true;
  }

  if (req.value == "dummy" || req.value == "ground_truth") {
    res.success = false;
    std::stringstream ss;
    ss << "Switching to " << req.value << " estimator is not allowed.";
    res.message = ss.str();
    ROS_WARN("[%s]: Switching to %s estimator is not allowed.", getName().c_str(), req.value.c_str());
    return true;
  }

  bool                                                        target_estimator_found = false;
  boost::shared_ptr<mrs_uav_state_estimation::StateEstimator> target_estimator;
  for (auto estimator : estimator_list_) {
    if (estimator->getName() == req.value) {
      target_estimator       = estimator;
      target_estimator_found = true;
      break;
    }
  }

  if (target_estimator_found) {

    if (target_estimator->isRunning()) {
      sm_.changeState(StateMachine::ESTIMATOR_SWITCHING_STATE);
      switchToEstimator(target_estimator);
      sm_.changeToPreSwitchState();
    } else {
      ROS_WARN("[%s]: Not running estimator %s requested", getName().c_str(), req.value.c_str());
      res.success = false;
      res.message = ("Requested estimator is not running");
      return true;
    }

  } else {
    ROS_WARN("[%s]: Switch to invalid estimator %s requested", getName().c_str(), req.value.c_str());
    res.success = false;
    res.message = ("Not a valid estimator type");
    return true;
  }

  res.success = true;
  res.message = "Estimator switch successful";

  return true;
}
/*//}*/

/*//{ switchToHealthyEstimator() */
bool EstimationManager::switchToHealthyEstimator() {

  // available estimators should be specified in decreasing priority order in config file
  for (auto estimator : estimator_list_) {
    if (estimator->isRunning()) {
      switchToEstimator(estimator);
      return true;
    }
  }
  return false;  // no other estimator is running
}
/*//}*/

/*//{ switchToEstimator() */
void EstimationManager::switchToEstimator(const boost::shared_ptr<mrs_uav_state_estimation::StateEstimator>& target_estimator) {

  std::scoped_lock lock(mutex_active_estimator_);
  ROS_INFO("[%s]: switching estimator from %s to %s", getName().c_str(), active_estimator_->getName().c_str(), target_estimator->getName().c_str());
  active_estimator_ = target_estimator;
  estimator_switch_count_++;
}
/*//}*/

/*//{ callFailsafeService() */
bool EstimationManager::callFailsafeService() {
  std_srvs::Trigger srv_out;
  return srvch_failsafe_.call(srv_out);
}
/*//}*/

/*//{ getName() */
std::string EstimationManager::getName() const {
  return nodelet_name_;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::EstimationManager, nodelet::Nodelet)
