#define VERSION "0.6.0.0"
#include "estimation_manager.h"

namespace mrs_uav_state_estimation
{

/*//{ onInit() */
void EstimationManager::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing", getName().c_str());

  /* sm_ = std::make_unique<StateMachine>(); */

  mrs_lib::ParamLoader param_loader(nh, "EstimationManager");

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

    std::string estimator_name = estimator_names_[i];

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
  for (int i = 0; i < int(estimator_list_.size()); i++) {
    if (estimator_list_[i]->getName() == initial_estimator_name_) {
      initial_estimator_      = estimator_list_[i];
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
  for (int i = 0; i < int(estimator_list_.size()); i++) {

    try {
      ROS_INFO("[%s]: initializing the estimator '%s'", getName().c_str(), estimator_list_[i]->getName().c_str());
      estimator_list_[i]->initialize(nh);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[%s]: exception caught during tracker initialization: '%s'", getName().c_str(), ex.what());
    }
  }

  ROS_INFO("[%s]: estimators were initialized", getName().c_str());

  /*//}*/

  /*//{ initialize publishers */
  ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, "uav_state_out", 1);
  ph_odom_main_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, "odom_main_out", 1);
  /*//}*/

  /*//{ initialize timers */
  param_loader.loadParam("rate/uav_state", timer_rate_publish_);
  timer_publish_ = nh.createTimer(ros::Rate(timer_rate_publish_), &EstimationManager::timerPublish, this);

  param_loader.loadParam("rate/health", timer_rate_check_health_);
  timer_check_health_ = nh.createTimer(ros::Rate(timer_rate_check_health_), &EstimationManager::timerCheckHealth, this);
  /*//}*/

  sm_.changeState(StateMachine::INITIALIZED_STATE);

  ROS_INFO("[%s]: initialized", getName().c_str());
}
/*//}*/

/*//{ timerPublish() */
void EstimationManager::timerPublish(const ros::TimerEvent& event) {

  if (!sm_.isInitialized()) {
    return;
  }

  if (sm_.isInPublishableState()) {

    mrs_msgs::UavState uav_state = active_estimator_->getUavState();

    // TODO state health checks

    ph_uav_state_.publish(uav_state);

    // TODO transform velocity to body frame and publish odom

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

  if (sm_.isInState(StateMachine::INITIALIZED_STATE)) {
    if (initial_estimator_->isRunning()) {
      std::scoped_lock lock(mutex_active_estimator_);
      ROS_INFO("[%s]: activating the initial estimator %s", getName().c_str(), initial_estimator_->getName().c_str());
      active_estimator_ = initial_estimator_;
      sm_.changeState(StateMachine::READY_FOR_TAKEOFF_STATE);
    } else if (initial_estimator_->isReady()) {
      initial_estimator_->start();
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

/*//{ getName() */
std::string EstimationManager::getName() const {
  return nodelet_name_;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::EstimationManager, nodelet::Nodelet)
