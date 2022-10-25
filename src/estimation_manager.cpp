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
  ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::UavState>(nh, "uav_state_out", 1);
  ph_odom_main_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh, "odom_main_out", 1);
  /*//}*/

  /*//{ initialize timers */
  param_loader.loadParam("rate/uav_state", timer_rate_publish_);
  timer_publish_ = nh.createTimer(ros::Rate(timer_rate_publish_), &EstimationManager::timerPublish, this);

  param_loader.loadParam("rate/health", timer_rate_check_health_);
  timer_check_health_ = nh.createTimer(ros::Rate(timer_rate_check_health_), &EstimationManager::timerCheckHealth, this);
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

  if (sm_.isInPublishableState()) {

    const mrs_msgs::UavState uav_state = active_estimator_->getUavState();

    // TODO state health checks

    ph_uav_state_.publish(uav_state);

    nav_msgs::Odometry odom_main = uavStateToOdom(uav_state);

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

  /*//{ start ready estimators */
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
  }

  /*//}*/

  if (sm_.isInState(StateMachine::INITIALIZED_STATE) && initial_estimator_->isRunning()) {
    std::scoped_lock lock(mutex_active_estimator_);
    ROS_INFO("[%s]: activating the initial estimator %s", getName().c_str(), initial_estimator_->getName().c_str());
    active_estimator_ = initial_estimator_;
    sm_.changeState(StateMachine::READY_FOR_TAKEOFF_STATE);
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

/*//{ uavStateToOdom() */
nav_msgs::Odometry EstimationManager::uavStateToOdom(const mrs_msgs::UavState& uav_state) const {
  nav_msgs::Odometry odom;
  odom.header              = uav_state.header;
  odom.child_frame_id      = uav_state.child_frame_id;
  odom.pose.pose           = uav_state.pose;
  odom.twist.twist.angular = uav_state.velocity.angular;

  tf2::Quaternion q;
  tf2::fromMsg(odom.pose.pose.orientation, q);
  odom.twist.twist.linear = Support::rotateTwist(uav_state.velocity.linear, q.inverse(), ch_->transformer);

  return odom;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimation::EstimationManager, nodelet::Nodelet)
