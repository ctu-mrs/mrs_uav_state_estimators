#ifndef ESTIMATORS_STATE_STATE_GENERIC_H
#define ESTIMATORS_STATE_STATE_GENERIC_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/utils.h>

#include <mrs_uav_managers/state_estimator.h>

#include <mrs_uav_state_estimators/estimators/lateral/lat_generic.h>
#include <mrs_uav_state_estimators/estimators/altitude/alt_generic.h>
#include <mrs_uav_state_estimators/estimators/heading/heading_estimator.h>
#include <mrs_uav_state_estimators/estimators/heading/hdg_generic.h>
#include <mrs_uav_state_estimators/estimators/heading/hdg_passthrough.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

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

namespace hdg_estimator
{
const int n_states = 2;
}  // namespace hdg_estimator

namespace state_generic
{
const char package_name[] = "mrs_uav_state_estimators";
}

class StateGeneric : public mrs_uav_managers::StateEstimator {

private:
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  std::unique_ptr<LatGeneric> est_lat_;
  std::string                 est_lat_name_;

  std::unique_ptr<AltGeneric> est_alt_;
  std::string                 est_alt_name_;

  bool                                                       is_hdg_passthrough_;
  std::unique_ptr<HeadingEstimator<hdg_estimator::n_states>> est_hdg_;
  std::string                                                est_hdg_name_;

  bool is_override_frame_id_;

  const bool is_core_plugin_;

  std::string                                                       topic_orientation_;
  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_hw_api_orient_;

  std::string                                                    topic_angular_velocity_;
  mrs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped> sh_hw_api_ang_vel_;

  std::shared_ptr<TimerType> timer_update_;
  void                       timerUpdate();

  std::shared_ptr<TimerType> timer_pub_attitude_;
  void                       timerPubAttitude();

  bool isConverged();

  void waitForEstimationInitialization();

public:
  StateGeneric(const std::string &name, const bool is_core_plugin)
      : StateEstimator(name, name + "_origin", state_generic::package_name), is_core_plugin_(is_core_plugin) {
  }

  void initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const mrs_msgs::msg::UavState &uav_state) override;

  std::optional<double> getHeading() const;

  void updateUavState() override;

  std::mutex mutex_update_uav_state_;
};

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_STATE_GENERIC_H
