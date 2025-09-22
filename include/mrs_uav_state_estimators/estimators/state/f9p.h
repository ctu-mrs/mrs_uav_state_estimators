#ifndef ESTIMATORS_STATE_F9P_H
#define ESTIMATORS_STATE_F9P_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/gps_conversions.h>

#include <mrs_uav_managers/state_estimator.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

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

namespace f9p
{
const char name[]         = "f9p";
const char frame_id[]     = "f9p_origin";
const char package_name[] = "mrs_uav_state_estimators";

const bool is_core_plugin = true;

using namespace mrs_uav_managers::estimation_manager;

class F9P : public mrs_uav_managers::StateEstimator {

  using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;

private:
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;

  const std::string package_name_ = "mrs_uav_state_estimators";

  const std::string est_lat_name_ = "lat_f9p";

  const std::string est_alt_name_ = "alt_f9p";

  const std::string est_hdg_name_ = "hdg_f9p";

  const bool is_core_plugin_;

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_f9p_odom_;
  void                                                callbackF9POdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  double                                              _critical_timeout_f9p_odom_;
  std::string                                         msg_topic_;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnss_;
  void                                                    callbackGnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  double                                                  gnss_x_, gnss_y_, gnss_z_;
  bool                                                    got_gnss_ = false;
  std::string                                             gnss_topic_;

  std::shared_ptr<TimerType> timer_check_f9p_odom_hz_;
  void                       timerCheckF9POdomHz();
  std::atomic<int>           counter_odom_msgs_ = 0;
  rclcpp::Time               t_check_hz_last_;
  double                     prev_avg_hz_ = 0;
  bool                       kickoff_     = false;

  std::shared_ptr<TimerType>              timer_update_;
  void                                    timerUpdate();
  nav_msgs::msg::Odometry::ConstSharedPtr prev_msg_;
  bool                                    first_iter_ = true;

  bool isConverged();

  void waitForEstimationInitialization();

  void updateUavState() override;

  std::mutex mutex_update_uav_state_;

public:
  F9P() : StateEstimator(f9p::name, f9p::frame_id, f9p::package_name), is_core_plugin_(is_core_plugin) {
  }

  void initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const mrs_msgs::msg::UavState &uav_state) override;
};

}  // namespace f9p

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_F9P_H
