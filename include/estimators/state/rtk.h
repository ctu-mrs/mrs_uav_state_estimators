#ifndef ESTIMATORS_STATE_RTK_H
#define ESTIMATORS_STATE_RTK_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

/* #include <mrs_msgs/AttitudeCommand.h> */

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <limits>
#include <algorithm>
#include <vector>
#include <memory>

#include <mrs_uav_managers/state_estimator.h>

#include "estimators/lateral/lat_generic.h"
#include "estimators/altitude/alt_generic.h"
#include "estimators/heading/hdg_passthrough.h"

//}

namespace mrs_uav_state_estimators
{

namespace rtk
{
const char name[]     = "rtk";
const char frame_id[] = "rtk_origin";

class Rtk : public mrs_uav_managers::StateEstimator {

private:
  std::unique_ptr<LatGeneric> est_lat_rtk_;
  const std::string           est_lat_name_ = "lat_rtk";

  std::unique_ptr<AltGeneric> est_alt_rtk_;
  const std::string           est_alt_name_ = "alt_rtk";

  std::unique_ptr<HdgPassthrough> est_hdg_mavros_;
  const std::string               est_hdg_name_ = "hdg_mavros";

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  double                                        _critical_timeout_mavros_odom_;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  ros::Timer timer_pub_attitude_;
  int        _pub_attitude_timer_rate_;
  void       timerPubAttitude(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

  void   getAvgRtkInitZ();
  bool   got_rtk_avg_init_z_ = false;
  double rtk_avg_init_z_     = 0.0;
  int    got_rtk_counter_    = 0;

  int    utm_origin_units_;
  double utm_origin_x_, utm_origin_y_;

public:
  Rtk() : StateEstimator(rtk::name, rtk::frame_id) {
  }

  ~Rtk(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  mrs_msgs::UavState  getUavState() const override;
  nav_msgs::Odometry  getInnovation() const override;
  std::vector<double> getPoseCovariance() const override;
  std::vector<double> getTwistCovariance() const override;

  bool setUavState(const mrs_msgs::UavState &uav_state) override;

};

}  // namespace rtk

}  // namespace mrs_uav_state_estimation

#endif  // ESTIMATORS_STATE_RTK_H
