#ifndef GPSGARMIN_H
#define GPSGARMIN_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/AttitudeCommand.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include "support.h"
#include "estimators/state/state_estimator.h"
#include "estimators/lateral/gps.h"
#include "estimators/altitude/garmin.h"

//}

namespace mrs_uav_state_estimation
{

namespace gps_garmin
{
const std::string name     = "GpsGarmin";
const std::string frame_id = "gps_garmin_origin";
}  // namespace gps_garmin


class GpsGarmin : public StateEstimator {


private:
  ros::NodeHandle nh_;
  /* std::string uav_name_; */

  std::unique_ptr<Gps>    est_lat_gps_;
  std::unique_ptr<Garmin> est_alt_garmin_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  double                                        _critical_timeout_mavros_odom_;

  mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand> sh_attitude_command_;
  void                                                 callbackAttitudeCommand(mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand> &wrp);

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  GpsGarmin() : StateEstimator(gps_garmin::name, gps_garmin::frame_id){};

  ~GpsGarmin(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string &uav_name) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual mrs_msgs::UavState getUavState() const override;

  virtual bool setUavState(const mrs_msgs::UavState &uav_state) override;
};
}  // namespace mrs_uav_state_estimation

#endif
