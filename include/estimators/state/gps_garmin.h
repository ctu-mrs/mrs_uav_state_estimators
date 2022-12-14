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

#include "estimators/state/state_estimator.h"
#include "estimators/lateral/lat_generic.h"
#include "estimators/altitude/alt_generic.h"
#include "estimators/heading/hdg_passthrough.h"

//}

namespace mrs_uav_state_estimation
{

namespace gps_garmin
{
const std::string name     = "gps_garmin";
const std::string frame_id = "gps_garmin_origin";
}  // namespace gps_garmin

class GpsGarmin : public StateEstimator {

private:
  std::unique_ptr<LatGeneric>    est_lat_gps_;
  const std::string est_lat_name_ = "lat_gps";

  std::unique_ptr<AltGeneric> est_alt_garmin_;
  const std::string est_alt_name_ = "alt_garmin";

  std::unique_ptr<HdgPassthrough> est_hdg_mavros_;
  const std::string est_hdg_name_ = "hdg_mavros";

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  double                                        _critical_timeout_mavros_odom_;

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

  virtual void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual mrs_msgs::UavState  getUavState() const override;
  virtual std::vector<double> getPoseCovariance() const override;
  virtual std::vector<double> getTwistCovariance() const override;

  virtual bool setUavState(const mrs_msgs::UavState &uav_state) override;
};
}  // namespace mrs_uav_state_estimation

#endif
