#ifndef ESTIMATORS_STATE_GPS_GARMIN_H
#define ESTIMATORS_STATE_GPS_GARMIN_H

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

#include <mrs_uav_managers/state_estimator.h>

#include "estimators/lateral/lat_generic.h"
#include "estimators/altitude/alt_generic.h"
#include "estimators/heading/hdg_passthrough.h"

//}

namespace mrs_uav_state_estimators
{

namespace gps_garmin
{
const char name[]         = "gps_garmin";
const char frame_id[]     = "gps_garmin_origin";
const char package_name[] = "mrs_uav_state_estimators";

class GpsGarmin : public mrs_uav_managers::StateEstimator {

private:
  std::unique_ptr<LatGeneric> est_lat_gps_;
  const std::string           est_lat_name_ = "lat_gps";

  std::unique_ptr<AltGeneric> est_alt_garmin_;
  const std::string           est_alt_name_ = "alt_garmin";

  std::unique_ptr<HdgPassthrough> est_hdg_hw_api_;
  const std::string               est_hdg_name_ = "hdg_hw_api";

  std::string topic_orientation_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orient_;

  std::string topic_angular_velocity_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>    sh_hw_api_ang_vel_;

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

public:
  GpsGarmin() : StateEstimator(gps_garmin::name, gps_garmin::frame_id, gps_garmin::package_name) {
  }

  ~GpsGarmin(void) {
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

}  // namespace gps_garmin

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_GPS_GARMIN_H
