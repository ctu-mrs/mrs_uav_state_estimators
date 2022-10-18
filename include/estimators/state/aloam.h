#ifndef STATEALOAM_H
#define STATEALOAM_H

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
/* #include "estimators/lateral/lat_aloam.h" */
#include "estimators/altitude/alt_generic.h"
/* #include "estimators/altitude/alt_aloam.h" */
#include "estimators/heading/hdg_generic.h"
/* #include "estimators/heading/hdg_aloam.h" */

//}

namespace mrs_uav_state_estimation
{

namespace aloam
{
const std::string name     = "Aloam";
const std::string frame_id = "aloam_origin";
}  // namespace aloam

class Aloam : public StateEstimator {

private:
  std::unique_ptr<LatGeneric>    est_lat_aloam_;
  const std::string est_lat_name_ = "lateral_aloam";

  std::unique_ptr<AltGeneric> est_alt_aloam_;
  const std::string est_alt_name_ = "altitude_aloam";

  std::unique_ptr<HdgGeneric> est_hdg_aloam_;
  const std::string est_hdg_name_ = "heading_aloam";

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
  Aloam() : StateEstimator(aloam::name, aloam::frame_id){};

  ~Aloam(void) {
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
