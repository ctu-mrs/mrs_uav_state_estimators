#ifndef ESTIMATORS_STATE_GROUND_TRUTH_H
#define ESTIMATORS_STATE_GROUND_TRUTH_H

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

namespace ground_truth
{
const char name[]     = "ground_truth";
const char frame_id[] = "ground_truth_origin";

class GroundTruth : public mrs_uav_managers::StateEstimator {

private:
  /* std::unique_ptr<LatGeneric>    est_lat_gps_; */
  const std::string est_lat_name_ = "lat_gt";

  /* std::unique_ptr<AltGeneric> est_alt_garmin_; */
  const std::string est_alt_name_ = "alt_gt";

  /* std::unique_ptr<HdgPassthrough> est_hdg_mavros_; */
  const std::string est_hdg_name_ = "hdg_gt";

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_gt_odom_;
  double                                        _critical_timeout_gt_odom_;
  std::string                                   msg_topic_;

  ros::Timer                 timer_update_;
  int                        _update_timer_rate_;
  void                       timerUpdate(const ros::TimerEvent &event);
  nav_msgs::OdometryConstPtr prev_msg_;
  bool                       first_iter_ = true;

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  GroundTruth() : StateEstimator(ground_truth::name, ground_truth::frame_id) {
  }

  ~GroundTruth(void) {
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

}  // namespace ground_truth

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_GROUND_TRUTH_H
