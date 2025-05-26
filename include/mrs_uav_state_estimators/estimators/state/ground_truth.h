#ifndef ESTIMATORS_STATE_GROUND_TRUTH_H
#define ESTIMATORS_STATE_GROUND_TRUTH_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_uav_managers/state_estimator.h>

#include <mrs_uav_state_estimators/estimators/lateral/lat_generic.h>
#include <mrs_uav_state_estimators/estimators/altitude/alt_generic.h>
#include <mrs_uav_state_estimators/estimators/heading/hdg_passthrough.h>

//}

namespace mrs_uav_state_estimators
{

namespace ground_truth
{
const char name[]         = "ground_truth";
const char frame_id[]     = "ground_truth_origin";
const char package_name[] = "mrs_uav_state_estimators";

class GroundTruth : public mrs_uav_managers::StateEstimator {

private:
  const std::string package_name_ = "mrs_uav_state_estimators";

  const std::string est_lat_name_ = "lat_gt";

  const std::string est_alt_name_ = "alt_gt";

  const std::string est_hdg_name_ = "hdg_gt";

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_gt_odom_;
  double                                        _critical_timeout_gt_odom_;
  std::string                                   msg_topic_;

  ros::Timer                 timer_update_;
  void                       timerUpdate(const ros::TimerEvent &event);
  nav_msgs::OdometryConstPtr prev_msg_;
  bool                       first_iter_ = true;

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  GroundTruth() : StateEstimator(ground_truth::name, ground_truth::frame_id, ground_truth::package_name) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const mrs_msgs::UavState &uav_state) override;
};

}  // namespace ground_truth

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_GROUND_TRUTH_H
