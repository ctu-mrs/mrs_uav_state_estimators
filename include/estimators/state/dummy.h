#ifndef ESTIMATORS_STATE_DUMMY_H
#define ESTIMATORS_STATE_DUMMY_H

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

namespace dummy
{
const char name[]     = "dummy";
const char frame_id[] = "dummy_origin";
}  // namespace dummy

class Dummy : public StateEstimator {

private:
  const std::string est_lat_name_ = "lat_dummy";

  const std::string est_alt_name_ = "alt_dummy";

  const std::string est_hdg_name_ = "hdg_dummy";

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);
  bool       first_iter_ = true;

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  Dummy() : StateEstimator(dummy::name, dummy::frame_id) {
  }

  ~Dummy(void) {
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
}  // namespace mrs_uav_state_estimation

#endif  // ESTIMATORS_STATE_DUMMY_H
