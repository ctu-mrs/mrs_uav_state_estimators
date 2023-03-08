#ifndef ESTIMATORS_STATE_GARMIN_AGL_H
#define ESTIMATORS_STATE_GARMIN_AGL_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_msgs/Float64Stamped.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_uav_managers/agl_estimator.h>

#include "estimators/altitude/alt_generic.h"

//}

namespace mrs_uav_state_estimators
{

namespace garmin_agl
{
const char name[]         = "garmin_agl";
const char frame_id[]     = "garmin_agl_origin";
const char package_name[] = "mrs_uav_state_estimators";

class GarminAgl : public mrs_uav_managers::AglEstimator {

private:
  std::unique_ptr<AltGeneric> est_agl_garmin_;
  const std::string           est_agl_name_ = "garmin_agl";

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  GarminAgl() : AglEstimator(garmin_agl::name, garmin_agl::frame_id, garmin_agl::package_name) {
  }

  ~GarminAgl(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  mrs_msgs::Float64Stamped  getUavAglHeight() const override;
  std::vector<double> getHeightCovariance() const override;

};

}  // namespace garmin_agl

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_GARMIN_AGL_H
