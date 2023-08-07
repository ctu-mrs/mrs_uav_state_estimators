#ifndef ESTIMATORS_STATE_DUMMY_H
#define ESTIMATORS_STATE_DUMMY_H

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

namespace dummy
{
const char name[]         = "dummy";
const char frame_id[]     = "dummy_origin";
const char package_name[] = "mrs_uav_state_estimators";

using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;

class Dummy : public mrs_uav_managers::StateEstimator {

private:
  const std::string package_name_ = "mrs_uav_state_estimators";

  const std::string est_lat_name_ = "lat_dummy";

  const std::string est_alt_name_ = "alt_dummy";

  const std::string est_hdg_name_ = "hdg_dummy";

  ros::Timer timer_update_;
  void       timerUpdate(const ros::TimerEvent &event);
  bool       first_iter_ = true;

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  Dummy() : StateEstimator(dummy::name, dummy::frame_id, dummy::package_name) {
  }

  ~Dummy(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const mrs_msgs::UavState &uav_state) override;
};

}  // namespace dummy

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_DUMMY_H
