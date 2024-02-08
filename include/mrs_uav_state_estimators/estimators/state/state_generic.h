#ifndef ESTIMATORS_STATE_STATE_GENERIC_H
#define ESTIMATORS_STATE_STATE_GENERIC_H

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
#include <mrs_uav_state_estimators/estimators/heading/heading_estimator.h>
#include <mrs_uav_state_estimators/estimators/heading/hdg_generic.h>
#include <mrs_uav_state_estimators/estimators/heading/hdg_passthrough.h>

//}

namespace mrs_uav_state_estimators
{

namespace hdg_estimator
{
const int n_states = 2;
}  // namespace hdg_estimator

namespace state_generic
{
const char package_name[] = "mrs_uav_state_estimators";
}

class StateGeneric : public mrs_uav_managers::StateEstimator {

private:
  std::unique_ptr<LatGeneric> est_lat_;
  std::string                 est_lat_name_;

  std::unique_ptr<AltGeneric> est_alt_;
  std::string                 est_alt_name_;

  bool                                                       is_hdg_passthrough_;
  std::unique_ptr<HeadingEstimator<hdg_estimator::n_states>> est_hdg_;
  std::string                                                est_hdg_name_;

  bool is_override_frame_id_;

  const bool is_core_plugin_;

  std::string                                                 topic_orientation_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orient_;
  void                                                        callbackOrientation(const geometry_msgs::QuaternionStamped::ConstPtr msg);

  std::string                                              topic_angular_velocity_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> sh_hw_api_ang_vel_;

  /* ros::Timer timer_update_; */
  /* void       timerUpdate(const ros::TimerEvent &event); */

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  ros::Timer timer_pub_attitude_;
  void       timerPubAttitude(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  StateGeneric(const std::string &name, const bool is_core_plugin)
      : StateEstimator(name, name + "_origin", state_generic::package_name), is_core_plugin_(is_core_plugin) {
  }

  ~StateGeneric(void) {
  }

  void initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const mrs_msgs::UavState &uav_state) override;

  std::optional<double> getHeading() const;

  void updateUavState(const geometry_msgs::QuaternionStamped::ConstPtr msg);
};

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_STATE_GENERIC_H
