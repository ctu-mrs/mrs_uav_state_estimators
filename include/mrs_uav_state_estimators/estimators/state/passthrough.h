#ifndef ESTIMATORS_STATE_PASSTHROUGH_H
#define ESTIMATORS_STATE_PASSTHROUGH_H

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

//}

namespace mrs_uav_state_estimators
{

namespace passthrough
{
const char name[]         = "passthrough";
const char frame_id[]     = "passthrough_origin";
const char package_name[] = "mrs_uav_state_estimators";

const bool is_core_plugin = true;

using namespace mrs_uav_managers::estimation_manager;

class Passthrough : public mrs_uav_managers::StateEstimator {

  using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;

private:
  const std::string package_name_ = "mrs_uav_state_estimators";

  const std::string est_lat_name_ = "lat_passthrough";

  const std::string est_alt_name_ = "alt_passthrough";

  const std::string est_hdg_name_ = "hdg_passthrough";

  const bool is_core_plugin_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_passthrough_odom_;
  void                                          callbackPassthroughOdom(const nav_msgs::Odometry::ConstPtr msg);
  double                                        _critical_timeout_passthrough_odom_;
  std::string                                   msg_topic_;

  ros::Timer       timer_check_passthrough_odom_hz_;
  void             timerCheckPassthroughOdomHz(const ros::TimerEvent &event);
  std::atomic<int> counter_odom_msgs_ = 0;
  ros::WallTime    t_check_hz_last_;
  double           prev_avg_hz_ = 0;
  bool             kickoff_     = false;

  ros::Timer                 timer_update_;
  void                       timerUpdate(const ros::TimerEvent &event);
  nav_msgs::OdometryConstPtr prev_msg_;
  bool                       first_iter_ = true;

  bool isConverged();

  void waitForEstimationInitialization();

public:
  Passthrough() : StateEstimator(passthrough::name, passthrough::frame_id, passthrough::package_name), is_core_plugin_(is_core_plugin) {
  }

  ~Passthrough(void) {
  }

  void initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  /* mrs_msgs::UavState  getUavState() override; */
  /* nav_msgs::Odometry  getInnovation() const override; */
  /* std::vector<double> getPoseCovariance() const override; */
  /* std::vector<double> getTwistCovariance() const override; */

  bool setUavState(const mrs_msgs::UavState &uav_state) override;
};

}  // namespace passthrough

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_PASSTHROUGH_H
