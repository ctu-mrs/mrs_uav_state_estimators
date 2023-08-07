#ifndef ESTIMATORS_HEADING_HDG_PASSTHROUGH_H
#define ESTIMATORS_HEADING_HDG_PASSTHROUGH_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/geometry/cyclic.h>

#include <mrs_uav_state_estimators/estimators/heading/heading_estimator.h>

//}

namespace mrs_uav_state_estimators
{

namespace hdg_passthrough
{
const int n_states = 2;
}  // namespace hdg_passthrough

using namespace mrs_uav_managers::estimation_manager;

class HdgPassthrough : public HeadingEstimator<hdg_passthrough::n_states> {

  using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;

private:
  const std::string package_name_ = "mrs_uav_state_estimators";

  std::string parent_state_est_name_;

  states_t           hdg_state_;
  mutable std::mutex mtx_hdg_state_;
  states_t           prev_hdg_state_;
  mutable std::mutex mtx_prev_hdg_state_;

  covariance_t       hdg_covariance_;
  mutable std::mutex mtx_hdg_covariance_;

  states_t           innovation_;
  mutable std::mutex mtx_innovation_;

  std::string                                                 orient_topic_;
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_orientation_;
  void                                                        callbackOrientation(mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> &wrp);
  std::atomic<bool>                                           is_orient_ready_ = false;

  std::string                                              ang_vel_topic_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> sh_ang_vel_;
  void                                                     callbackAngularVelocity(mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> &wrp);
  std::atomic<bool>                                        is_ang_vel_ready_ = false;

  ros::Timer timer_update_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

public:
  HdgPassthrough(const std::string &name, const std::string &ns_frame_id, const std::string &parent_state_est_name)
      : HeadingEstimator<hdg_passthrough::n_states>(name, ns_frame_id), parent_state_est_name_(parent_state_est_name) {
  }

  ~HdgPassthrough(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  double getState(const int &state_idx_in) const override;
  double getState(const int &state_id_in, const int &axis_in) const override;

  void setState(const double &state_in, const int &state_idx_in) override;
  void setState(const double &state_in, const int &state_id_in, const int &axis_in) override;

  states_t getStates(void) const override;
  void     setStates(const states_t &states_in) override;

  double getCovariance(const int &state_idx_in) const override;
  double getCovariance(const int &state_id_in, const int &axis_in) const override;

  covariance_t getCovarianceMatrix(void) const override;
  void         setCovarianceMatrix(const covariance_t &cov_in) override;

  double getInnovation(const int &state_idx) const override;
  double getInnovation(const int &state_id_in, const int &axis_in) const override;

  double getLastValidHdg() const override;

  std::string getNamespacedName() const;

  std::string getPrintName() const;
};
}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_HEADING_HDG_PASSTHROUGH_H
