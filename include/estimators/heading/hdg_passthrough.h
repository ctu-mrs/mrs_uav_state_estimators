#ifndef HDGPASSTHROUGH_H
#define HDGPASSTHROUGH_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "estimators/heading/heading_estimator.h"

//}

namespace mrs_uav_state_estimation
{

namespace hdg_passthrough
{
const int n_states = 2;
}  // namespace hdg_passthrough

using namespace mrs_lib;

class HdgPassthrough : public HeadingEstimator<hdg_passthrough::n_states> {


private:
  std::string parent_state_est_name_;

  states_t           hdg_state_;
  mutable std::mutex mtx_hdg_state_;

  covariance_t       hdg_covariance_;
  mutable std::mutex mtx_hdg_covariance_;

  states_t           innovation_;
  mutable std::mutex mtx_innovation_;

  std::string odom_topic_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
  std::atomic<bool>                             is_odom_ready_ = false;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

public:
  HdgPassthrough(const std::string& name, const std::string& ns_frame_id, const std::string& parent_state_est_name) : HeadingEstimator<hdg_passthrough::n_states>(name, ns_frame_id), parent_state_est_name_(parent_state_est_name){};

  ~HdgPassthrough(void) {
  }

  virtual void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual double getState(const int &state_idx_in) const override;
  virtual double getState(const int &state_id_in, const int &axis_in) const override;

  virtual void setState(const double &state_in, const int &state_idx_in) override;
  virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) override;

  virtual states_t getStates(void) const override;
  virtual void     setStates(const states_t &states_in) override;

  virtual double getCovariance(const int &state_idx_in) const override;
  virtual double getCovariance(const int &state_id_in, const int &axis_in) const override;

  virtual covariance_t getCovarianceMatrix(void) const override;
  virtual void         setCovarianceMatrix(const covariance_t &cov_in) override;

  virtual double getInnovation(const int &state_idx) const override;
  virtual double getInnovation(const int &state_id_in, const int &axis_in) const override;

  std::string getNamespacedName() const;
};
}  // namespace mrs_uav_state_estimation

#endif
