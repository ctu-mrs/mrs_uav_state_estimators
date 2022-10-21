#ifndef ALTGENERIC_H
#define ALTGENERIC_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "estimators/altitude/altitude_estimator.h"
#include "estimators/correction.h"

//}

namespace mrs_uav_state_estimation
{

namespace alt_generic
{

const int n_states       = 3;
const int n_inputs       = 1;
const int n_measurements = 1;

/* const std::string name     = "altitude_garmin"; */
/* const std::string frame_id = "pixhawk_gps_origin"; */

}  // namespace alt_generic

using namespace mrs_lib;

class AltGeneric : public AltitudeEstimator<alt_generic::n_states> {

  using lkf_t      = LKF<alt_generic::n_states, alt_generic::n_inputs, alt_generic::n_measurements>;
  using A_t        = lkf_t::A_t;
  using B_t        = lkf_t::B_t;
  using H_t        = lkf_t::H_t;
  using Q_t        = lkf_t::Q_t;
  using x_t        = lkf_t::x_t;
  using P_t        = lkf_t::P_t;
  using u_t        = lkf_t::u_t;
  using z_t        = lkf_t::z_t;
  using R_t        = lkf_t::R_t;
  using statecov_t = lkf_t::statecov_t;

private:
  double                 dt_;
  double                 input_coeff_, default_input_coeff_;
  A_t                    A_;
  B_t                    B_;
  H_t                    H_;
  Q_t                    Q_;
  R_t                    R_;
  statecov_t             sc_;
  std::unique_ptr<lkf_t> lkf_;
  mutable std::mutex     mutex_lkf_;

  std::atomic<bool> is_input_ready_ = false;

  z_t                innovation_;
  mutable std::mutex mtx_innovation_;

  /* bool fuse_pos_range_; */
  /* bool fuse_pos_odom_; */
  /* bool fuse_vel_odom_; */

  std::vector<std::string> correction_names_;
  std::vector<Correction> corrections_;

  mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand> sh_attitude_command_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
  double                                        _critical_timeout_odom_;

  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_range_;
  double                                        _critical_timeout_range_;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  void doCorrection(const z_t& z);

  bool isConverged();

public:
  AltGeneric(const std::string name, const std::string frame_id, const AltitudeSource_t& alt_src) : AltitudeEstimator<alt_generic::n_states>(name, frame_id), alt_src_(alt_src){};

  ~AltGeneric(void) {
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

  virtual void setDt(const double& dt);
  virtual void setInputCoeff(const double& input_coeff);
  
  virtual void generateA();
  virtual void generateB();

  void timeoutOdom(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  void timeoutRange(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
};
}  // namespace mrs_uav_state_estimation

#endif
