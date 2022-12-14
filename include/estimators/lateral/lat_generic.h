#ifndef LATGENERIC_H
#define LATGENERIC_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "estimators/lateral/lateral_estimator.h"
#include "estimators/correction.h"

#include "mrs_uav_state_estimation/LateralEstimatorConfig.h"

//}

namespace mrs_uav_state_estimation
{

namespace lat_generic
{

const int n_states       = 6;
const int n_inputs       = 2;
const int n_measurements = 2;

}  // namespace lat_generic

using namespace mrs_lib;

class LatGeneric : public LateralEstimator<lat_generic::n_states> {

  typedef mrs_lib::DynamicReconfigureMgr<LateralEstimatorConfig> drmgr_t;

  using lkf_t      = LKF<lat_generic::n_states, lat_generic::n_inputs, lat_generic::n_measurements>;
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
  std::string parent_state_est_name_;

  double                 dt_;
  double                 input_coeff_, default_input_coeff_;
  A_t                    A_;
  B_t                    B_;
  H_t                    H_;
  Q_t                    Q_;
  statecov_t             sc_;
  std::unique_ptr<lkf_t> lkf_;
  mutable std::mutex     mutex_lkf_;

  std::unique_ptr<drmgr_t> drmgr_;

  z_t                innovation_;
  mutable std::mutex mtx_innovation_;

  std::vector<std::string>                                              correction_names_;
  std::vector<std::shared_ptr<Correction<lat_generic::n_measurements>>> corrections_;

  mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand> sh_attitude_command_;
  std::atomic<bool>                                    is_input_ready_ = false;

  std::string                                                          hdg_source_topic_;
  mrs_lib::SubscribeHandler<mrs_uav_state_estimation::EstimatorOutput> sh_hdg_state_;
  std::atomic<bool>                                                    is_hdg_state_ready_ = false;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  void doCorrection(const z_t &z, const double R, const StateId_t &state_id);

  bool isConverged();

  Q_t                getQ();
  mutable std::mutex mtx_Q_;

public:
  LatGeneric(const std::string& name, const std::string& ns_frame_id, const std::string &parent_state_est_name)
      : LateralEstimator<lat_generic::n_states>(name, ns_frame_id), parent_state_est_name_(parent_state_est_name){};

  ~LatGeneric(void) {
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

  virtual void setDt(const double &dt);
  virtual void setInputCoeff(const double &input_coeff);

  virtual void generateA();
  virtual void generateB();

  std::string getNamespacedName() const;
};
}  // namespace mrs_uav_state_estimation

#endif
