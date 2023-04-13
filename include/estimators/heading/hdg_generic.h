#ifndef ESTIMATORS_HEADING_HDG_GENERIC_H
#define ESTIMATORS_HEADING_HDG_GENERIC_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/repredictor.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/geometry/cyclic.h>

#include "estimators/heading/heading_estimator.h"
#include "estimators/correction.h"

#include <mrs_uav_state_estimators/HeadingEstimatorConfig.h>

//}

namespace mrs_uav_state_estimators
{

namespace hdg_generic
{

const int n_states       = 2;
const int n_inputs       = 1;
const int n_measurements = 1;

}  // namespace hdg_generic

class HdgGeneric : public HeadingEstimator<hdg_generic::n_states> {

  const std::string package_name_ = "mrs_uav_state_estimators";

  typedef mrs_lib::DynamicReconfigureMgr<HeadingEstimatorConfig> drmgr_t;

  using lkf_t      = mrs_lib::LKF<hdg_generic::n_states, hdg_generic::n_inputs, hdg_generic::n_measurements>;
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

  typedef mrs_lib::Repredictor<lkf_t> rep_lkf_t;

  using StateId_t = mrs_uav_managers::estimation_manager::StateId_t;

private:
  std::string parent_state_est_name_;

  double                              dt_;
  double                              input_coeff_;
  double                              default_input_coeff_;
  A_t                                 A_;
  B_t                                 B_;
  H_t                                 H_;
  Q_t                                 Q_;
  std::shared_ptr<lkf_t>              lkf_;
  std::unique_ptr<rep_lkf_t>          lkf_rep_;
  std::vector<std::shared_ptr<lkf_t>> models_;
  mutable std::mutex                  mutex_lkf_;
  statecov_t                          sc_;
  mutable std::mutex                  mutex_sc_;

  std::unique_ptr<drmgr_t> drmgr_;
  void                     callbackReconfigure(HeadingEstimatorConfig &config, [[maybe_unused]] uint32_t level);

  z_t                innovation_;
  mutable std::mutex mtx_innovation_;

  bool is_repredictor_enabled_;
  int  rep_buffer_size_ = 200;

  std::vector<std::string>                                              correction_names_;
  std::vector<std::shared_ptr<Correction<hdg_generic::n_measurements>>> corrections_;

  mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput> sh_control_input_;
  void                                                timeoutCallback(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  std::atomic<bool>                                   is_input_ready_ = false;

  ros::Timer timer_update_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  void doCorrection(const Correction<hdg_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t &state_id);
  void doCorrection(const z_t &z, const double R, const StateId_t &H_idx, const ros::Time &meas_stamp);

  bool isConverged();

  Q_t                getQ();
  mutable std::mutex mtx_Q_;

  mutable std::mutex mutex_last_valid_hdg_;
  double             last_valid_hdg_;

public:
  HdgGeneric(const std::string &name, const std::string &ns_frame_id, const std::string &parent_state_est_name)
      : HeadingEstimator<hdg_generic::n_states>(name, ns_frame_id), parent_state_est_name_(parent_state_est_name) {
  }

  ~HdgGeneric(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) override;
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

  void setDt(const double &dt);
  void setInputCoeff(const double &input_coeff);

  void generateA();
  void generateB();


  std::string getNamespacedName() const;

  std::string getPrintName() const;
};
}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_HEADING_HDG_GENERIC_H
