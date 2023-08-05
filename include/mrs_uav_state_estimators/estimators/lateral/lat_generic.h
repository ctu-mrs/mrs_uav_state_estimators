#ifndef ESTIMATORS_LATERAL_LAT_GENERIC_H
#define ESTIMATORS_LATERAL_LAT_GENERIC_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/repredictor.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_uav_state_estimators/estimators/lateral/lateral_estimator.h>
#include <mrs_uav_state_estimators/estimators/correction.h>

#include <mrs_uav_state_estimators/LateralEstimatorConfig.h>

//}

namespace mrs_uav_state_estimators
{

namespace lat_generic
{

const int n_states       = 6;
const int n_inputs       = 2;
const int n_measurements = 2;

}  // namespace lat_generic

class LatGeneric : public LateralEstimator<lat_generic::n_states> {

  typedef mrs_lib::DynamicReconfigureMgr<LateralEstimatorConfig> drmgr_t;

  using lkf_t      = mrs_lib::LKF<lat_generic::n_states, lat_generic::n_inputs, lat_generic::n_measurements>;
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

private:
  const std::string package_name_ = "mrs_uav_state_estimators";

  std::string parent_state_est_name_;

  double                              dt_;
  double                              input_coeff_, default_input_coeff_;
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
  void                     callbackReconfigure(LateralEstimatorConfig &config, [[maybe_unused]] uint32_t level);

  z_t                innovation_;
  mutable std::mutex mtx_innovation_;

  bool is_repredictor_enabled_;
  int  rep_buffer_size_ = 200;

  std::vector<std::string>                                              correction_names_;
  std::vector<std::shared_ptr<Correction<lat_generic::n_measurements>>> corrections_;

  mrs_lib::SubscribeHandler<mrs_msgs::EstimatorInput> sh_control_input_;
  void                                                timeoutCallback(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  std::atomic<bool>                                   is_input_ready_ = false;


  std::function<std::optional<double>()>               fun_get_hdg_;
  std::string                                          hdg_source_topic_;
  mrs_lib::SubscribeHandler<mrs_msgs::EstimatorOutput> sh_hdg_state_;
  std::atomic<bool>                                    is_hdg_state_ready_ = false;

  ros::Timer timer_update_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);


  void doCorrection(const Correction<lat_generic::n_measurements>::MeasurementStamped &meas, const double R, const StateId_t &state_id);
  void doCorrection(const z_t &z, const double R, const StateId_t &H_idx, const ros::Time &meas_stamp);

  bool isConverged();

  Q_t                getQ();
  mutable std::mutex mtx_Q_;

public:
  LatGeneric(const std::string &name, const std::string &ns_frame_id, const std::string &parent_state_est_name,
             std::function<std::optional<double>()> fun_get_hdg)
      : LateralEstimator<lat_generic::n_states>(name, ns_frame_id), parent_state_est_name_(parent_state_est_name), fun_get_hdg_(fun_get_hdg) {
  }

  ~LatGeneric(void) {
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

  void setDt(const double &dt);
  void setInputCoeff(const double &input_coeff);

  void generateA();
  void generateB();

  std::string getNamespacedName() const;

  std::string getPrintName() const;
};
}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_LATERAL_LAT_GENERIC_H
