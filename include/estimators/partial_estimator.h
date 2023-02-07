#ifndef PARTIAL_ESTIMATOR_H_
#define PARTIAL_ESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/Float64ArrayStamped.h>

#include "mrs_uav_state_estimation/EstimatorOutput.h"

#include "estimators/estimator.h"

//}

namespace mrs_uav_state_estimation
{

template <int n_states, int n_axes>
class PartialEstimator : public Estimator {

public:
  typedef Eigen::Matrix<double, n_states, 1>        states_t;
  typedef Eigen::Matrix<double, n_states, n_states> covariance_t;

protected:
  mutable mrs_lib::PublisherHandler<mrs_uav_state_estimation::EstimatorOutput> ph_output_;
  mutable mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped> ph_input_;

private:
  static const int _n_axes_   = n_axes;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

public:
  PartialEstimator(const std::string &type, const std::string &name, const std::string &frame_id) : Estimator(type, name, frame_id){};

  virtual ~PartialEstimator(void) {
  }

  // virtual methods
  virtual double getState(const int &state_idx_in) const                    = 0;
  virtual double getState(const int &state_id_in, const int &axis_in) const = 0;

  virtual void setState(const double &state_in, const int &state_idx_in)                    = 0;
  virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) = 0;

  virtual states_t getStates(void) const                = 0;
  virtual void     setStates(const states_t &states_in) = 0;

  virtual double getCovariance(const int &state_idx_in) const                    = 0;
  virtual double getCovariance(const int &state_id_in, const int &axis_in) const = 0;

  virtual covariance_t getCovarianceMatrix(void) const                 = 0;
  virtual void         setCovarianceMatrix(const covariance_t &cov_in) = 0;

  virtual double getInnovation(const int &state_idx) const = 0;
  virtual double getInnovation(const int &state_id_in, const int &axis_in) const = 0;

  // implemented methods
  // access methods
  std::vector<double> getStatesAsVector(void) const;
  std::vector<double> getCovarianceAsVector(void) const;

  int stateIdToIndex(const int &state_id_in, const int &axis_in) const;

  template <typename u_t>
  void publishInput(const u_t& u) const;
  void publishOutput() const;
};

/*//{ method implementations */

/*//{ getStatesAsvector() */
template <int n_states, int n_axes>
std::vector<double> PartialEstimator<n_states, n_axes>::getStatesAsVector(void) const {
  const states_t      states = getStates();
  std::vector<double> states_vec;
  /* for (auto st : Eigen::MatrixXd::Map(states, states.size(), 1).rowwise()) { */
  /*   states_vec.push_back(*st); */
  /* } */
  for (int i = 0; i < states.size(); i++) {
    states_vec.push_back(states(i));
  }
  return states_vec;
}
/*//}*/

/*//{ getCovarianceAsvector() */
template <int n_states, int n_axes>
std::vector<double> PartialEstimator<n_states, n_axes>::getCovarianceAsVector(void) const {
  const covariance_t  covariance = getCovarianceMatrix();
  std::vector<double> covariance_vec;
  /* for (auto cov : covariance.reshaped<Eigen::RowMajor>(covariance.size())) { */
  /*   covariance_vec.push_back(*cov); */
  /* } */
  for (int i = 0; i < covariance.rows(); i++) {
    for (int j = 0; j < covariance.cols(); j++) {
      covariance_vec.push_back(covariance(i, j));
    }
  }
  return covariance_vec;
}
/*//}*/

/*//{ stateIdToIndex() */
template <int n_states, int n_axes>
int PartialEstimator<n_states, n_axes>::stateIdToIndex(const int &state_id_in, const int &axis_in) const {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

/*//{ publishOutput() */
template <int n_states, int n_axes>
void PartialEstimator<n_states, n_axes>::publishOutput() const {

  mrs_uav_state_estimation::EstimatorOutput msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = getFrameId();
  msg.state           = getStatesAsVector();
  msg.covariance      = getCovarianceAsVector();

  ph_output_.publish(msg);
}
/*//}*/

/*//{ publishInput() */
template <int n_states, int n_axes>
template <typename u_t>
void PartialEstimator<n_states, n_axes>::publishInput(const u_t& u) const {

  mrs_msgs::Float64ArrayStamped msg;
  msg.header.stamp    = ros::Time::now();
  for (int i = 0; i < u.rows(); i++) {
    msg.values.push_back(u(i));
  }

  ph_input_.publish(msg);
}
/*//}*/

/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
