#ifndef PARTIAL_ESTIMATOR_H_
#define PARTIAL_ESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_uav_state_estimation/EstimatorOutput.h>

#include "types.h"
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
  ros::Publisher pub_output_;

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

  virtual covariance_t getCovariance(void) const                 = 0;
  virtual void         setCovariance(const covariance_t &cov_in) = 0;

  // implemented methods
  // access methods
  std::vector<double> getStatesAsVector(void) const;
  std::vector<double> getCovarianceAsVector(void) const;

  int stateIdToIndex(const int &axis_in, const int &state_id_in) const;

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
  const covariance_t  covariance = getCovariance();
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
int PartialEstimator<n_states, n_axes>::stateIdToIndex(const int &axis_in, const int &state_id_in) const {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

/*//{ publishOutput() */
template <int n_states, int n_axes>
void PartialEstimator<n_states, n_axes>::publishOutput() const {

  EstimatorOutput msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = getFrameId();
  msg.state           = getStatesAsVector();
  msg.covariance      = getCovarianceAsVector();

  try {
    pub_output_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_output_.getTopic().c_str());
  }
}
/*//}*/
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
