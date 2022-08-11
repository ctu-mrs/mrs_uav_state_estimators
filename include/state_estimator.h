#ifndef STATEESTIMATOR_H_
#define STATEESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_msgs/UavState.h>

#include <mrs_uav_state_estimation/EstimatorDiagnostics.h>
#include <mrs_uav_state_estimation/EstimatorOutput.h>

#include "types.h"

//}

namespace mrs_uav_state_estimation
{

class StateEstimator {

public:

protected:
  ros::Publisher pub_uav_state_;
  ros::Publisher pub_diagnostics_;

  const std::string type_ = "STATE";
  const std::string name_;
  const std::string frame_id_;

  mrs_msgs::UavState uav_state_;
  std::mutex mutex_uav_state_;

private:

  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

public:
  StateEstimator(const std::string &name, const std::string &frame_id) : name_(name), frame_id_(frame_id){};

  virtual ~StateEstimator(void) {
  }

  // virtual methods
  virtual void initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool start(void)                                  = 0;
  virtual bool pause(void)                                  = 0;
  virtual bool reset(void)                                  = 0;

  virtual mrs_msgs::UavState getUavState() const = 0;

  virtual bool setUavState(const mrs_msgs::UavState& uav_state) = 0;

  /* virtual double getState(const int &state_idx_in) const                    = 0; */
  /* virtual double getState(const int &state_id_in, const int &axis_in) const = 0; */

  /* virtual void setState(const double &state_in, const int &state_idx_in)                    = 0; */
  /* virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) = 0; */

  /* virtual states_t getStates(void) const                = 0; */
  /* virtual void     setStates(const states_t &states_in) = 0; */

  /* virtual covariance_t getCovariance(void) const                 = 0; */
  /* virtual void         setCovariance(const covariance_t &cov_in) = 0; */

  // implemented methods
  // access methods
  std::string         getName(void) const;
  std::string         getType(void) const;
  std::string         getFrameId(void) const;
  std::string         getSmStateString(const SMStates_t &state) const;
  std::string         getCurrentSmStateString(void) const;
  /* std::vector<double> getStatesAsVector(void) const; */
  /* std::vector<double> getCovarianceAsVector(void) const; */

  // state machine methods
  bool changeState(SMStates_t new_state);
  bool isInState(const SMStates_t &state_in) const;
  bool isInitialized() const;
  bool isReady() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;
  /* int  stateIdToIndex(const int &axis_in, const int &state_id_in) const; */

  void publishDiagnostics() const;
  void publishUavState() const;
};

/*//{ method implementations */
/*//{ changeState() */
bool StateEstimator::changeState(SMStates_t new_state) {

  // do not initialize if the pub is empty
  ros::Publisher empty_pub;
  if (new_state == INITIALIZED_STATE) {
    if (pub_uav_state_ == empty_pub || pub_diagnostics_ == empty_pub) {
      ROS_ERROR("[%s]: cannot transition to %s - publishers are not initialized", getName().c_str(), getSmStateString(INITIALIZED_STATE).c_str());
      return false;
    }
  }

  previous_sm_state_ = current_sm_state_;
  current_sm_state_  = new_state;

  ROS_INFO("[%s]: Switching sm state %s -> %s", getName().c_str(), getSmStateString(previous_sm_state_).c_str(), getSmStateString(current_sm_state_).c_str());
  return true;
}
/*//}*/

/*//{ isInState() */
bool StateEstimator::isInState(const SMStates_t &state_in) const {
  return state_in == current_sm_state_;
}
/*//}*/

/*//{ isInitialized() */
bool StateEstimator::isInitialized() const {
  return !isInState(UNINITIALIZED_STATE);
}
/*//}*/

/*//{ isReady() */
bool StateEstimator::isReady() const {
  return isInState(READY_STATE);
}
/*//}*/

/*//{ isRunning() */
bool StateEstimator::isRunning() const {
  return isInState(RUNNING_STATE);
}
/*//}*/

/*//{ isStopped() */
bool StateEstimator::isStopped() const {
  return isInState(STOPPED_STATE);
}
/*//}*/

/*//{ isError() */
bool StateEstimator::isError() const {
  return isInState(ERROR_STATE);
}
/*//}*/

/*//{ stateIdToIndex() */
int StateEstimator::stateIdToIndex(const int &axis_in, const int &state_id_in) const {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

/*//{ getSmStateString() */
std::string StateEstimator::getSmStateString(const SMStates_t &state) const {
  return _sm_state_names_[state];
}
/*//}*/

/*//{ getCurrentSmStateName() */
std::string StateEstimator::getCurrentSmStateString(void) const {
  return getSmStateString(current_sm_state_);
}
/*//}*/

/*//{ getName() */
std::string StateEstimator::getName(void) const {
  return name_;
}
/*//}*/

/*//{ getType() */
std::string StateEstimator::getType(void) const {
  return type_;
}
/*//}*/

/*//{ getFrameId() */
std::string StateEstimator::getFrameId(void) const {
  return frame_id_;
}
/*//}*/

/*//{ getStatesAsvector() */
std::vector<double> StateEstimator::getStatesAsVector(void) const {
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
std::vector<double> StateEstimator::getCovarianceAsVector(void) const {
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

/*//{ publishDiagnostics() */
void StateEstimator::publishDiagnostics() const {

  EstimatorDiagnostics msg;
  msg.header.stamp       = ros::Time::now();
  msg.header.frame_id    = getFrameId();
  msg.estimator_name     = getName();
  msg.estimator_type     = getType();
  msg.estimator_sm_state = getCurrentSmStateString();

  try {
    pub_diagnostics_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_diagnostics_.getTopic().c_str());
  }
}
/*//}*/

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {


  try {
    pub_uav_state_.publish(uav_state_);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_uav_state_.getTopic().c_str());
  }
}
/*//}*/
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
