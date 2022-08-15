#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_msgs/UavState.h>

#include <mrs_uav_state_estimation/EstimatorDiagnostics.h>

#include "types.h"

//}

namespace mrs_uav_state_estimation
{

class Estimator {

public:
protected:
  ros::Publisher pub_diagnostics_;

  const std::string type_;
  const std::string name_;
  const std::string frame_id_;

private:
  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

public:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id){};

  virtual ~Estimator(void) {
  }

  // virtual methods
  virtual void initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool start(void)                                  = 0;
  virtual bool pause(void)                                  = 0;
  virtual bool reset(void)                                  = 0;

  // implemented methods
  // access methods
  std::string getName(void) const;
  std::string getType(void) const;
  std::string getFrameId(void) const;
  std::string getSmStateString(const SMStates_t &state) const;
  std::string getCurrentSmStateString(void) const;

  // state machine methods
  bool changeState(SMStates_t new_state);
  bool isInState(const SMStates_t &state_in) const;
  bool isInitialized() const;
  bool isReady() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;

  void publishDiagnostics() const;
};


}  // namespace mrs_uav_state_estimation

#endif
