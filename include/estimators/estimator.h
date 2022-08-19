#ifndef ESTIMATOR_H
#define ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_msgs/UavState.h>

#include <mrs_lib/publisher_handler.h>

#include <mrs_uav_state_estimation/EstimatorDiagnostics.h>

#include "types.h"

//}

namespace mrs_uav_state_estimation
{

class Estimator {

protected:
  mutable mrs_lib::PublisherHandler<EstimatorDiagnostics> ph_diagnostics_;

  std::string uav_name_; 

  const std::string type_;
  const std::string name_;
  const std::string frame_id_;

  // TODO load as parameters in manager, pass to estimators
  const std::string fcu_frame_id_ = "fcu";
  const std::string fcu_untilted_frame_id_ = "fcu_untilted";

private:
  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

public:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id){};

  virtual ~Estimator(void) {
  }

  // virtual methods
  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string& uav_name) = 0;
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
  bool isStarted() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;

  void publishDiagnostics() const;
};


}  // namespace mrs_uav_state_estimation

#endif
