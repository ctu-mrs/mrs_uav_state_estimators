#ifndef ESTIMATORS_ESTIMATOR_H
#define ESTIMATORS_ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/AttitudeCommand.h>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>

#include "mrs_uav_state_estimation/EstimatorDiagnostics.h"

#include "types.h"
#include "support.h"
#include "common_handlers.h"

//}

namespace mrs_uav_state_estimation
{

class Estimator {

protected:
  mutable mrs_lib::PublisherHandler<EstimatorDiagnostics> ph_diagnostics_;

  const std::string type_;
  const std::string name_;

  std::string frame_id_;  // cannot be constant - must remain overridable by loaded parameter
  std::string ns_frame_id_;

  std::shared_ptr<CommonHandlers_t> ch_;

  double max_flight_altitude_agl_ = -1.0;

private:
  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

protected:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id) {
  }

  virtual ~Estimator(void) {
  }

public:
  // virtual methods
  virtual void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) = 0;
  virtual bool start(void)                                                                  = 0;
  virtual bool pause(void)                                                                  = 0;
  virtual bool reset(void)                                                                  = 0;

  // implemented methods
  // access methods
  std::string getName(void) const;
  std::string getPrintName(void) const;
  std::string getType(void) const;
  std::string getFrameId(void) const;
  double      getMaxFlightAltitudeAgl(void) const;
  std::string getSmStateString(const SMStates_t &state) const;
  std::string getCurrentSmStateString(void) const;
  SMStates_t  getCurrentSmState() const;

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

  tf2::Vector3          getAccGlobal(const mrs_msgs::AttitudeCommand::ConstPtr &att_cmd_msg, const geometry_msgs::Quaternion &orientation);
  tf2::Vector3          getAccGlobal(const mrs_msgs::AttitudeCommand::ConstPtr &att_cmd_msg, const double hdg);
  std::optional<double> getHeadingRate(const geometry_msgs::Quaternion &att, const geometry_msgs::Vector3 &att_rate);
  std::optional<double> getHeadingRate(const mrs_msgs::AttitudeCommand::ConstPtr &att_cmd_msg);
  std::optional<double> getHeadingRate(const nav_msgs::Odometry::ConstPtr &odom_msg);
};


}  // namespace mrs_uav_state_estimation

#endif  // ESTIMATORS_ESTIMATOR_H
