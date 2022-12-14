#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Float64ArrayStamped.h>

#include "estimators/estimator.h"
#include "support.h"

//}

namespace mrs_uav_state_estimation
{

namespace state
{
const std::string type = "STATE";
}

class StateEstimator : public Estimator {

protected:
  mrs_msgs::UavState uav_state_;
  mutable std::mutex mtx_uav_state_;

  nav_msgs::Odometry odom_;
  mutable std::mutex mtx_odom_;

  nav_msgs::Odometry innovation_;
  mutable std::mutex mtx_innovation_;

  mrs_msgs::Float64ArrayStamped pose_covariance_, twist_covariance_;
  mutable std::mutex            mtx_covariance_;

protected:
  mutable mrs_lib::PublisherHandler<mrs_msgs::UavState>            ph_uav_state_;
  mutable mrs_lib::PublisherHandler<nav_msgs::Odometry>            ph_odom_;
  mutable mrs_lib::PublisherHandler<mrs_msgs::Float64ArrayStamped> ph_pose_covariance_, ph_twist_covariance_;
  mutable mrs_lib::PublisherHandler<nav_msgs::Odometry>            ph_innovation_;

public:
  StateEstimator(const std::string &name, const std::string &frame_id) : Estimator(state::type, name, frame_id){};

  virtual ~StateEstimator(void) {
  }

  // virtual methods
  virtual mrs_msgs::UavState  getUavState() const        = 0;
  virtual std::vector<double> getPoseCovariance() const  = 0;
  virtual std::vector<double> getTwistCovariance() const = 0;

  virtual bool setUavState(const mrs_msgs::UavState &uav_state) = 0;


  // implemented methods
  void publishUavState() const;
  void publishOdom() const;
  void publishCovariance() const;
  void publishInnovation() const;
  geometry_msgs::Quaternion rotateQuaternionByHeading(const geometry_msgs::Quaternion& q, const double hdg) const;

};

}  // namespace mrs_uav_state_estimation

#endif
