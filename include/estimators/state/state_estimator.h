#ifndef STATEESTIMATOR_H_
#define STATEESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_msgs/UavState.h>

#include "types.h"
#include "estimators/estimator.h"

//}

namespace mrs_uav_state_estimation
{

namespace state
{
const std::string type = "STATE";
}

class StateEstimator : public Estimator {

public:
protected:
  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

protected:
  ros::Publisher pub_uav_state_;

public:
  StateEstimator(const std::string &name, const std::string &frame_id) : Estimator(state::type, name, frame_id){};

  virtual ~StateEstimator(void) {
  }

  // virtual methods
  virtual mrs_msgs::UavState getUavState() const = 0;

  virtual bool setUavState(const mrs_msgs::UavState &uav_state) = 0;


  // implemented methods
  void publishUavState() const;
};


}  // namespace mrs_uav_state_estimation

#endif
