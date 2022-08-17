#include "estimators/estimator.h"

namespace mrs_uav_state_estimation
{

/*//{ method implementations */
/*//{ changeState() */
bool Estimator::changeState(SMStates_t new_state) {

  // do not initialize if the pub is empty
  ros::Publisher empty_pub;
  if (new_state == INITIALIZED_STATE) {
    if (pub_diagnostics_ == empty_pub) {
      ROS_ERROR("[%s]: cannot transition to %s - diagnostic publisher is not initialized", getName().c_str(), getSmStateString(INITIALIZED_STATE).c_str());
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
bool Estimator::isInState(const SMStates_t &state_in) const {
  return state_in == current_sm_state_;
}
/*//}*/

/*//{ isInitialized() */
bool Estimator::isInitialized() const {
  return !isInState(UNINITIALIZED_STATE);
}
/*//}*/

/*//{ isReady() */
bool Estimator::isReady() const {
  return isInState(READY_STATE);
}
/*//}*/

/*//{ isStarted() */
bool Estimator::isStarted() const {
  return isInState(STARTED_STATE);
}
/*//}*/

/*//{ isRunning() */
bool Estimator::isRunning() const {
  return isInState(RUNNING_STATE);
}
/*//}*/

/*//{ isStopped() */
bool Estimator::isStopped() const {
  return isInState(STOPPED_STATE);
}
/*//}*/

/*//{ isError() */
bool Estimator::isError() const {
  return isInState(ERROR_STATE);
}
/*//}*/

/*//{ getSmStateString() */
std::string Estimator::getSmStateString(const SMStates_t &state) const {
  return sm::state_names[state];
}
/*//}*/

/*//{ getCurrentSmStateName() */
std::string Estimator::getCurrentSmStateString(void) const {
  return getSmStateString(current_sm_state_);
}
/*//}*/

/*//{ getName() */
std::string Estimator::getName(void) const {
  return name_;
}
/*//}*/

/*//{ getType() */
std::string Estimator::getType(void) const {
  return type_;
}
/*//}*/

/*//{ getFrameId() */
std::string Estimator::getFrameId(void) const {
  return frame_id_;
}
/*//}*/

/*//{ publishDiagnostics() */
void Estimator::publishDiagnostics() const {

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

}  // namespace mrs_uav_state_estimation

/*//}*/
