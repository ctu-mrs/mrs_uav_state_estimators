#include "estimators/estimator.h"

namespace mrs_uav_state_estimation
{

/*//{ method implementations */
/*//{ changeState() */
bool Estimator::changeState(SMStates_t new_state) {

  previous_sm_state_ = current_sm_state_;
  current_sm_state_  = new_state;

  ROS_INFO("[%s]: Switching sm state %s -> %s", getName().c_str(), getSmStateString(previous_sm_state_).c_str(), getSmStateString(current_sm_state_).c_str());
  return true;
}
/*//}*/

/*//{ isInState() */
bool Estimator::isInState(const SMStates_t& state_in) const {
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
std::string Estimator::getSmStateString(const SMStates_t& state) const {
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
  return ns_frame_id_;
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

  ph_diagnostics_.publish(msg);
}
/*//}*/

/*//{ getAccGlobal() */
tf2::Vector3 Estimator::getAccGlobal(const mrs_msgs::AttitudeCommand::ConstPtr& att_cmd_msg, const nav_msgs::Odometry::ConstPtr& mavros_odom_msg) {

  // untilt the desired acceleration vector
  geometry_msgs::PointStamped des_acc;
  geometry_msgs::Vector3      des_acc_untilted;
  des_acc.point.x         = att_cmd_msg->desired_acceleration.x;
  des_acc.point.y         = att_cmd_msg->desired_acceleration.y;
  des_acc.point.z         = att_cmd_msg->desired_acceleration.z;
  des_acc.header.frame_id = ch_->frames.ns_fcu_untilted;
  des_acc.header.stamp    = att_cmd_msg->header.stamp;
  auto response_acc       = ch_->transformer->transformSingle(des_acc, ch_->frames.ns_fcu_untilted);
  if (response_acc) {
    des_acc_untilted.x = response_acc.value().point.x;
    des_acc_untilted.y = response_acc.value().point.y;
    des_acc_untilted.z = response_acc.value().point.z;
  } else {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transform from %s to %s failed", getName().c_str(), des_acc.header.frame_id.c_str(), ch_->frames.ns_fcu_untilted.c_str());
  }

  // rotate the desired acceleration vector to global frame
  const tf2::Vector3 des_acc_global =
      Support::rotateVecByHdg(des_acc_untilted, mrs_lib::AttitudeConverter(mavros_odom_msg->pose.pose.orientation).getHeading());

  return des_acc_global;
}
/*//}*/

/*//}*/

}  // namespace mrs_uav_state_estimation

