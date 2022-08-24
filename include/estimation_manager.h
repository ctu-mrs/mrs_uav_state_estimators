#pragma once
#ifndef ESTIMATION_MANAGER_H
#define ESTIMATION_MANAGER_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <pluginlib/class_loader.h>
#include <nodelet/loader.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>

#include "estimators/state/state_estimator.h"
#include "support.h"
#include "common_handlers.h"

namespace mrs_uav_state_estimation
{

/*//{ class StateMachine */
class StateMachine {

  /*//{ states */
public:
  typedef enum
  {

    UNINITIALIZED_STATE,
    INITIALIZED_STATE,
    READY_FOR_TAKEOFF_STATE,
    TAKING_OFF_STATE,
    FLYING_STATE,
    HOVER_STATE,
    LANDING_STATE,
    LANDED_STATE,
    ESTIMATOR_SWITCHING_STATE,
    EMERGENCY_STATE,
    ERROR_STATE

  } SMState_t;

  /*//}*/

public:
  StateMachine(){};

  bool isInState(const SMState_t &state) const {
    std::scoped_lock lock(mtx_state_);
    return state == current_state_;
  }

  bool isInitialized() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_ != UNINITIALIZED_STATE;
  }

  bool isInPublishableState() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_ == READY_FOR_TAKEOFF_STATE || current_state_ == TAKING_OFF_STATE || current_state_ == HOVER_STATE || current_state_ == FLYING_STATE ||
           current_state_ == LANDING_STATE;
  }

  SMState_t getCurrentState() const {
    std::scoped_lock lock(mtx_state_);
    return current_state_;
  }

  std::string getCurrentStateString() const {
    std::scoped_lock lock(mtx_state_);
    return sm_state_names_[current_state_];
  }

  std::string getStateAsString(const SMState_t &state) const {
    return sm_state_names_[state];
  }

  /*//{ changeState() */
  bool changeState(const SMState_t &target_state) {

    switch (target_state) {

      case UNINITIALIZED_STATE: {
        ROS_ERROR("[%s]: transition to %s is not possible from any state", getName().c_str(), getStateAsString(UNINITIALIZED_STATE).c_str());
        return false;
        break;
      }

      case INITIALIZED_STATE: {
        if (current_state_ != UNINITIALIZED_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getName().c_str(), getStateAsString(INITIALIZED_STATE).c_str(),
                    getStateAsString(UNINITIALIZED_STATE).c_str());
          return false;
        }
        break;
      }

      case READY_FOR_TAKEOFF_STATE: {
        if (current_state_ != INITIALIZED_STATE && current_state_ != LANDED_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getName().c_str(), getStateAsString(READY_FOR_TAKEOFF_STATE).c_str(),
                    getStateAsString(INITIALIZED_STATE).c_str(), getStateAsString(LANDED_STATE).c_str());
          return false;
        }
        break;
      }

      case TAKING_OFF_STATE: {
        if (current_state_ != READY_FOR_TAKEOFF_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getName().c_str(), getStateAsString(TAKING_OFF_STATE).c_str(),
                    getStateAsString(READY_FOR_TAKEOFF_STATE).c_str());
          return false;
        }
        break;
      }

      case FLYING_STATE: {
        if (current_state_ != TAKING_OFF_STATE && current_state_ != HOVER_STATE && current_state_ != ESTIMATOR_SWITCHING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s or %s", getName().c_str(), getStateAsString(FLYING_STATE).c_str(),
                    getStateAsString(TAKING_OFF_STATE).c_str(), getStateAsString(HOVER_STATE).c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str());
          return false;
        }
        break;
      }

      case HOVER_STATE: {
        if (current_state_ != FLYING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getName().c_str(), getStateAsString(HOVER_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str());
          return false;
        }
        break;
      }

      case ESTIMATOR_SWITCHING_STATE: {
        if (current_state_ != FLYING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getName().c_str(), getStateAsString(ESTIMATOR_SWITCHING_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str());
          return false;
        }
        break;
      }

      case LANDING_STATE: {
        if (current_state_ != FLYING_STATE && current_state_ != HOVER_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s or %s", getName().c_str(), getStateAsString(LANDING_STATE).c_str(),
                    getStateAsString(FLYING_STATE).c_str(), getStateAsString(HOVER_STATE).c_str());
          return false;
        }
        break;
      }

      case LANDED_STATE: {
        if (current_state_ != LANDING_STATE) {
          ROS_ERROR("[%s]: transition to %s is possible only from %s", getName().c_str(), getStateAsString(LANDED_STATE).c_str(),
                    getStateAsString(LANDING_STATE).c_str());
          return false;
        }
        break;
      }

      case EMERGENCY_STATE: {
        ROS_WARN("[%s]: transition to %s", getName().c_str(), getStateAsString(EMERGENCY_STATE).c_str());
        break;
      }

      case ERROR_STATE: {
        ROS_WARN("[%s]: transition to %s", getName().c_str(), getStateAsString(ERROR_STATE).c_str());
        break;
      }

      default: {
        ROS_ERROR("[%s]: rejected transition to unknown state id %d", getName().c_str(), target_state);
        return false;
        break;
      }
    }

    std::scoped_lock lock(mtx_state_);
    {
      previous_state_ = current_state_;
      current_state_  = target_state;
    }

    ROS_INFO("[%s]: successfully changed states %s -> %s", getName().c_str(), getStateAsString(previous_state_).c_str(),
             getStateAsString(current_state_).c_str());

    return true;
  }
  /*//}*/

private:
  const std::string name_ = "StateMachine";

  SMState_t current_state_  = UNINITIALIZED_STATE;
  SMState_t previous_state_ = UNINITIALIZED_STATE;

  mutable std::mutex mtx_state_;

  std::string getName() const {
    return name_;
  }

  // clang-format off
  const std::vector<std::string> sm_state_names_ = {
  "UNINITIALIZED_STATE",
  "INITIALIZED_STATE",
  "READY_FOR_TAKEOFF_STATE",
  "TAKING_OFF_STATE",
  "FLYING_STATE",
  "HOVER_STATE",
  "LANDING_STATE",
  "LANDED_STATE",
  "ESTIMATOR_SWITCHING_STATE",
  "EMERGENCY_STATE",
  "ERROR_STATE"
  };
  // clang-format on
};
/*//}*/

class EstimationManager : public nodelet::Nodelet {

private:
  const std::string nodelet_name_ = "EstimationManager";

  std::string version_;

  std::shared_ptr<CommonHandlers_t> ch_;

  StateMachine sm_;

  mrs_lib::PublisherHandler<mrs_msgs::OdometryDiag> ph_diagnostics_;
  mrs_lib::PublisherHandler<mrs_msgs::UavState>     ph_uav_state_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>     ph_odom_main_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>     ph_odom_slow_;  // use topic throttler instead?
  mrs_lib::PublisherHandler<nav_msgs::Odometry>     ph_odom_main_innovation_;

  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_asml_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_altitude_agl_;
  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_max_altitude_agl_;

  mrs_lib::PublisherHandler<geometry_msgs::QuaternionStamped> ph_orientation_;

  ros::Timer timer_publish_;
  double     timer_rate_publish_;
  void       timerPublish(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  double     timer_rate_check_health_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  ros::ServiceServer srvs_change_estimator_;
  ros::ServiceServer srvs_toggle_callbacks_;

  // TODO service clients
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvch_failsafe_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_hover_; */
  /* mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> srvc_reference_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::Trigger> srvc_ehover_; */
  /* mrs_lib::ServiceClientHandler<std_srvs::SetBool> srvc_enable_callbacks_; */

  // | ------------- dynamic loading of estimators ------------- |
  std::unique_ptr<pluginlib::ClassLoader<mrs_uav_state_estimation::StateEstimator>> estimator_loader_;  // pluginlib loader of dynamically loaded estimators
  std::vector<std::string>                                                          _estimator_names_;  // list of estimator names
  /* std::map<std::string, EstimatorParams>                               estimator_params_;        // map between estimator names and estimator params */
  std::vector<boost::shared_ptr<mrs_uav_state_estimation::StateEstimator>> estimator_list_;  // list of estimators, routines are callable from this
  std::mutex                                                               mutex_estimator_list_;
  std::vector<std::string>                                                 estimator_names_;
  /* int                                                                      active_estimator_idx_; */
  std::string                                                 initial_estimator_name_ = "UNDEFINED_INITIAL_ESTIMATOR";
  boost::shared_ptr<mrs_uav_state_estimation::StateEstimator> initial_estimator_;
  boost::shared_ptr<mrs_uav_state_estimation::StateEstimator> active_estimator_;
  std::mutex                                                  mutex_active_estimator_;

  nav_msgs::Odometry uavStateToOdom(const mrs_msgs::UavState &uav_state) const;

public:
  EstimationManager();

  void onInit();

  std::string getName() const;
};

// constructor
EstimationManager::EstimationManager() {
}

}  // namespace mrs_uav_state_estimation

#endif
