#define VERSION "0.0.0.1"

/* includes //{ */

#include "estimators/lateral/gps.h"

//}

namespace mrs_uav_state_estimation
{

/* initialize() //{*/
void Gps::initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch) {

  ch_ = ch;

  // TODO load parameters

  // clang-format off
    dt_ = 0.01;
    input_coeff_ = 0.1;

    A_ <<
      1, 0, dt_, 0, std::pow(dt_, 2)/2, 0,
      0, 1, 0, dt_, 0, std::pow(dt_, 2)/2,
      0, 0, 1, 0, dt_, 0,
      0, 0, 0, 1, 0, dt_,
      0, 0, 0, 0, 1-input_coeff_, 0,
      0, 0, 0, 0, 0, 1-input_coeff_;

    B_ <<
      0, 0,
      0, 0,
      0, 0,
      0, 0,
      input_coeff_, 0,
      0, input_coeff_;

    H_ <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0;

    Q_ <<
      0.001, 0, 0, 0, 0, 0,
      0, 0.001, 0, 0, 0, 0,
      0, 0, 0.01, 0, 0, 0,
      0, 0, 0, 0.01, 0, 0,
      0, 0, 0, 0, 0.01, 0,
      0, 0, 0, 0, 0, 0.01;

    R_ <<
      0.01, 0,
      0, 0.01;

  // clang-format on

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e1 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                    // TODO: parametrize
  timer_update_             = nh.createTimer(ros::Rate(_update_timer_rate_), &Gps::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                      // TODO: parametrize
  timer_check_health_       = nh.createTimer(ros::Rate(_check_health_timer_rate_), &Gps::timerCheckHealth, this);

  _critical_timeout_mavros_odom_ = 1.0;  // TODO: parametrize


  // | --------------- subscribers initialization --------------- |
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_attitude_command_ = mrs_lib::SubscribeHandler<mrs_msgs::AttitudeCommand>(shopts, "attitude_command_in");
  sh_mavros_odom_      = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in");

  // | ---------------- publishers initialization --------------- |
  ph_output_      = mrs_lib::PublisherHandler<mrs_uav_state_estimation::EstimatorOutput>(nh, getName() + "/output", 1);
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_uav_state_estimation::EstimatorDiagnostics>(nh, getName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool Gps::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool Gps::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool Gps::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  // Initialize LKF state and covariance
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  // Instantiate the LKF itself
  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  ROS_INFO("[%s]: Estimator reset", getName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void Gps::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }


  tf2::Vector3 des_acc_global;
  if (sh_attitude_command_.hasMsg() && sh_mavros_odom_.hasMsg()) {

    des_acc_global    = getAccGlobal(sh_attitude_command_.getMsg(), sh_mavros_odom_.getMsg());
    last_input_stamp_ = sh_attitude_command_.getMsg()->header.stamp;
    is_input_ready_   = true;
  }

  // prediction step
  u_t u;
  if (is_input_ready_) {
    u(0) = des_acc_global.getX();
    u(1) = des_acc_global.getY();
  } else {
    u = u_t::Zero();
  }


  try {
    // Apply the prediction step
    {
      std::scoped_lock lock(mutex_lkf_);
      sc_ = lkf_->predict(sc_, u, Q_, dt_);
    }
  }
  catch (const std::exception &e) {
    ROS_ERROR("[%s]: LKF prediction failed: %s", getName().c_str(), e.what());
  }

  if (sh_mavros_odom_.newMsg()) {

    z_t z = z_t::Zero();

    nav_msgs::Odometry::ConstPtr mavros_odom_msg = sh_mavros_odom_.getMsg();

    z(0) = mavros_odom_msg->pose.pose.position.x;
    z(1) = mavros_odom_msg->pose.pose.position.y;

    bool health_flag = true;

    if (!std::isfinite(z(0))) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in mavros position x correction", getName().c_str());
      health_flag = false;
    }

    if (!std::isfinite(z(1))) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: NaN detected in mavros position y correction", getName().c_str());
      health_flag = false;
    }

    if (health_flag) {

      {
        std::scoped_lock lock(mtx_innovation_);


        innovation_(0) = z(0) - getState(POSITION, AXIS_X);
        innovation_(1) = z(1) - getState(POSITION, AXIS_Y);

        if (innovation_(0) > 1.0 || innovation_(1) > 1.0) {
          ROS_WARN_THROTTLE(1.0, "[%s]: innovation too large - x: %.2f y: %.2f", getName().c_str(), innovation_(0), innovation_(1));
        }
      }

      try {
        // Apply the correction step
        {
          std::scoped_lock lock(mutex_lkf_);
          sc_ = lkf_->correct(sc_, z, R_);
        }
      }
      catch (const std::exception &e) {
        ROS_ERROR("[%s]: LKF correction failed: %s", getName().c_str(), e.what());
      }
    }
  }

  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void Gps::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    // initialize the estimator with current position
    if (sh_mavros_odom_.hasMsg()) {
      nav_msgs::OdometryConstPtr msg = sh_mavros_odom_.getMsg();
      setState(msg->pose.pose.position.x, POSITION, AXIS_X);
      setState(msg->pose.pose.position.y, POSITION, AXIS_Y);
      changeState(READY_STATE);
      ROS_INFO("[%s]: Ready to start", getName().c_str());
    } else {
      ROS_INFO("[%s]: Waiting for msg on topic %s", getName().c_str(), sh_mavros_odom_.topicName().c_str());
    }
  }

  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Waiting for convergence of LKF", getName().c_str());

    if (isConverged()) {
      ROS_INFO("[%s]: LKF converged", getName().c_str());
      changeState(RUNNING_STATE);
    }
  }

  // check age of input
  if (is_input_ready_ && (ros::Time::now() - last_input_stamp_).toSec() > 0.1) {
    ROS_WARN("[%s]: input too old (%.4f), using zero input instead", getName().c_str(), (ros::Time::now() - last_input_stamp_).toSec());
    is_input_ready_ = false;
  }
}
/*//}*/

/*//{ timeoutMavrosOdom() */
void Gps::timeoutMavrosOdom(const std::string &topic, const ros::Time &last_msg, const int n_pubs) {
  ROS_ERROR_STREAM("[" << getName().c_str() << "]: Estimator has not received message from topic '" << topic << "' for "
                       << (ros::Time::now() - last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");

  if ((ros::Time::now() - last_msg).toSec() > _critical_timeout_mavros_odom_) {
    ROS_ERROR("[%s]: Estimator not healthy", getName().c_str());
    changeState(ERROR_STATE);
  }
}
/*//}*/

/*//{ isConverged() */
bool Gps::isConverged() {

  // TODO: check convergence by rate of change of determinant

  return true;
}
/*//}*/

/*//{ getState() */
double Gps::getState(const int &state_id_in, const int &axis_in) const {

  return getState(stateIdToIndex(state_id_in, axis_in));
}

double Gps::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void Gps::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
  setState(state_in, stateIdToIndex(state_id_in, axis_in));
}

void Gps::setState(const double &state_in, const int &state_idx_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
Gps::states_t Gps::getStates(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.x;
}
/*//}*/

/*//{ setStates() */
void Gps::setStates(const states_t &states_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.x = states_in;
}
/*//}*/

/*//{ getCovariance() */
double Gps::getCovariance(const int &state_id_in, const int &axis_in) const {

  return getCovariance(stateIdToIndex(state_id_in, axis_in));
}

double Gps::getCovariance(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.P(state_idx_in, state_idx_in);
}
/*//}*/

/*//{ getCovarianceMatrix() */
Gps::covariance_t Gps::getCovarianceMatrix(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.P;
}
/*//}*/

/*//{ setCovarianceMatrix() */
void Gps::setCovarianceMatrix(const covariance_t &cov_in) {
  std::scoped_lock lock(mutex_lkf_);
  sc_.P = cov_in;
}
/*//}*/

/*//{ getInnovation() */
double Gps::getInnovation(const int &state_idx) const {
  std::scoped_lock lock(mtx_innovation_);
  return innovation_(state_idx);
}

double Gps::getInnovation(const int &state_id_in, const int &axis_in) const {
  return getInnovation(stateIdToIndex(state_id_in, axis_in));
}
/*//}*/

};  // namespace mrs_uav_state_estimation
