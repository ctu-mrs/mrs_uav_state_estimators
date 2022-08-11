#ifndef GPSGARMIN_H_
#define GPSGARMIN_H_

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "state_estimator.h"
#include "support.h"
#include "gps.h"
#include "garmin.h"

//}

namespace mrs_uav_state_estimation
{

  namespace gps_garmin {
  const std::string name = "gps_garmin";
  const std::string frame_id = "gps_garmin_origin";
  }


class GpsGarmin : public StateEstimator {


private:
  ros::NodeHandle nh_;

  std::unique_ptr<Gps> est_lat_gps_;
  std::unique_ptr<Garmin> est_alt_garmin_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  double                                        _critical_timeout_mavros_odom_;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

  void waitForEstimationInitialization();

public:
  GpsGarmin() : StateEstimator(gps_garmin::name, gps_garmin::frame_id){};

  ~GpsGarmin(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual mrs_msgs::UavState getUavState() const = 0;

  virtual bool setUavState(const mrs_msgs::UavState& uav_state) = 0;

  /* virtual double getState(const int &state_idx_in) const override; */
  /* virtual double getState(const int &state_id_in, const int &axis_in) const override; */

  /* virtual void setState(const double &state_in, const int &state_idx_in) override; */
  /* virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) override; */

  /* virtual states_t getStates(void) const override; */
  /* virtual void     setStates(const states_t &states_in) override; */

  /* virtual covariance_t getCovariance(void) const override; */
  /* virtual void         setCovariance(const covariance_t &cov_in) override; */

};
}  // namespace mrs_uav_state_estimation

#endif
