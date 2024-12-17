#pragma once
#ifndef PROCESSORS_MAG_DECLINATION_H
#define PROCESSORS_MAG_DECLINATION_H

#include <mrs_uav_state_estimators/processors/processor.h>
#include <mrs_uav_state_estimators/processors/mag_declination/geo_mag_declination.h>

#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class ProcMagDeclination : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcMagDeclination(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch,
                     const std::shared_ptr<PrivateHandlers_t>& ph);

  std::tuple<bool, bool> process(measurement_t& measurement) override;
  void                   reset();

private:
  bool is_initialized_ = false;

  bool        use_gnss_msg_;
  std::string gnss_topic_;

  std::mutex       mtx_gnss_;
  std::atomic_bool got_gnss_ = false;
  double           gnss_lat_;
  double           gnss_lon_;

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_gnss_;
  void                                              callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg);
};

/*//{ constructor */
template <int n_measurements>
ProcMagDeclination<n_measurements>::ProcMagDeclination(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name,
                                             const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph)
    : Processor<n_measurements>(nh, correction_name, name, ch, ph) {

  ph->param_loader->setPrefix(ch->package_name + "/" + Support::toSnakeCase(ch->nodelet_name) + "/" + Processor<n_measurements>::getNamespacedName() + "/");

  ph->param_loader->loadParam("use_gnss_msg", use_gnss_msg_);
  if (use_gnss_msg_) {
    ph->param_loader->loadParam("gnss_topic", gnss_topic_);
  } else {
    ph->param_loader->loadParam("latitude", gnss_lat_);
    ph->param_loader->loadParam("longitude", gnss_lon_);
  }

  gnss_topic_ = "/" + ch->uav_name + "/" + gnss_topic_;

  if (!ph->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", Processor<n_measurements>::getPrintName().c_str());
    ros::shutdown();
  }

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = Processor<n_measurements>::getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, gnss_topic_, &ProcMagDeclination::callbackGnss, this);

  is_initialized_ = true;
}
/*//}*/

/*//{ callbackGnss() */
template <int n_measurements>
void ProcMagDeclination<n_measurements>::callbackGnss(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if (got_gnss_) {
    return;
  }

  if (!std::isfinite(msg->latitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->latitude\"!!!", Processor<n_measurements>::getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->longitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->longitude\"!!!", Processor<n_measurements>::getPrintName().c_str());
    return;
  }

  std::scoped_lock lock(mtx_gnss_);
  gnss_lat_ = msg->latitude;
  gnss_lon_ = msg->longitude;
  ROS_INFO("[%s]: First GNSS obtained: lat:%.2f lon:%.2f", Processor<n_measurements>::getPrintName().c_str(), gnss_lat_, gnss_lon_);
  got_gnss_ = true;
}
/*//}*/

/*//{ process() */
template <int n_measurements>
std::tuple<bool, bool> ProcMagDeclination<n_measurements>::process(measurement_t& measurement) {

  if (!Processor<n_measurements>::enabled_) {
    return {true, true};
  }

  std::scoped_lock lock(mtx_gnss_);

  if (!got_gnss_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: Missing GNSS data on topic: %s", Processor<n_measurements>::getPrintName().c_str(), gnss_topic_.c_str());
    return {false, false};
  }

  float mag_declination = GeoMagDeclination::getMagDeclinationRadians(gnss_lat_, gnss_lon_);

  measurement(0) -= mag_declination;

  return {true, true};
}
/*//}*/

/*//{ reset() */
template <int n_measurements>
void ProcMagDeclination<n_measurements>::reset() {
  got_gnss_                  = false;
}
/*//}*/
}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROC_MAG_DECLINATION
