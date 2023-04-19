#pragma once
#ifndef PROCESSORS_PROC_TF_TO_WORLD_H
#define PROCESSORS_PROC_TF_TO_WORLD_H

#include "processors/processor.h"

#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class ProcTfToWorld : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcTfToWorld(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch);

  bool process(measurement_t& measurement) override;

private:
  bool is_initialized_ = false;

  std::string gnss_topic_;

  bool   got_gnss_                  = false;
  bool   is_gnss_offset_calculated_ = false;
  double gnss_x_;
  double gnss_y_;

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_gnss_;
  void                                              callbackGnss(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp);
};

/*//{ constructor */
template <int n_measurements>
ProcTfToWorld<n_measurements>::ProcTfToWorld(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name,
                                             const std::shared_ptr<CommonHandlers_t>& ch)
    : Processor<n_measurements>(nh, correction_name, name, ch) {

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, Processor<n_measurements>::getPrintName());
  param_loader.setPrefix(Processor<n_measurements>::getNamespacedName() + "/");

  param_loader.loadParam("gnss_topic", gnss_topic_);
  gnss_topic_ = "/" + ch->uav_name + "/" + gnss_topic_;

  if (!param_loader.loadedSuccessfully()) {
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

  sh_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, gnss_topic_, &ProcTfToWorld::callbackGnss, this);

  is_initialized_ = true;
}
/*//}*/

/*//{ callbackGnss() */
template <int n_measurements>
void ProcTfToWorld<n_measurements>::callbackGnss(mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>& wrp) {

  if (!is_initialized_) {
    return;
  }

  if (got_gnss_) {
    return;
  }

  const sensor_msgs::NavSatFixConstPtr msg = wrp.getMsg();

  if (!std::isfinite(msg->latitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->latitude\"!!!", Processor<n_measurements>::getPrintName().c_str());
  }

  if (!std::isfinite(msg->longitude)) {
    ROS_ERROR_THROTTLE(1.0, "[%s] NaN detected in GNSS variable \"msg->longitude\"!!!", Processor<n_measurements>::getPrintName().c_str());
  }

  mrs_lib::UTM(msg->latitude, msg->longitude, &gnss_x_, &gnss_y_);
  got_gnss_ = true;
}
/*//}*/

/*//{ process() */
template <int n_measurements>
bool ProcTfToWorld<n_measurements>::process(measurement_t& measurement) {

  if (!Processor<n_measurements>::enabled_) {
    return false;
  }

  if (!got_gnss_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: Missing GNSS data on topic: %s", Processor<n_measurements>::getPrintName().c_str(), gnss_topic_.c_str());
    return false;
  }

  if (!is_gnss_offset_calculated_) {

    gnss_x_                    = (gnss_x_ - measurement(0)) - Processor<n_measurements>::ch_->utm_origin.x;
    gnss_y_                    = (gnss_y_ - measurement(1)) - Processor<n_measurements>::ch_->utm_origin.y;
    is_gnss_offset_calculated_ = true;
  }

  measurement(0) += gnss_x_;
  measurement(1) += gnss_y_;
  return true;
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROC_TF_TO_WORLD_H
