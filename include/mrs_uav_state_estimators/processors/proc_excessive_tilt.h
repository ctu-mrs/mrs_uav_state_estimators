#pragma once
#ifndef PROCESSORS_PROC_EXCESSIVE_TILT_H
#define PROCESSORS_PROC_EXCESSIVE_TILT_H

#include <mrs_uav_state_estimators/processors/processor.h>

#include <mrs_uav_managers/estimation_manager/support.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <Eigen/Dense>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class ProcExcessiveTilt : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcExcessiveTilt(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name,
                    const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph);

  std::tuple<bool, bool> process(measurement_t& measurement) override;
  void                   reset();

private:
  double max_tilt_sq_;

  std::string                                                       orientation_topic_;
  mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped> sh_orientation_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
};

/*//{ constructor */
template <int n_measurements>
ProcExcessiveTilt<n_measurements>::ProcExcessiveTilt(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name,
                                                     const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph)
    : Processor<n_measurements>(node, correction_name, name, ch, ph) {

  cbkgrp_subs_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | --------------------- load parameters -------------------- |
  /* ph->param_loader->setPrefix(ch->package_name + "/" + Support::toSnakeCase(ch->nodelet_name) + "/" + Processor<n_measurements>::getNamespacedName() + "/");
   */

  ph->param_loader->loadParam("orientation_topic", orientation_topic_);
  double max_tilt;
  ph->param_loader->loadParam("max_tilt", max_tilt);

  max_tilt = M_PI * (max_tilt / 180.0);

  max_tilt_sq_ = std::pow(max_tilt, 2);

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(this->node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.",
                 Processor<n_measurements>::getPrintName().c_str());
    rclcpp::shutdown();
  }

  // | -------------- initialize subscribe handlers ------------- |
  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node;
  shopts.node_name                           = Processor<n_measurements>::getPrintName();
  shopts.no_message_timeout                  = rclcpp::Duration(std::chrono::duration<double>(1.0));
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_orientation_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, "/" + ch->uav_name + "/" + orientation_topic_);
}
/*//}*/

/*//{ process() */
template <int n_measurements>
std::tuple<bool, bool> ProcExcessiveTilt<n_measurements>::process([[maybe_unused]] measurement_t& measurement) {

  if (!Processor<n_measurements>::enabled_) {
    return {true, true};
  }

  if (!sh_orientation_.hasMsg()) {
    return {false, false};
  }

  bool ok_flag     = true;
  bool should_fuse = true;

  try {
    Eigen::Matrix3d orientation_R = mrs_lib::AttitudeConverter(sh_orientation_.getMsg()->quaternion);

    const double tilt = mrs_lib::geometry::angleBetween(Eigen::Vector3d(0, 0, 1), orientation_R.col(2));

    const bool is_excessive_tilt = std::pow(tilt, 2) > max_tilt_sq_;

    if (is_excessive_tilt) {
      RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->clock_, 1000, "[%s]: excessive tilt of %.2f deg. Not fusing correction.",
                           Processor<n_measurements>::getPrintName().c_str(), tilt / M_PI * 180);
      ok_flag     = false;
      should_fuse = false;
    }
  }
  catch (...) {
    RCLCPP_ERROR_THROTTLE(this->node_->get_logger(), *this->clock_, 1000, "[%s]: failed obtaining tilt value",
                          Processor<n_measurements>::getPrintName().c_str());
    ok_flag     = false;
    should_fuse = false;
  }
  return {ok_flag, should_fuse};
}
/*//}*/

/*//{ reset() */
template <int n_measurements>
void ProcExcessiveTilt<n_measurements>::reset() {
  // no need to do anything
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROC_EXCESSIVE_TILT_H
