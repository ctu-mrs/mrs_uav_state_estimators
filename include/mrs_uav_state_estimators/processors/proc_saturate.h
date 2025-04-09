#pragma once
#ifndef PROCESSORS_PROC_SATURATE_H
#define PROCESSORS_PROC_SATURATE_H

#include <mrs_uav_state_estimators/processors/processor.h>
#include <mrs_uav_managers/estimation_manager/estimator.h>

#include <mrs_lib/param_loader.h>

#include <functional>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class ProcSaturate : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcSaturate(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch,
               const std::shared_ptr<PrivateHandlers_t>& ph, StateId_t state_id, std::function<double(int, int)> fun_get_state);

  std::tuple<bool, bool> process(measurement_t& measurement) override;
  void                   reset();

private:
  const StateId_t                 state_id_;
  std::function<double(int, int)> fun_get_state_;

  bool   keep_enabled_;
  double saturate_min_;
  double saturate_max_;
  double innovation_limit_;
};

/*//{ constructor */
template <int n_measurements>
ProcSaturate<n_measurements>::ProcSaturate(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name,
                                           const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph, const StateId_t state_id,
                                           std::function<double(int, int)> fun_get_state)
    : Processor<n_measurements>(node, correction_name, name, ch, ph), state_id_(state_id), fun_get_state_(fun_get_state) {

  // | --------------------- load parameters -------------------- |
  /* ph->param_loader->setPrefix(ch->package_name + "/" + Support::toSnakeCase(ch->nodelet_name) + "/" + Processor<n_measurements>::getNamespacedName() + "/"); */

  ph->param_loader->loadParam("start_enabled", this->start_enabled_);
  this->enabled_ = this->start_enabled_;
  ph->param_loader->loadParam("keep_enabled", keep_enabled_);
  ph->param_loader->loadParam("min", saturate_min_);
  ph->param_loader->loadParam("max", saturate_max_);
  ph->param_loader->loadParam("limit", innovation_limit_);

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(this->node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.",
                 Processor<n_measurements>::getPrintName().c_str());
    rclcpp::shutdown();
  }
}
/*//}*/

/*//{ process() */
template <int n_measurements>
std::tuple<bool, bool> ProcSaturate<n_measurements>::process(measurement_t& measurement) {

  // if no saturation is required, processing is successful
  if (!this->enabled_) {
    return {true, true};
  }

  bool ok_flag     = true;
  bool should_fuse = true;
  for (int i = 0; i < measurement.rows(); i++) {

    const double state = fun_get_state_(state_id_, i);
    RCLCPP_INFO_ONCE(this->node_->get_logger(), "[%s]: first state[%d][%d]: %.2f", Processor<n_measurements>::getNamespacedName().c_str(), state_id_, i, state);

    if (measurement(i) > state + innovation_limit_ || measurement(i) < state - innovation_limit_) {
      return {true, true};  // do not even try to saturate, trigger innovation-based switch to other estimator
    }

    if (measurement(i) > state + saturate_max_) {
      const double saturated = state + saturate_max_;
      RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *(this->clock_), 1000, "[%s]: state[%d][%d]: %.2f, measurement[%d]: %.2f saturated to: %.2f.",
                           Processor<n_measurements>::getPrintName().c_str(), state_id_, i, state, i, measurement(i), saturated);
      measurement(i) = saturated;
      ok_flag        = false;
      should_fuse    = true;
    } else if (measurement(i) < state + saturate_min_) {
      const double saturated = state + saturate_min_;
      RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *(this->clock_), 1000, "[%s]: state[%d][%d]: %.2f, measurement[%d]: %.2f saturated to: %.2f.",
                           Processor<n_measurements>::getPrintName().c_str(), state_id_, i, state, i, measurement(i), saturated);
      measurement(i) = saturated;
      ok_flag        = false;
      should_fuse    = true;
    }
  }

  // measurements are close to the state, no need to saturate until triggered externally again
  if (!this->keep_enabled_ && ok_flag) {
    this->enabled_ = false;
  }

  return {ok_flag, should_fuse};  // saturated measurement is valid
}
/*//}*/

/*//{ reset() */
template <int n_measurements>
void ProcSaturate<n_measurements>::reset() {
  // no need to reset anything
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROC_MEDIAN_FILTER_H
