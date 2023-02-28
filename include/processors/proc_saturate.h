#pragma once
#ifndef PROCESSORS_PROC_SATURATE_H
#define PROCESSORS_PROC_SATURATE_H

#include "processors/processor.h"

#include <mrs_lib/param_loader.h>

#include <limits>
#include <functional>

namespace mrs_uav_state_estimation
{

template <int n_measurements>
class ProcSaturate : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcSaturate(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch,
               StateId_t state_id, std::function<double(int, int)> fun_get_state);

  bool process(measurement_t& measurement) override;

private:
  const StateId_t                 state_id_;
  std::function<double(int, int)> fun_get_state_;

  bool keep_enabled_;
  double saturate_min_;
  double saturate_max_;
};

/*//{ constructor */
template <int n_measurements>
ProcSaturate<n_measurements>::ProcSaturate(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name,
                                           const std::shared_ptr<CommonHandlers_t>& ch, const StateId_t state_id, std::function<double(int,int)> fun_get_state)
    : Processor<n_measurements>(nh, correction_name, name, ch), state_id_(state_id), fun_get_state_(fun_get_state) {

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, Processor<n_measurements>::getPrintName());
  param_loader.setPrefix(Processor<n_measurements>::getNamespacedName() + "/");

  param_loader.loadParam("start_enabled", this->start_enabled_);
  this->enabled_ = this->start_enabled_;
  param_loader.loadParam("keep_enabled", keep_enabled_);
  param_loader.loadParam("min", saturate_min_);
  param_loader.loadParam("max", saturate_max_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", Processor<n_measurements>::getPrintName().c_str());
    ros::shutdown();
  }
}
/*//}*/

/*//{ process() */
template <int n_measurements>
bool ProcSaturate<n_measurements>::process(measurement_t& measurement) {

// if no saturation is required, processing is successful
  if (!this->enabled_) {
    return true;
  }

  bool ok_flag = true;
  for (int i = 0; i < measurement.rows(); i++) {
    const double state = fun_get_state_(state_id_, i);
    ROS_INFO_ONCE("[%s]: first state[%d][%d]: %.2f", Processor<n_measurements>::getNamespacedName().c_str(), state_id_, i, state);
    if (measurement(i) > state + saturate_max_) {
      const double saturated = state + saturate_max_;
      ROS_WARN_THROTTLE(1.0, "[%s]: state[%d][%d]: %.2f, measurement[%d]: %.2f saturated to: %.2f.", Processor<n_measurements>::getPrintName().c_str(), state_id_, i, state, i, measurement(i), saturated);
      measurement(i) = saturated;
      ok_flag        = false;
    } else if (measurement(i) < state + saturate_min_) {
      const double saturated = state + saturate_min_;
      ROS_WARN_THROTTLE(1.0, "[%s]: state[%d][%d]: %.2f, measurement[%d]: %.2f saturated to: %.2f.", Processor<n_measurements>::getPrintName().c_str(), state_id_, i, state, i, measurement(i), saturated);
      measurement(i) = saturated;
      ok_flag        = false;
    }
  }

  // measurements are close to the state, no need to saturate until triggered externally again
  if (!this->keep_enabled_ && ok_flag) {
    this->enabled_ = false;
  }

  return true; // saturated measurement is valid
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif  // PROCESSORS_PROC_MEDIAN_FILTER_H
