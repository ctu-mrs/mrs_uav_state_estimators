#pragma once
#ifndef PROCESSORS_PROC_MEDIAN_FILTER_H
#define PROCESSORS_PROC_MEDIAN_FILTER_H

#include "processors/processor.h"

#include <mrs_lib/median_filter.h>
#include <mrs_lib/param_loader.h>

#include <limits>

namespace mrs_uav_state_estimation
{

template <int n_measurements>
class ProcMedianFilter : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcMedianFilter(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch);

  bool process(measurement_t& measurement) override;

private:
  std::vector<mrs_lib::MedianFilter> vec_mf_;
  int                                buffer_size_;
  double                             max_diff_;
};

/*//{ constructor */
template <int n_measurements>
ProcMedianFilter<n_measurements>::ProcMedianFilter(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name,
                                                   const std::shared_ptr<CommonHandlers_t>& ch)
    : Processor<n_measurements>(nh, correction_name, name, ch) {

  // | --------------------- load parameters -------------------- |
  mrs_lib::ParamLoader param_loader(nh, Processor<n_measurements>::getName());
  param_loader.setPrefix(Processor<n_measurements>::getNamespacedName() + "/");

  param_loader.loadParam("buffer_size", buffer_size_);
  param_loader.loadParam("max_diff", max_diff_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", Processor<n_measurements>::getNamespacedName().c_str());
    ros::shutdown();
  }

  // min and max values are not checked by median filter, so set them to limits of double
  const double min_valid = std::numeric_limits<double>::lowest();
  const double max_valid = std::numeric_limits<double>::max();

  // initialize median filter
  for (int i = 0; i < n_measurements; i++) {
    vec_mf_.push_back(mrs_lib::MedianFilter(buffer_size_, min_valid, max_valid, max_diff_));
  }
}
/*//}*/

/*//{ process() */
template <int n_measurements>
bool ProcMedianFilter<n_measurements>::process(measurement_t& measurement) {

  if (!Processor<n_measurements>::enabled_) {
    return false;
  }

  bool ok_flag = true;
  for (int i = 0; i < measurement.rows(); i++) {
    vec_mf_[i].add(measurement(i));
    if (vec_mf_[i].full()) {
      if (!vec_mf_[i].check(measurement(i))) {
        std::stringstream ss_measurement_string;
        ss_measurement_string << measurement(i);
        ss_measurement_string << " ";
        ROS_WARN_THROTTLE(1.0, "[%s]: measurement[%d]: %sdeclined by median filter (median: %.2f, max_diff: %.2f).",
                          Processor<n_measurements>::correction_name_.c_str(), i, ss_measurement_string.str().c_str(), vec_mf_[i].median(), max_diff_);
        ok_flag = false;
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[%s]: median filter not full yet", Processor<n_measurements>::correction_name_.c_str());
      ok_flag = false;
    }
  }

  return ok_flag;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif  // PROCESSORS_PROC_MEDIAN_FILTER_H
