#pragma once
#ifndef PROCESSORS_PROC_MEDIAN_FILTER_H
#define PROCESSORS_PROC_MEDIAN_FILTER_H

#include <mrs_uav_state_estimators/processors/processor.h>

#include <mrs_lib/median_filter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_uav_managers/estimation_manager/support.h>

#include <rclcpp/rclcpp.hpp>

#include <limits>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class ProcMedianFilter : public Processor<n_measurements> {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  ProcMedianFilter(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name,
                   const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph);

  std::tuple<bool, bool> process(measurement_t& measurement) override;
  void                   reset();

private:
  std::vector<mrs_lib::MedianFilter> vec_mf_;
  int                                buffer_size_;
  double                             max_diff_;
};

/*//{ constructor */
template <int n_measurements>
ProcMedianFilter<n_measurements>::ProcMedianFilter(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name,
                                                   const std::shared_ptr<CommonHandlers_t>& ch, const std::shared_ptr<PrivateHandlers_t>& ph)
    : Processor<n_measurements>(node, correction_name, name, ch, ph) {

  // | --------------------- load parameters -------------------- |
  /* ph->param_loader->setPrefix(ch->package_name + "/" + Support::toSnakeCase(ch->nodelet_name) + "/" + Processor<n_measurements>::getNamespacedName() + "/"); */

  ph->param_loader->loadParam("buffer_size", buffer_size_);
  ph->param_loader->loadParam("max_diff", max_diff_);

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(this->node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.",
                 Processor<n_measurements>::getPrintName().c_str());
    rclcpp::shutdown();
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
std::tuple<bool, bool> ProcMedianFilter<n_measurements>::process(measurement_t& measurement) {

  if (!Processor<n_measurements>::enabled_) {
    return {true, true};
  }

  bool ok_flag     = true;
  bool should_fuse = true;
  for (int i = 0; i < measurement.rows(); i++) {
    vec_mf_[i].add(measurement(i));
    if (vec_mf_[i].full()) {
      if (!vec_mf_[i].check(measurement(i))) {
        std::stringstream ss_measurement_string;
        ss_measurement_string << measurement(i);
        ss_measurement_string << " ";
        RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->clock_, 1000,
                             "[%s]: measurement[%d]: %s declined by median filter (median: %.2f, max_diff: %.2f).",
                             Processor<n_measurements>::getPrintName().c_str(), i, ss_measurement_string.str().c_str(), vec_mf_[i].median(), max_diff_);
        ok_flag     = false;
        should_fuse = false;
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->clock_, 1000, "[%s]: median filter not full yet",
                           Processor<n_measurements>::getPrintName().c_str());
      ok_flag     = false;
      should_fuse = false;
    }
  }

  return {ok_flag, should_fuse};
}
/*//}*/

/*//{ reset() */
template <int n_measurements>
void ProcMedianFilter<n_measurements>::reset() {
  for (auto mf : vec_mf_) {
    mf.clear();
  }
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROC_MEDIAN_FILTER_H
