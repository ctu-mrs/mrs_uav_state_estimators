#pragma once
#ifndef PROCESSORS_PROCESSOR_H
#define PROCESSORS_PROCESSOR_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>
#include <mrs_uav_managers/estimation_manager/private_handlers.h>

namespace mrs_uav_state_estimators
{

using namespace mrs_uav_managers::estimation_manager;

template <int n_measurements>
class Processor {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  std::string getName() const;
  std::string getNamespacedName() const;
  std::string getPrintName() const;

  void toggle(bool enable);

  virtual std::tuple<bool, bool> process(measurement_t& measurement) = 0;
  virtual void                   reset()                             = 0;

protected:
  Processor(const rclcpp::Node::SharedPtr& node, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch,
            const std::shared_ptr<PrivateHandlers_t>& ph)
      : correction_name_(correction_name), name_(name), ch_(ch), ph_(ph) {
    node_  = node;
    clock_ = node->get_clock();
  }  // protected constructor to prevent instantiation

  const std::string correction_name_;
  const std::string name_;

  const std::shared_ptr<CommonHandlers_t>  ch_;
  const std::shared_ptr<PrivateHandlers_t> ph_;

  bool enabled_       = true;
  bool start_enabled_ = true;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
};

/*//{ getName() */
template <int n_measurements>
std::string Processor<n_measurements>::getName() const {
  return name_;
}
/*//}*/

/*//{ getNamespacedName() */
template <int n_measurements>
std::string Processor<n_measurements>::getNamespacedName() const {
  return correction_name_ + "/" + name_;
}
/*//}*/

/*//{ getPrintName() */
template <int n_measurements>
std::string Processor<n_measurements>::getPrintName() const {
  return ch_->nodelet_name + "/" + correction_name_ + "/" + name_;
}
/*//}*/

/*//{ toggle() */
template <int n_measurements>
void Processor<n_measurements>::toggle(bool enable) {
  enabled_ = enable;
}
/*//}*/

}  // namespace mrs_uav_state_estimators

#endif  // PROCESSORS_PROCESSOR_H
