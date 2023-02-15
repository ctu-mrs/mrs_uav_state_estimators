#pragma once
#ifndef PROCESSORS_PROCESSOR_H
#define PROCESSORS_PROCESSOR_H

namespace mrs_uav_state_estimation
{

template <int n_measurements>
class Processor {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  std::string getName() const;
  std::string getNamespacedName() const;

  void toggle(bool enable);

  virtual bool process(measurement_t& measurement) = 0;

protected:
  Processor(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch)
      : correction_name_(correction_name), name_(name), ch_(ch) {
  }  // protected constructor to prevent instantiation

  const std::string correction_name_;
  const std::string name_;

  const std::shared_ptr<CommonHandlers_t> ch_;

  bool enabled_ = true;
  bool start_enabled_ = true;
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

/*//{ toggle() */
template <int n_measurements>
void Processor<n_measurements>::toggle(bool enable) {
  enabled_ = enable;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif  // PROCESSORS_PROCESSOR_H
