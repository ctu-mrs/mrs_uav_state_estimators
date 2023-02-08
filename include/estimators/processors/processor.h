#pragma once
#ifndef ESTIMATORS_PROCESSORS_PROCESSOR_H
#define ESTIMATORS_PROCESSORS_PROCESSOR_H

namespace mrs_uav_state_estimation
{

template <int n_measurements>
class Processor {

public:
  typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;

public:
  std::string getName() const;
  std::string getNamespacedName() const;

  virtual bool process(measurement_t& measurement) = 0;

protected:
  Processor(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const std::shared_ptr<CommonHandlers_t>& ch)
      : correction_name_(correction_name), name_(name), ch_(ch) {
  }  // protected constructor to prevent instantiation

  const std::string correction_name_;
  const std::string name_;

  const std::shared_ptr<CommonHandlers_t> ch_;
};

/*//{ constructor */
/* template <typename measurement_t> */
/* Processor<measurement_t>::Processor(ros::NodeHandle& nh, const std::string& correction_name, const std::string& name, const
 * std::shared_ptr<CommonHandlers_t>& ch) : correction_name_(correction_name), name_(name), ch_(ch) { */

/* } */
/*//}*/

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

}  // namespace mrs_uav_state_estimation

#endif  // ESTIMATORS_PROCESSORS_PROCESSOR_H
