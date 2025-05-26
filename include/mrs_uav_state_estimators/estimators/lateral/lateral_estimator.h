#ifndef ESTIMATORS_LATERAL_LATERAL_ESTIMATOR_H
#define ESTIMATORS_LATERAL_LATERAL_ESTIMATOR_H

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/partial_estimator.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace mrs_uav_state_estimators
{

namespace lateral
{
const char type[] = "LATERAL";
const int  n_axes = 2;
}  // namespace lateral

template <int n_states>
class LateralEstimator : public PartialEstimator<n_states, lateral::n_axes> {

protected:
  LateralEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, lateral::n_axes>(lateral::type, name, frame_id) {
  }

private:
  static const int _n_axes_   = lateral::n_axes;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;
};

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_LATERAL_LATERAL_ESTIMATOR_H
