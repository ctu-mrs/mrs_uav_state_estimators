#ifndef LATERALESTIMATOR_H_
#define LATERALESTIMATOR_H_

/* includes //{ */

#include "estimators/partial_estimator.h"

//}

namespace mrs_uav_state_estimation
{

namespace lateral
{
const std::string type = "LATERAL";
const int n_axes = 2;
}

template <int n_states>
class LateralEstimator : public PartialEstimator<n_states, lateral::n_axes> {

public:
  LateralEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, lateral::n_axes>(lateral::type, name, frame_id){};

  virtual ~LateralEstimator(void) {
  }

private:
  static const int _n_axes_   = lateral::n_axes;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

};

}  // namespace mrs_uav_state_estimation

#endif
