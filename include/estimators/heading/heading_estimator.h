#ifndef HEADINGESTIMATOR_H_
#define HEADINGESTIMATOR_H_

/* includes //{ */

#include "estimators/partial_estimator.h"

//}

namespace mrs_uav_state_estimation
{

namespace heading
{
const std::string type = "HEADING";
}

template <int n_states>
class HeadingEstimator : public PartialEstimator<n_states, 1> {

private:
public:
  HeadingEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, 1>(heading::type, name, frame_id){};

  virtual ~HeadingEstimator(void) {
  }


private:
  static const int _n_axes_   = 1;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;
};

}  // namespace mrs_uav_state_estimation

#endif
