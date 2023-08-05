#ifndef ESTIMATORS_HEADING_HEADING_ESTIMATOR_H
#define ESTIMATORS_HEADING_HEADING_ESTIMATOR_H

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/partial_estimator.h>

//}

namespace mrs_uav_state_estimators
{

namespace heading
{
const char type[] = "HEADING";
}

template <int n_states>
class HeadingEstimator : public PartialEstimator<n_states, 1> {

protected:
  HeadingEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, 1>(heading::type, name, frame_id) {
  }

  mutable std::mutex mutex_last_valid_hdg_;
  double             last_valid_hdg_;

private:
  static const int _n_axes_   = 1;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

public:
  virtual double getLastValidHdg() const = 0;
};

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_HEADING_HEADING_ESTIMATOR_H
