#ifndef ALTITUDEESTIMATOR_H_
#define ALTITUDEESTIMATOR_H_

/* includes //{ */

#include "estimators/partial_estimator.h"

//}

namespace mrs_uav_state_estimation
{

namespace altitude
{
const std::string type = "ALTITUDE";
}

template <int n_states>
class AltitudeEstimator : public PartialEstimator<n_states, 1> {

private:
public:
  AltitudeEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, 1>(altitude::type, name, frame_id){};

  virtual ~AltitudeEstimator(void) {
  }


private:
  static const int _n_axes_   = 1;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;
};

}  // namespace mrs_uav_state_estimation

#endif
