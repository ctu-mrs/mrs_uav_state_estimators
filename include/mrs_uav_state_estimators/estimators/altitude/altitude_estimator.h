#pragma once
#ifndef ESTIMATORS_ALTITUDE_ALTITUDE_ESTIMATOR_H
#define ESTIMATORS_ALTITUDE_ALTITUDE_ESTIMATOR_H

/* includes //{ */

#include <mrs_uav_state_estimators/estimators/partial_estimator.h>

//}

namespace mrs_uav_state_estimators
{

namespace altitude
{
const char type[] = "ALTITUDE";

typedef enum
{
  ODOMETRY,
  RANGE
} MeasurementType_t;

}  // namespace altitude

template <int n_states>
class AltitudeEstimator : public PartialEstimator<n_states, 1> {

protected:
  AltitudeEstimator(const std::string& name, const std::string& frame_id) : PartialEstimator<n_states, 1>(altitude::type, name, frame_id) {
  }

private:
  static const int _n_axes_   = 1;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;
};

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_ALTITUDE_ALTITUDE_ESTIMATOR_H
