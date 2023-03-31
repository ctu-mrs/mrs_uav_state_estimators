#include "estimators/state/state_generic.h"


namespace mrs_uav_state_estimators
{

namespace hector_garmin
{

const char estimator_name[] = "hector_garmin";

class HectorGarmin : public StateGeneric {
public:
  HectorGarmin() : StateGeneric(estimator_name) {
  }

  ~HectorGarmin(void) {
  }
};

}  // namespace hector_garmin
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::hector_garmin::HectorGarmin, mrs_uav_managers::StateEstimator)
