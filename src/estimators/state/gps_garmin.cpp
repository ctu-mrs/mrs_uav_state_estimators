#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_uav_state_estimators
{

namespace gps_garmin
{

const char estimator_name[] = "gps_garmin";

class GpsGarmin : public StateGeneric {
public:
  GpsGarmin() : StateGeneric(estimator_name) {
  }

  ~GpsGarmin(void) {
  }
};

}  // namespace gps_garmin
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::gps_garmin::GpsGarmin, mrs_uav_managers::StateEstimator)
