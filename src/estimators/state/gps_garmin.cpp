#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_uav_state_estimators
{

namespace gps_garmin
{

const char estimator_name[] = "gps_garmin";
const bool is_core_plugin   = true;

class GpsGarmin : public StateGeneric {
public:
  GpsGarmin() : StateGeneric(estimator_name, is_core_plugin) {
  }
};

} // namespace gps_garmin

} // namespace mrs_uav_state_estimators


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::gps_garmin::GpsGarmin, mrs_uav_managers::StateEstimator)
