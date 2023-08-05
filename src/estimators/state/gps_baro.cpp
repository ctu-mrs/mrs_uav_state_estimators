#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_uav_state_estimators
{

namespace gps_baro
{

const char estimator_name[] = "gps_baro";

class GpsBaro : public StateGeneric {
public:
  GpsBaro() : StateGeneric(estimator_name) {
  }

  ~GpsBaro(void) {
  }
};

}  // namespace gps_baro
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::gps_baro::GpsBaro, mrs_uav_managers::StateEstimator)
