#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_uav_state_estimators
{

namespace vio
{

const char estimator_name[] = "vio";
const bool is_core_plugin = true;

class Vio : public StateGeneric {
public:
  Vio() : StateGeneric(estimator_name, is_core_plugin) {
  }

  ~Vio(void) {
  }
};

}  // namespace vio
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::vio::Vio, mrs_uav_managers::StateEstimator)
