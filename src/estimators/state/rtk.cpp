#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_uav_state_estimators
{

namespace rtk
{

const char estimator_name[] = "rtk";

class Rtk : public StateGeneric {
public:
  Rtk() : StateGeneric(estimator_name) {
  }

  ~Rtk(void) {
  }
};

}  // namespace rtk
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::rtk::Rtk, mrs_uav_managers::StateEstimator)
