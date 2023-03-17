#include "estimators/state/state_generic.h"


namespace mrs_uav_state_estimators
{

namespace aloam
{

const char estimator_name[] = "aloam";

class Aloam : public StateGeneric {
public:
  Aloam() : StateGeneric(estimator_name) {
  }

  ~Aloam(void) {
  }
};

}  // namespace aloam
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::aloam::Aloam, mrs_uav_managers::StateEstimator)
