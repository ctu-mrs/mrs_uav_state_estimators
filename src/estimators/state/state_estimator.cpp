#include "estimators/state/state_estimator.h"

namespace mrs_uav_state_estimation
{

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {
  std::scoped_lock lock(mtx_uav_state_);
  ph_uav_state_.publish(uav_state_);
}
/*//}*/

/*//{ publishInnovation() */
void StateEstimator::publishInnovation() const {

  std::scoped_lock lock(mtx_innovation_);
  ph_innovation_.publish(innovation_);
}
/*//}*/

}  // namespace mrs_uav_state_estimation

