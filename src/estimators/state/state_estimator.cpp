#include "estimators/state/state_estimator.h"

namespace mrs_uav_state_estimation
{

/*//{ publishUavState() */
void StateEstimator::publishUavState() const {

  try {
    pub_uav_state_.publish(uav_state_);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_uav_state_.getTopic().c_str());
  }
}

}  // namespace mrs_uav_state_estimation

/*//}*/
