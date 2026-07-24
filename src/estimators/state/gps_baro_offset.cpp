#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/Float64Stamped.h>

namespace mrs_uav_state_estimators
{

namespace gps_baro_offset
{

const char estimator_name[] = "gps_baro_offset";
const bool is_core_plugin = true;

// same as GpsBaro, except the published z is offset so that it reads zero at the point where the UAV
// was last armed (never while in offboard mode, i.e. never while the UAV could be in the air)
class GpsBaroOffset : public StateGeneric {
public:
  GpsBaroOffset() : StateGeneric(estimator_name, is_core_plugin) {
  }

  ~GpsBaroOffset(void) {
  }

  void initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;

  void updateUavState();

private:
  std::string                                      topic_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
  void                                              callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg);

  mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped> ph_z_offset_;

  std::atomic<bool>   is_first_status_msg_ = true;
  std::atomic<bool>   is_armed_prev_       = false;
  std::atomic<double> z_raw_last_          = 0.0;  // latest unoffset altitude estimate, updated every updateUavState() tick
  std::atomic<double> z_offset_            = 0.0;  // z_raw_last_ captured at the last disarmed -> armed edge
};

/* initialize() //{ */
void GpsBaroOffset::initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  StateGeneric::initialize(parent_nh, ch, ph);

  ros::NodeHandle nh(parent_nh);

  ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  std::string topic_hw_api_status;
  ph->param_loader->loadParam("topics/hw_api_status", topic_hw_api_status);
  topic_hw_api_status_ = "/" + ch_->uav_name + "/" + topic_hw_api_status;

  if (!ph->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    ros::shutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, topic_hw_api_status_, &GpsBaroOffset::callbackHwApiStatus, this);

  if (ch_->debug_topics.diag) {
    ph_z_offset_ = mrs_lib::PublisherHandler<mrs_msgs::Float64Stamped>(nh, Support::toSnakeCase(getName()) + "/z_offset", 10);
  }
}
/*//}*/

/* updateUavState() //{ */
void GpsBaroOffset::updateUavState() {

  StateGeneric::updateUavState();

  const double offset = z_offset_.load();

  auto uav_state = mrs_lib::get_mutexed(mtx_uav_state_, uav_state_);
  auto odom      = mrs_lib::get_mutexed(mtx_odom_, odom_);

  // StateGeneric::updateUavState() always (re)computes z from scratch from the raw altitude estimate,
  // so this is never affected by the offset subtracted below
  z_raw_last_ = uav_state.pose.position.z;

  uav_state.pose.position.z -= offset;
  odom.pose.pose.position.z -= offset;

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);

  if (ch_->debug_topics.diag) {
    mrs_msgs::Float64Stamped offset_msg;
    offset_msg.header.stamp    = uav_state.header.stamp;
    offset_msg.header.frame_id = ns_frame_id_;
    offset_msg.value           = offset;
    ph_z_offset_.publish(offset_msg);
  }
}
/*//}*/

/* callbackHwApiStatus() //{ */
void GpsBaroOffset::callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg) {

  if (!isInitialized()) {
    return;
  }

  const bool armed_now = msg->armed;

  // the very first received message only initializes is_armed_prev_ - it must not be treated as an
  // arming edge, otherwise (re)starting this estimator while already armed and airborne would zero
  // the altitude mid-flight
  if (is_first_status_msg_) {
    is_first_status_msg_ = false;
    is_armed_prev_       = armed_now;
    return;
  }

  // capture the offset only on the disarmed -> armed edge, and never while in offboard mode (i.e. never
  // while the UAV could be in the air under the control of this system)
  if (armed_now && !is_armed_prev_ && !msg->offboard) {
    const double offset = z_raw_last_.load();
    z_offset_           = offset;
    ROS_INFO("[%s]: captured altitude offset of %.3f m at arming", getPrintName().c_str(), offset);
  }

  is_armed_prev_ = armed_now;
}
/*//}*/

}  // namespace gps_baro_offset
}  // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::gps_baro_offset::GpsBaroOffset, mrs_uav_managers::StateEstimator)
