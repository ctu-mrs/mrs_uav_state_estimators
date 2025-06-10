#include <gtest/gtest.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <string>
#include <vector>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>

#include <mrs_lib/transformer.h>

#include <mrs_msgs/RtkGps.h>

#include <mrs_uav_state_estimators/estimators/correction.h>
#include <mrs_uav_managers/estimation_manager/common_handlers.h>

namespace mrs_uav_state_estimation
{

struct DataPoint_t
{
  double fcu_qx;
  double fcu_qy;
  double fcu_qz;
  double fcu_qw;
  double antenna_z;
  double meas_x;
  double meas_y;
};

const int                                        n_measurements = 2;
typedef Eigen::Matrix<double, n_measurements, 1> measurement_t;
typedef Eigen::Matrix<double, 6, 1>              Matrix6d;

/* correctionTfExpected() //{ */

measurement_t correctionTfExpected(const DataPoint_t& data) {

  measurement_t result;

  tf2::Quaternion q(data.fcu_qx, data.fcu_qy, data.fcu_qz, data.fcu_qw);
  tf2::Matrix3x3  rot_mat(q);
  tf2::Vector3    meas(data.meas_x, data.meas_y, 0);
  tf2::Vector3    antenna_pos(0, 0, data.antenna_z);

  tf2::Vector3 res_vec = meas - rot_mat * antenna_pos;
  result(0)            = res_vec.getX();
  result(1)            = res_vec.getY();
  return result;
}

//}

/*//{ correctionTfTestImpl() */
int correctionTfTestImpl(const std::vector<double> data_vec) {

  int               result = 1;
  const std::string name   = "corrections_test";
  ros::NodeHandle   nh;

  std::shared_ptr<mrs_uav_managers::estimation_manager::CommonHandlers_t> ch = std::make_shared<mrs_uav_managers::estimation_manager::CommonHandlers_t>();
  ch->transformer                      = std::make_shared<mrs_lib::Transformer>(nh, name);
  ch->transformer->retryLookupNewest(true);
  ch->frames.fcu             = "fcu";
  ch->frames.ns_fcu          = ch->frames.fcu;
  ch->frames.fcu_untilted    = "fcu_untilted";
  ch->frames.ns_fcu_untilted = ch->frames.fcu_untilted;
  ch->frames.rtk_antenna     = "rtk_antenna";
  ch->frames.ns_rtk_antenna  = ch->frames.rtk_antenna;

  ros::Publisher pub_rtk_msg = nh.advertise<mrs_msgs::RtkGps>("/rtk_test/rtk_msg", 1);
  ros::Publisher pub_tf      = nh.advertise<tf2_msgs::TFMessage>("tf", 1);

  // | -------------- initialize the async spinner -------------- |

  /* ros::AsyncSpinner spinner(10); */
  /* spinner.start(); */

  mrs_uav_state_estimators::Correction<n_measurements> correction(nh, name, "lat_rtk/pos_rtk", "uav1/rtk_origin", mrs_uav_state_estimators::EstimatorType_t::LATERAL, ch);

  DataPoint_t data;
  data.fcu_qx    = data_vec[0];
  data.fcu_qy    = data_vec[1];
  data.fcu_qz    = data_vec[2];
  data.fcu_qw    = data_vec[3];
  data.antenna_z = data_vec[4];
  data.meas_x    = data_vec[5];
  data.meas_y    = data_vec[6];


  const measurement_t expected = correctionTfExpected(data);
  measurement_t       output;

  mrs_msgs::RtkGps rtk_msg;
  rtk_msg.header.stamp            = ros::Time::now();
  rtk_msg.header.frame_id         = "utm";
  rtk_msg.pose.pose.position.x    = data.meas_x;
  rtk_msg.pose.pose.position.y    = data.meas_y;
  rtk_msg.pose.pose.orientation.w = 1;
  pub_rtk_msg.publish(rtk_msg);
  ros::spinOnce();

  geometry_msgs::TransformStamped tf_fcu_fcu_untilted;
  tf_fcu_fcu_untilted.header.stamp    = ros::Time::now();
  tf_fcu_fcu_untilted.header.frame_id = "fcu";
  tf_fcu_fcu_untilted.child_frame_id  = "fcu_untilted";
  tf2::Quaternion q_fcu;
  tf_fcu_fcu_untilted.transform.rotation.x = data.fcu_qx;
  tf_fcu_fcu_untilted.transform.rotation.y = data.fcu_qy;
  tf_fcu_fcu_untilted.transform.rotation.z = data.fcu_qz;
  tf_fcu_fcu_untilted.transform.rotation.w = data.fcu_qw;

  geometry_msgs::TransformStamped tf_fcu_antenna;
  tf_fcu_antenna.header.stamp            = ros::Time::now();
  tf_fcu_antenna.header.frame_id         = "fcu";
  tf_fcu_antenna.child_frame_id          = "rtk_antenna";
  tf_fcu_antenna.transform.translation.z = data.antenna_z;
  tf_fcu_antenna.transform.rotation.w    = 1;

  tf2_msgs::TFMessage tf_msg;
  tf_msg.transforms.push_back(tf_fcu_fcu_untilted);
  tf_msg.transforms.push_back(tf_fcu_antenna);
  pub_tf.publish(tf_msg);
  ros::spinOnce();
  /* ros::spinOnce(); */

  /* while (!correction.isHealthy()) { */
  /*   ros::spinOnce(); */
  /* } */

  while (!ch->transformer->getTransform(ch->frames.ns_fcu, ch->frames.ns_rtk_antenna, ros::Time::now())) {
    ROS_INFO_THROTTLE(1.0, "Waiting for TF");
    ros::spinOnce();
    /* result = 0; */
  }

  while (!correction.getCorrection(output)) {
    ROS_INFO_THROTTLE(1.0, "Could not obtain correction!\n");
    ros::spinOnce();
    /* result = 0; */
  }

  /* ROS_INFO("correction test #%d for values: meas_x: %.2f, meas_y: %.2f, antenna_z %.2f, fcu_r: %.2f, fcu_p: %.2f, fcu_y: %.2f\n", 1, data.meas_x, */
  /*          data.meas_y, data.antenna_z, data.fcu_r, data.fcu_p, data.fcu_y); */
  ROS_INFO("correction test #%d for values: meas_x: %.2f, meas_y: %.2f, antenna_z %.2f, fcu_qx: %.2f, fcu_qy: %.2f, fcu_qz: %.2f, fcu_qw: %.2f\n", 1,
           data.meas_x, data.meas_y, data.antenna_z, data.fcu_qx, data.fcu_qy, data.fcu_qz, data.fcu_qw);
  ROS_INFO("output: %.2f %.2f expected: %.2f %.2f\n", output(0), output(1), expected(0), expected(1));

  for (int i = 0; i < n_measurements; i++) {
    if (fabs(output(i) - expected(i)) > 1e-6) {
      result = 0;
    }
  }

  if (result) {
    ROS_INFO("PASSED\n");
  } else {
    ROS_INFO("FAILED\n");
  }
  ROS_INFO("-----------\n");
  return result;
}
/*//}*/

}  // namespace mrs_uav_state_estimation

/* TEST(TESTSuite, correctionTf) //{ */

TEST(TESTSuite, correctionTf) {

  const std::string    name = "corrections_test";
  ros::NodeHandle      nh;
  mrs_lib::ParamLoader param_loader(nh, name);
  param_loader.setPrefix(name + "/");
  /* int n; */
  /* param_loader.loadParam("test_data_n", n); */
  std::vector<double> test_data;
  param_loader.loadParam("test_data", test_data);
  std::vector<double> data;
  for (int i = 0; i < test_data.size(); i++) {
    data.push_back(test_data.at(i));
    if (data.size() > 6) {
      int result = mrs_uav_state_estimation::correctionTfTestImpl(data);
      EXPECT_TRUE(result);
      data.clear();
    }
  }

  for (int i = 0; i < 10; i++) {
    tf2::Quaternion q;
    double          low  = -1;
    double          high = 1;
    q.setX(low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low))));
    q.setY(low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low))));
    q.setZ(low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low))));
    q.setW(low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low))));
    q.normalize();
    data.push_back(q.getX());
    data.push_back(q.getY());
    data.push_back(q.getZ());
    data.push_back(q.getW());
    for (int j = 0; j < 3; j++) {
      const double low  = -10000;
      const double high = 10000;
      const double num  = low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low)));
      data.push_back(num);
    }
    if (data.size() > 6) {
      int result = mrs_uav_state_estimation::correctionTfTestImpl(data);
      EXPECT_TRUE(result);
      data.clear();
    }
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test_corrections");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();  // To show debug output in the tests
  }

  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

