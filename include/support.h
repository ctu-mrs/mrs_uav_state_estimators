#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */

#include <string.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

//}

namespace mrs_uav_state_estimation
{

/*//{ class Support */
class Support {

public:
  /*//{ toSnakeCase() */
  static std::string toSnakeCase(const std::string& str_in) {

    std::string str(1, tolower(str_in[0]));

    for (auto it = str_in.begin() + 1; it != str_in.end(); ++it) {
      if (isupper(*it) && *(it - 1) != '_' && islower(*(it - 1))) {
        str += "_";
      }
      str += *it;
    }

    std::transform(str.begin(), str.end(), str.begin(), ::tolower);

    return str;
  }
  /*//}*/

  /*//{ stateCovToString() */
  template <typename StateCov>
  static std::string stateCovToString(const StateCov& sc) {
    std::stringstream ss;
    ss << "State:\n";
    for (int i = 0; i < sc.x.rows(); i++) {
      for (int j = 0; j < sc.x.cols(); j++) {
        ss << sc.x(i, j) << " ";
      }
      ss << "\n";
    }
    ss << "Cov:\n";
    for (int i = 0; i < sc.P.rows(); i++) {
      for (int j = 0; j < sc.P.cols(); j++) {
        ss << sc.P(i, j) << " ";
      }
      ss << "\n";
    }
    return ss.str();
  }
  /*//}*/

  /* //{ rotateVecByHdg() */
  static tf2::Vector3 rotateVecByHdg(const geometry_msgs::Vector3& vec_in, const double hdg_in) {

    const tf2::Quaternion q_hdg = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(hdg_in);

    const tf2::Vector3 vec_tf2(vec_in.x, vec_in.y, vec_in.z);

    const tf2::Vector3 vec_rotated = tf2::quatRotate(q_hdg, vec_tf2);

    return vec_rotated;
  }
  //}

  /* noNans() //{ */
  static bool noNans(const geometry_msgs::TransformStamped& tf) {

    return (std::isfinite(tf.transform.rotation.x) && std::isfinite(tf.transform.rotation.y) && std::isfinite(tf.transform.rotation.z) &&
            std::isfinite(tf.transform.rotation.w) && std::isfinite(tf.transform.translation.x) && std::isfinite(tf.transform.translation.y) &&
            std::isfinite(tf.transform.translation.z));
  }
  //}

  /* noNans() //{ */
  static bool noNans(const geometry_msgs::Quaternion& q) {

    return (std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w));
  }
  //}

  /* isZeroQuaternion() //{ */
  static bool isZeroQuaternion(const geometry_msgs::Quaternion& q) {

    return (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0);
  }
  //}

  /* tf2FromPose() //{ */

  static tf2::Transform tf2FromPose(const geometry_msgs::Pose& pose_in) {

    tf2::Vector3 position(pose_in.position.x, pose_in.position.y, pose_in.position.z);

    tf2::Quaternion q;
    tf2::fromMsg(pose_in.orientation, q);

    tf2::Transform tf_out;
    tf_out.setOrigin(position);
    tf_out.setRotation(q);
    tf_out.inverse();

    return tf_out;
  }

  //}

  /* poseFromTf2() //{ */

  static geometry_msgs::Pose poseFromTf2(const tf2::Transform& tf_in) {

    geometry_msgs::Pose pose_out;
    pose_out.position.x = tf_in.getOrigin().getX();
    pose_out.position.y = tf_in.getOrigin().getY();
    pose_out.position.z = tf_in.getOrigin().getZ();

    pose_out.orientation = tf2::toMsg(tf_in.getRotation());

    return pose_out;
  }

  //}

  /* pointToVector3() //{ */

  static geometry_msgs::Vector3 pointToVector3(const geometry_msgs::Point& point_in) {

    geometry_msgs::Vector3 vec_out;
    vec_out.x = point_in.x;
    vec_out.y = point_in.y;
    vec_out.z = point_in.z;

    return vec_out;
  }

  //}

  /*//{ rotateTwist() */
  static geometry_msgs::Vector3 rotateTwist(const geometry_msgs::Vector3& twist_in, const tf2::Quaternion& q_in,
                                            const std::shared_ptr<mrs_lib::Transformer> transformer) {

    geometry_msgs::TransformStamped tf;
    tf.header.stamp            = ros::Time::now();
    tf.header.frame_id         = "fake_id";
    tf.transform.translation.x = 0;
    tf.transform.translation.y = 0;
    tf.transform.translation.z = 0;
    tf.transform.rotation      = tf2::toMsg(q_in);

    geometry_msgs::PointStamped twist_stamped;
    twist_stamped.header.stamp    = ros::Time::now();
    twist_stamped.header.frame_id = "fake_id";
    twist_stamped.point.x         = twist_in.x;
    twist_stamped.point.y         = twist_in.y;
    twist_stamped.point.z         = twist_in.z;

    auto response = transformer->transform(twist_stamped, tf);
    if (response) {
      geometry_msgs::Vector3 twist_out;
      twist_out.x = response.value().point.x;
      twist_out.y = response.value().point.y;
      twist_out.z = response.value().point.z;
      return twist_out;
    } else {
      ROS_ERROR("[support]: could not rotate twist by quaternion");
      return twist_in;
    }
  }
  /*//}*/

  /*//{ loadParamFile() */
  static void loadParamFile(const std::string& file_path, const std::string& ns = "") {
      std::string command = "rosparam load " + file_path + " " + ns;
      int         result  = std::system(command.c_str());
      if (result != 0) {
        ROS_ERROR_STREAM("Could not set config file " << file_path << " to the parameter server.");
      }
  }
  /*//}*/

private:
  Support(){};
};
/*//}*/

}  // namespace mrs_uav_state_estimation

#endif
