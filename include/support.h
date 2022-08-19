#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */

#include <string.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Vector3.h>

#include <mrs_lib/attitude_converter.h>

//}

namespace mrs_uav_state_estimation
{

/*//{ toSnakeCase() */
std::string toSnakeCase(const std::string& str_in) {

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
std::string stateCovToString(const StateCov& sc) {
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

/* //{ getRotatedVector() */
tf2::Vector3 rotateVecByHdg(const geometry_msgs::Vector3& acc_in, const double hdg_in) {

  const tf2::Quaternion q_hdg = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(hdg_in);

  const tf2::Vector3 acc_tf2(acc_in.x, acc_in.y, acc_in.z);

  const tf2::Vector3 acc_rotated = quatRotate(q_hdg, acc_tf2);

  /* geometry_msgs::Vector3 acc_out; */
  /* acc_out.x = acc_rotated.getX(); */
  /* acc_out.y = acc_rotated.getY(); */
  /* acc_out.z = acc_rotated.getZ(); */

  return acc_rotated;
}
//}

}  // namespace mrs_uav_state_estimation

#endif
