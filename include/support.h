#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */

//}

namespace mrs_uav_state_estimation
{

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

}  // namespace mrs_uav_state_estimation

#endif
