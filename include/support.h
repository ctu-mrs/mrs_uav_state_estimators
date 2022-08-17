#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */

#include <string.h>

//}

namespace mrs_uav_state_estimation
{

  std::string toSnakeCase(const std::string& str_in) {

    std::string str(1, tolower(str_in[0]));

    for (auto it = str_in.begin() + 1; it != str_in.end(); ++it) {
      if (isupper(*it) && *(it-1) != '_' && islower(*(it-1))) {
        str += "_";
      }
      str += *it;
    }

    std::transform(str.begin(), str.end(), str.begin(), ::tolower);

    return str;
  }

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
