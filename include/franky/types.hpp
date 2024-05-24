#pragma once

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

namespace franky {

using Vector6d = Eigen::Vector<double, 6>;
using Vector7d = Eigen::Vector<double, 7>;

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using Affine = Eigen::Affine3d;

class timeout_exception : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}