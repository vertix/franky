#include "franky/robot_velocity.hpp"

#include <optional>
#include <Eigen/Core>
#include <utility>
#include <franka/control_types.h>

#include "franky/types.hpp"

namespace franky {

RobotVelocity::RobotVelocity() = default;

RobotVelocity::RobotVelocity(const RobotVelocity &) = default;

RobotVelocity::RobotVelocity(const Twist &end_effector_twist, double elbow_velocity)
    : end_effector_twist_(end_effector_twist),
      elbow_velocity_(elbow_velocity) {}

RobotVelocity::RobotVelocity(const Vector7d &vector_repr)
    : RobotVelocity(Twist::fromVectorRepr(vector_repr.head<6>()), vector_repr[6]) {}

RobotVelocity::RobotVelocity(const franka::CartesianVelocities franka_velocity)
    : RobotVelocity(
    Twist{
        Vector6d::Map(franka_velocity.O_dP_EE.data()).tail<3>(),
        Vector6d::Map(franka_velocity.O_dP_EE.data()).head<3>()
    }, franka_velocity.elbow[0]) {}

Vector7d RobotVelocity::vector_repr() const {
  Vector7d result;
  result << end_effector_twist_.vector_repr(), elbow_velocity_;
  return result;
}

franka::CartesianVelocities RobotVelocity::as_franka_velocity() const {
  std::array<double, 6> array;
  auto vec = vector_repr().head<6>();
  std::copy(vec.data(), vec.data() + array.size(), array.begin());
  return franka::CartesianVelocities(array, {elbow_velocity_, -1});
}

}  // namespace franky
