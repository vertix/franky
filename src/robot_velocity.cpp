#include "franky/robot_velocity.hpp"

#include <optional>
#include <Eigen/Core>
#include <utility>
#include <franka/control_types.h>

#include "franky/types.hpp"

namespace franky {

RobotVelocity::RobotVelocity(const RobotVelocity &) = default;

RobotVelocity::RobotVelocity(
    Eigen::Vector3d end_effector_linear_velocity,
    Eigen::Vector3d end_effector_angular_velocity,
    std::optional<double> elbow_velocity)
    : end_effector_linear_velocity_(std::move(end_effector_linear_velocity)),
      end_effector_angular_velocity_(std::move(end_effector_angular_velocity)),
      elbow_velocity_(elbow_velocity) {}

RobotVelocity::RobotVelocity(const Vector7d &vector_repr, bool ignore_elbow)
    : RobotVelocity(
    vector_repr.head<6>(),
    ignore_elbow ? std::optional<double>(std::nullopt) : vector_repr[6]) {}

RobotVelocity::RobotVelocity(const Vector6d &vector_repr, std::optional<double> elbow_velocity)
    : end_effector_linear_velocity_(vector_repr.head<3>()),
      end_effector_angular_velocity_(vector_repr.tail<3>()),
      elbow_velocity_(elbow_velocity) {}

RobotVelocity::RobotVelocity(const franka::CartesianVelocities franka_velocity)
    : RobotVelocity(Vector6d::Map(franka_velocity.O_dP_EE.data()), std::optional<double>(franka_velocity.elbow[0])) {}

Vector7d RobotVelocity::vector_repr() const {
  Vector7d result;
  result << end_effector_linear_velocity_, end_effector_angular_velocity_, elbow_velocity_.value_or(0.0);
  return result;
}

franka::CartesianVelocities RobotVelocity::as_franka_velocity() const {
  std::array<double, 6> array;
  auto vec = vector_repr().head<6>();
  std::copy(vec.data(), vec.data() + array.size(), array.begin());
  if (elbow_velocity_.has_value()) {
    return franka::CartesianVelocities(array, {elbow_velocity_.value(), -1});
  }
  return {array};
}

RobotVelocity::RobotVelocity()
    : end_effector_linear_velocity_(Eigen::Vector3d::Zero()),
      end_effector_angular_velocity_(Eigen::Vector3d::Zero()),
      elbow_velocity_(std::nullopt) {}

}  // namespace franky
