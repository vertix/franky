#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"

namespace franky {

class RobotVelocity {
 public:
  RobotVelocity();

  RobotVelocity(const RobotVelocity &robot_velocity);

  RobotVelocity(
      Eigen::Vector3d end_effector_linear_velocity,
      Eigen::Vector3d end_effector_angular_velocity,
      std::optional<double> elbow_velocity = std::nullopt);

  explicit RobotVelocity(const Vector7d &vector_repr, bool ignore_elbow = false);

  explicit RobotVelocity(const Vector6d &vector_repr, std::optional<double> elbow_velocity = std::nullopt);

  explicit RobotVelocity(franka::CartesianVelocities franka_velocity);

  [[nodiscard]] Vector7d vector_repr() const;

  [[nodiscard]] franka::CartesianVelocities as_franka_velocity() const;

  [[nodiscard]] inline RobotVelocity transform(const Affine &transform) const {
    return RobotVelocity(
        transform * end_effector_linear_velocity_, transform * end_effector_angular_velocity_, elbow_velocity_);
  }

  [[nodiscard]] inline RobotVelocity with_elbow_velocity(const std::optional<double> elbow_velocity) const {
    return RobotVelocity(end_effector_linear_velocity_, end_effector_angular_velocity_, elbow_velocity);
  }

  [[nodiscard]] inline Eigen::Vector3d end_effector_linear_velocity() const {
    return end_effector_linear_velocity_;
  }

  [[nodiscard]] inline Eigen::Vector3d end_effector_angular_velocity() const {
    return end_effector_angular_velocity_;
  }

  [[nodiscard]] inline std::optional<double> elbow_velocity() const {
    return elbow_velocity_;
  }

 private:
  Eigen::Vector3d end_effector_linear_velocity_;
  Eigen::Vector3d end_effector_angular_velocity_;
  std::optional<double> elbow_velocity_;
};

inline RobotVelocity operator*(const Affine &transform, const RobotVelocity &robot_velocity) {
  return robot_velocity.transform(transform);
}

}  // namespace franky
