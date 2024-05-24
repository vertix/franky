#pragma once

#include <optional>
#include <Eigen/Core>

#include "franky/robot_pose.hpp"
#include "franky/robot_velocity.hpp"

namespace franky {

class CartesianState {
 public:
  // Allows for implicit conversion from RobotPose to CartesianState
  CartesianState(RobotPose pose) : pose_(std::move(pose)) {}

  CartesianState(RobotPose pose, std::optional<RobotVelocity> velocity)
      : pose_(std::move(pose)), velocity_(std::move(velocity)) {}

  CartesianState(RobotPose pose, RobotVelocity velocity)
      : CartesianState(std::move(pose), std::optional(velocity)) {}

  CartesianState(const CartesianState &) = default;

  CartesianState() = default;

  [[nodiscard]] inline CartesianState transform(const Affine &transform) const {
    return CartesianState(
        transform * pose_, velocity_.has_value() ? std::optional(transform * velocity_.value()) : std::nullopt);
  }

  [[nodiscard]] inline CartesianState with_velocity(const std::optional<RobotVelocity> velocity) const {
    return CartesianState(pose_, velocity_);
  }

  [[nodiscard]] inline RobotPose pose() const {
    return pose_;
  }

  [[nodiscard]] inline std::optional<RobotVelocity> velocity() const {
    return velocity_;
  }

 private:
  RobotPose pose_;
  std::optional<RobotVelocity> velocity_ = std::nullopt;
};

inline CartesianState operator*(const Affine &transform, const CartesianState &cartesian_state) {
  return cartesian_state.transform(transform);
}

}  // namespace franky
