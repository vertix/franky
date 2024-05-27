#pragma once

#include <optional>
#include <Eigen/Core>

#include "franky/robot_pose.hpp"
#include "franky/robot_velocity.hpp"

namespace franky {

class CartesianState {
 public:
  // Suppress implicit conversion warning
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-conversion"
  CartesianState(const RobotPose &pose) : pose_(pose) {}
#pragma clang diagnostic pop

  CartesianState(const RobotPose &pose, const RobotVelocity &velocity)
      : pose_(pose), velocity_(velocity) {}

  CartesianState(const CartesianState &) = default;

  CartesianState() = default;

  [[nodiscard]] inline CartesianState transformWith(const Affine &transform) const {
    return {transform * pose_, transform * velocity_};
  }

  [[nodiscard]] CartesianState changeEndEffectorFrame(const Affine &transform) const {
    auto offset_world_frame = pose_.end_effector_pose() * transform.translation();
    return {pose_.changeEndEffectorFrame(transform), velocity_.changeEndEffectorFrame(offset_world_frame)};
  }

  [[nodiscard]] inline RobotPose pose() const {
    return pose_;
  }

  [[nodiscard]] inline RobotVelocity velocity() const {
    return velocity_;
  }

 private:
  RobotPose pose_;
  RobotVelocity velocity_;
};

inline CartesianState operator*(const Affine &transform, const CartesianState &cartesian_state) {
  return cartesian_state.transformWith(transform);
}

}  // namespace franky
