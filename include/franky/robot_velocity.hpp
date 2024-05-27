#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"
#include "franky/twist.hpp"

namespace franky {

class RobotVelocity {
 public:
  RobotVelocity();

  RobotVelocity(const RobotVelocity &robot_velocity);

  // Suppress implicit conversion warning
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-conversion"
  RobotVelocity(Twist end_effector_twist, double elbow_velocity = 0.0);
#pragma clang diagnostic pop

  explicit RobotVelocity(const Vector7d &vector_repr);

  explicit RobotVelocity(franka::CartesianVelocities franka_velocity);

  [[nodiscard]] Vector7d vector_repr() const;

  [[nodiscard]] franka::CartesianVelocities as_franka_velocity() const;

  [[nodiscard]] inline RobotVelocity transform(const Affine &affine) const {
    return transform(affine.rotation());
  }

  template<typename RotationMatrixType>
  [[nodiscard]] inline RobotVelocity transform(const RotationMatrixType &rotation) const {
    return {rotation * end_effector_twist_, elbow_velocity_};
  }

  /// Change the end-effector frame by adding the given offset to the current end-effector frame. Note that the offset
  /// must be given in world coordinates.
  /// \param offset_world_frame: The offset to add to the current end-effector frame.
  /// \return The velocity of the new end-effector frame.
  [[nodiscard]] inline RobotVelocity changeEndEffectorFrame(const Eigen::Vector3d &offset_world_frame) const {
    return {end_effector_twist_.propagateThroughLink(offset_world_frame), elbow_velocity_};
  }

  [[nodiscard]] inline Twist end_effector_twist() const {
    return end_effector_twist_;
  }

  [[nodiscard]] inline std::optional<double> elbow_velocity() const {
    return elbow_velocity_;
  }

 private:
  Twist end_effector_twist_;
  double elbow_velocity_ = 0.0;
};

inline RobotVelocity operator*(const Affine &affine, const RobotVelocity &robot_velocity) {
  return robot_velocity.transform(affine);
}

template<typename RotationMatrixType>
inline RobotVelocity operator*(const RotationMatrixType &rotation, const RobotVelocity &robot_velocity) {
  return robot_velocity.transform(rotation);
}

}  // namespace franky
