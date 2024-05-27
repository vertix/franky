#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"

namespace franky {

/**
 * @class RobotPose
 * @brief A class to represent the cartesian pose of a robot.
 *
 * This class encapsulates the cartesian pose of a robot, which comprises the end effector pose and the elbow position.
 */
class RobotPose {
 public:
  RobotPose();

  RobotPose(const RobotPose &robot_pose);

  // Suppress implicit conversion warning
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-conversion"
  /**
   * @param end_effector_pose The pose of the end effector.
   * @param elbow_position The position of the elbow. Optional.
   */
  RobotPose(Affine end_effector_pose, std::optional<double> elbow_position = std::nullopt);
#pragma clang diagnostic pop

  /**
   * @param vector_repr The vector representation of the pose.
   * @param ignore_elbow Whether to ignore the elbow position. Default is false.
   */
  explicit RobotPose(const Vector7d &vector_repr, bool ignore_elbow = false);

  /**
   * @param vector_repr The vector representation of the pose.
   * @param elbow_position The position of the elbow. Optional.
   */
  explicit RobotPose(const Vector6d &vector_repr, std::optional<double> elbow_position = std::nullopt);

  /**
   * @param franka_pose The franka pose.
   */
  explicit RobotPose(franka::CartesianPose franka_pose);

  /**
   * @brief Get the vector representation of the pose, which consists of the end effector position and orientation
   * (as rotation vector) and the elbow position.
   *
   * @return The vector representation of the pose.
   */
  [[nodiscard]] Vector7d vector_repr() const;

  /**
   * @brief Convert this pose to a franka pose.
   *
   * @return The franka pose.
   */
  [[nodiscard]] franka::CartesianPose as_franka_pose() const;

  /**
   * @brief Transform this pose with a given affine transformation from the left side.
   *
   * @param transform The transform to apply.
   * @return The transformed robot pose.
   */
  [[nodiscard]] inline RobotPose leftTransform(const Affine &transform) const {
    return {transform * end_effector_pose_, elbow_position_};
  }

  /**
   * @brief Transform this pose with a given affine transformation from the right side.
   *
   * @param transform The transform to apply.
   * @return The transformed robot pose.
   */
  [[nodiscard]] inline RobotPose rightTransform(const Affine &transform) const {
    return {end_effector_pose_ * transform, elbow_position_};
  }

  /**
   * @brief Change the frame of the end effector by applying a transformation from the right side. This is equivalent to
   * calling rightTransform(transform).
   *
   * @param transform The transform to apply.
   * @return The robot pose with the new end effector frame.
   */
  [[nodiscard]] inline RobotPose changeEndEffectorFrame(const Affine &transform) const {
    return rightTransform(transform);
  }

  /**
   * @brief Get the pose with a new elbow position.
   *
   * @param elbow_position The new elbow position.
   * @return The pose with the new elbow position.
   */
  [[nodiscard]] inline RobotPose withElbowPosition(const std::optional<double> elbow_position) const {
    return {end_effector_pose_, elbow_position};
  }

  /**
   * @brief Get the end effector pose.
   *
   * @return The end effector pose.
   */
  [[nodiscard]] inline Affine end_effector_pose() const {
    return end_effector_pose_;
  }

  /**
   * @brief Get the elbow position.
   *
   * @return The elbow position.
   */
  [[nodiscard]] inline std::optional<double> elbow_position() const {
    return elbow_position_;
  }

 private:
  Affine end_effector_pose_;
  std::optional<double> elbow_position_;
};

inline RobotPose operator*(const RobotPose &robot_pose, const Affine &right_transform) {
  return robot_pose.rightTransform(right_transform);
}

inline RobotPose operator*(const Affine &left_transform, const RobotPose &robot_pose) {
  return robot_pose.leftTransform(left_transform);
}

}  // namespace franky
