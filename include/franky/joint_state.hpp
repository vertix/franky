#pragma once

#include <optional>
#include <Eigen/Core>
#include <utility>

#include "franky/robot_pose.hpp"
#include "franky/robot_velocity.hpp"

namespace franky {

class JointState {
 public:
  // Suppress implicit conversion warning
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-conversion"
  JointState(Vector7d position) : position_(std::move(position)), velocity_(Vector7d::Zero()) {}
#pragma clang diagnostic pop

  JointState(Vector7d pose, Vector7d velocity)
      : position_(std::move(pose)), velocity_(std::move(velocity)) {}

  JointState(const JointState &) = default;

  JointState() = default;

  [[nodiscard]] inline Vector7d position() const {
    return position_;
  }

  [[nodiscard]] inline Vector7d velocity() const {
    return velocity_;
  }

 private:
  Vector7d position_;
  Vector7d velocity_;
};

}  // namespace franky
