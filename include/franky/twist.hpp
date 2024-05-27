#pragma once

#include <Eigen/Core>

#include "franky/types.hpp"

namespace franky {

class Twist {
 public:
  Twist() : linear_velocity_(Eigen::Vector3d::Zero()), angular_velocity_(Eigen::Vector3d::Zero()) {}

  Twist(const Twist &twist) = default;

  Twist(Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity)
      : linear_velocity_(std::move(linear_velocity)), angular_velocity_(std::move(angular_velocity)) {}

  explicit Twist(const Vector6d &vector_repr)
      : linear_velocity_(vector_repr.head<3>()), angular_velocity_(vector_repr.tail<3>()) {}

  [[nodiscard]] inline Vector6d vector_repr() const {
    Vector6d result;
    result << linear_velocity_, angular_velocity_;
    return result;
  }

  [[nodiscard]] inline Twist transformWith(const Affine &affine) const {
    return transformWith(affine.rotation());
  }

  template<typename RotationMatrixType>
  [[nodiscard]] inline Twist transformWith(const RotationMatrixType &rotation) const {
    return {rotation * linear_velocity_, rotation * angular_velocity_};
  }

  /// Propagate the twist through a link with the given translation. Hence, suppose this twist is the twist of a frame
  /// A, then this function computes the twist of a frame B that is rigidly attached to frame A by a link with the
  /// given translation: B = A + T, where T is the translation.
  /// \param link_translation: The translation of the link. Must be in the same reference frame as this twist.
  /// \return The twist propagated through the link.
  [[nodiscard]] inline Twist propagateThroughLink(const Eigen::Vector3d &link_translation) const {
    return {linear_velocity_ + angular_velocity_.cross(link_translation), angular_velocity_};
  }

  [[nodiscard]] inline Eigen::Vector3d linear_velocity() const {
    return linear_velocity_;
  }

  [[nodiscard]] inline Eigen::Vector3d angular_velocity() const {
    return angular_velocity_;
  }

 private:
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;
};

inline Twist operator*(const Affine &affine, const Twist &twist) {
  return twist.transformWith(affine);
}

template<typename RotationMatrixType>
inline Twist operator*(const RotationMatrixType &rotation, const Twist &twist) {
  return twist.transformWith(rotation);
}

}  // namespace franky
