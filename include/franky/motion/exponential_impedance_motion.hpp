#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/robot_pose.hpp"
#include "franky/motion/impedance_motion.hpp"

namespace franky {

/**
 * @brief Exponential cartesian impedance motion.
 *
 * This motion is a implements a exponential cartesian impedance controller on the client side and does not use
 * Franka's internal impedance controller. Instead, it uses Franka's internal torque controller and calculates the
 * torques itself.
 */
class ExponentialImpedanceMotion : public ImpedanceMotion {
 public:
  /**
   * @brief Parameters for the exponential cartesian impedance motion.
   * @see ImpedanceMotion::Params
   *
   * @param exponential_decay The exponential decay factor for the impedance controller.
   */
  struct Params : public ImpedanceMotion::Params {
    double exponential_decay{0.005};
  };

  /**
   * @param target The target pose.
   */
  explicit ExponentialImpedanceMotion(const Affine &target);

  /**
   * @param target The target pose.
   * @param params Parameters for the motion.
   */
  explicit ExponentialImpedanceMotion(const Affine &target, const Params &params);

 protected:
  std::tuple<Affine, bool>
  update(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

 private:
  Params params_;
};

}  // namespace franky
