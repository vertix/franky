#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/motion/impedance_motion.hpp"

namespace franky {

/**
 * @brief Cartesian impedance motion.
 *
 * This motion is a implements a cartesian impedance controller on the client side and does not use Franka's internal
 * impedance controller. Instead, it uses Franka's internal torque controller and calculates the torques itself.
 */
class CartesianImpedanceMotion : public ImpedanceMotion {
 public:
  /**
   * @brief Parameters for the Cartesian impedance motion.
   * @see ImpedanceMotion::Params
   *
   * @param return_when_finished Whether to end the motion when the target is reached or keep holding the last target.
   * @param finish_wait_factor How long to wait after the motion has finished. This factor gets multiplied with the
   * duration of the motion to obtain the total motion duration. After the motion duration has expired, the motion
   * will hold the target until the total motion duration is reached. E.g. a factor of 1.2 will hold the target for
   * 20% longer than the motion duration.
   */
  struct Params : public ImpedanceMotion::Params {
    bool return_when_finished{true};
    double finish_wait_factor{1.2};
  };

  /**
   * @param target The target pose.
   * @param duration The duration of the motion in [s].
   */
  explicit CartesianImpedanceMotion(const Affine &target, double duration);

  /**
   * @param target The target pose.
   * @param duration The duration of the motion in [s].
   * @param params Parameters for the motion.
   */
  explicit CartesianImpedanceMotion(const Affine &target, double duration, const Params &params);

 protected:
  void initImpl(const franka::RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;

  std::tuple<Affine, bool>
  update(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

 private:
  Affine initial_pose_;
  double duration_;
  Params params_;
};

}  // namespace franky
