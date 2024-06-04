#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/robot_pose.hpp"
#include "franky/motion/motion.hpp"
#include "franky/motion/reference_type.hpp"

namespace franky {

/**
 * @brief Base class for client-side cartesian impedance motions.
 *
 * This motion is a implements a cartesian impedance controller on the client side and does not use
 * Franka's internal impedance controller. Instead, it uses Franka's internal torque controller and calculates the
 * torques itself.
 */
class ImpedanceMotion : public Motion<franka::Torques> {
 public:
  /**
   * @brief Parameters for the impedance motion.
   *
   * @param target_type The type of the target reference (relative or absolute).
   * @param translational_stiffness The translational stiffness in [10, 3000] N/m.
   * @param rotational_stiffness The rotational stiffness in [1, 300] Nm/rad.
   * @param force_constraints The force constraints in [N, Nm] for each joint.
   * @param force_constraints_active Allows to enable or disable individual force constraints.
   */
  struct Params {
    ReferenceType target_type{ReferenceType::Absolute};
    double translational_stiffness{2000};
    double rotational_stiffness{200};
    Eigen::Vector<double, 6> force_constraints;
    Eigen::Vector<bool, 6> force_constraints_active{Eigen::Vector<bool, 6>::Zero()};
  };

  /**
   * @param target The target pose.
   * @param params Parameters for the motion.
   */
  explicit ImpedanceMotion(Affine target, const Params &params);

 protected:
  void initImpl(const franka::RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;

  franka::Torques
  nextCommandImpl(
      const franka::RobotState &robot_state,
      franka::Duration time_step,
      double rel_time,
      double abs_time,
      const std::optional<franka::Torques> &previous_command) override;

  [[nodiscard]] inline Affine intermediate_target() const {
    return intermediate_target_;
  }

  [[nodiscard]] inline Affine target() const {
    return absolute_target_;
  }

  virtual std::tuple<Affine, bool>
  update(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

 private:
  Affine absolute_target_;
  Affine target_;
  Params params_;

  Eigen::Matrix<double, 6, 6> stiffness, damping;
  Affine intermediate_target_;

  std::unique_ptr<franka::Model> model_;
};

}  // namespace franky
