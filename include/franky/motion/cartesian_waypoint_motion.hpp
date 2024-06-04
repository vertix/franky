#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/robot_pose.hpp"
#include "franky/robot.hpp"
#include "franky/motion/reference_type.hpp"
#include "franky/util.hpp"
#include "franky/motion/waypoint_motion.hpp"
#include "franky/cartesian_state.hpp"

namespace franky {

/**
 * @brief Cartesian waypoint motion.
 *
 * This motion follows multiple cartesian waypoints in a time-optimal way.
 */
class CartesianWaypointMotion : public WaypointMotion<franka::CartesianPose, CartesianState> {
 public:
  /**
   * @brief Parameters for the Cartesian waypoint motion.
   * @see WaypointMotion::Params
   */
  struct Params : WaypointMotion<franka::CartesianPose, CartesianState>::Params {
    /**
     * The end-effector frame for which the target is defined. This is a transformation from the configured
     * end-effector frame to the end-effector frame the target is defined for.
     */
    Affine frame{Affine::Identity()};
  };

  /**
   * @param waypoints Waypoints to follow.
   */
  explicit CartesianWaypointMotion(const std::vector<Waypoint<CartesianState>> &waypoints);

  /**
   * @param waypoints Waypoints to follow.
   * @param params Parameters for the motion.
   */
  explicit CartesianWaypointMotion(const std::vector<Waypoint<CartesianState>> &waypoints, Params params);

 protected:
  void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<franka::CartesianPose> &previous_command,
      ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<franka::CartesianPose> &previous_command,
      const Waypoint<CartesianState> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::CartesianPose getControlSignal(const ruckig::InputParameter<7> &input_parameter) const override;

 private:
  Params params_;

  CartesianState target_state_;
  Affine ref_frame_;

  static inline Vector7d vec_cart_rot_elbow(double cart, double rot, double elbow) {
    return {cart, cart, cart, rot, rot, rot, elbow};
  }
};

}  // namespace franky
