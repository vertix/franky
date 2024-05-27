#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/joint_state.hpp"
#include "franky/motion/waypoint_motion.hpp"

namespace franky {

class JointWaypointMotion : public WaypointMotion<franka::JointPositions, JointState> {
 public:
  explicit JointWaypointMotion(const std::vector<Waypoint<JointState>> &waypoints);

  explicit JointWaypointMotion(const std::vector<Waypoint<JointState>> &waypoints, Params params);

 protected:

  void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointPositions> &previous_command,
      ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointPositions> &previous_command,
      const Waypoint<JointState> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::JointPositions getControlSignal(
      const ruckig::InputParameter<7> &input_parameter) const override;
};

}  // namespace franky
