#pragma once

#include <cmath>
#include <iostream>
#include <future>

#include <franka/exception.h>
#include <franka/gripper.h>

namespace franky {

/**
 * @brief Exception thrown by the gripper class.
 */
struct GripperException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

class Gripper : public franka::Gripper {
 public:
  // Maximum speed of the gripper [m/s]
  static constexpr double max_speed{0.1};

  /**
   * @param fci_hostname The hostname of the Franka robot.
   * @param speed The speed of the gripper. Default is 0.04.
   * @param force The force of the gripper. Default is 20.0.
   */
  explicit Gripper(const std::string &fci_hostname, double speed = 0.04, double force = 20.0);

  // Force of the gripper [N]
  double gripper_force{20.0};

  // Speed of the gripper [m/s]
  double gripper_speed{0.04};

  // Flag to indicate if the gripper has an error
  bool has_error{false};

  // Maximum width of the gripper [m]
  const double max_width{0.081 + width_calibration};

  /*
   * @brief Get the current width of the gripper.
   */
  [[nodiscard]] double width() const;

  /*
   * @brief Check if the gripper is grasping.
   */
  [[nodiscard]] bool isGrasping() const;

  /*
   * @brief Move the gripper to the given width.
   *
   * @param width The width to move to [m]
   * @return True if the gripper moved successfully.
   */
  bool move(double width);

  /*
   * @brief Move the gripper to the given width without checking the current width.
   *
   * @param width The width to move to [m]
   * @return True if the gripper moved successfully.
   */
  bool moveUnsafe(double width);

  /*
   * @brief Move the gripper to the given width asynchronously.
   *
   * @param width The width to move to [m]
   * @return A future that will be set to true if the gripper moved successfully.
   */
  std::future<bool> moveAsync(double width);

  bool open();
  bool clamp();
  bool clamp(double min_clamping_width);

  bool release();
  bool release(double width); // [m]
  bool releaseRelative(double width); // [m]

  ::franka::GripperState get_state();

 private:
  // Difference from gripper jaws [m
  const double width_calibration{0.004};
  // Minimum width of the gripper [m]
  const double min_width{0.002};

  /**
  * Save clamp width and compare it in the is_grasping method. If it's smaller,
  * the gripper moves and the object is missing. [m]
  */
  double last_clamp_width{};
};

} // namepace franky
