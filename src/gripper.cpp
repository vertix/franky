#include "franky/gripper.hpp"

namespace franky {

std::future<bool> Gripper::graspAsync(
    double width, double speed, double force, double epsilon_inner, double epsilon_outer) const {
  return std::async(std::launch::async, &Gripper::grasp, this, width, speed, force, epsilon_inner, epsilon_outer);
}

std::future<bool> Gripper::moveAsync(double width, double speed) const {
  return std::async(std::launch::async, &Gripper::move, this, width, speed);
}

bool Gripper::open(double speed) {
  return move(max_width(), speed);
}

std::future<bool> Gripper::openAsync(double speed) {
  return std::async(std::launch::async, &Gripper::open, this, speed);
}

std::future<bool> Gripper::homingAsync() {
  return std::async(std::launch::async, &Gripper::homing, this);
}

std::future<bool> Gripper::stopAsync() {
  return std::async(std::launch::async, &Gripper::stop, this);
}

} // namespace franky
