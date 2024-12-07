#include "franky/robot.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/types.hpp"

namespace franky {

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

Eigen::Matrix<double, 6, 1> computeError(const Affine& x_cur, const Affine& target) {
    Eigen::Matrix<double, 6, 1> error;
    // Position error
    error.head<3>() = x_cur.translation() - target.translation();

    // Rotation error
    Eigen::Matrix3d R_cur = x_cur.linear();
    Eigen::Matrix3d R_target = target.linear();
    Eigen::Matrix3d R_error = R_cur.transpose() * R_target;

    Eigen::AngleAxisd rot_vec(R_error);
    error.tail<3>() = -R_cur * (rot_vec.angle() * rot_vec.axis());

    return error;
}

double lineSearch(
    Robot* robot,
    const Vector7d& q,
    const Vector7d& dq,
    const Eigen::Matrix<double, 6, 1>& e_current,
    const Affine& target,
    double alpha = 1.0,
    double beta = 0.5,
    int max_steps = 10) {

    double current_error_norm = e_current.norm();
    double step_size = alpha;

    for (int i = 0; i < max_steps; ++i) {
        Vector7d q_new = q + step_size * dq;
        Affine x_new = robot->forwardKinematics(q_new);
        Eigen::Matrix<double, 6, 1> e_new = computeError(x_new, target);

        if (e_new.norm() < current_error_norm) {
            return step_size;
        }

        step_size *= beta;
    }

    return step_size;
}

} // anonymous namespace

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_hostname) : Robot(fci_hostname, Params()) {}

Robot::Robot(const std::string &fci_hostname, const Params &params)
    : fci_hostname_(fci_hostname), params_(params), franka::Robot(fci_hostname, params.realtime_config) {
  setCollisionBehavior(params_.default_torque_threshold, params_.default_force_threshold);
}

bool Robot::hasErrors() {
  return bool(state().current_errors);
}

bool Robot::recoverFromErrors() {
  automaticErrorRecovery();
  return !hasErrors();
}

JointState Robot::currentJointState() {
  auto s = state();
  return {Eigen::Map<const Vector7d>(state().q.data()), Eigen::Map<const Vector7d>(state().dq.data())};
}

Vector7d Robot::currentJointPositions() {
  return currentJointState().position();
}

Vector7d Robot::currentJointVelocities() {
  return currentJointState().velocity();
}

franka::Model* Robot::lazyModel() {
  if (!model_) {
    model_ = std::make_unique<franka::Model>(loadModel());
  }
  return model_.get();
}

std::array<double, 16>& Robot::lazy_F_T_EE() {
  if (!F_T_EE_) {
    F_T_EE_ = std::make_unique<std::array<double, 16>>(state().F_T_EE);
  }
  return *F_T_EE_;
}

std::array<double, 16>& Robot::lazy_EE_T_K() {
  if (!EE_T_K_) {
    EE_T_K_ = std::make_unique<std::array<double, 16>>(state().EE_T_K);
  }
  return *EE_T_K_;
}

Affine Robot::forwardKinematics(const Vector7d &q) {
  std::array<double, 7> q_array;
  for (size_t i = 0; i < 7; ++i) { q_array[i] = q[i]; }

  std::array<double, 16> pose = lazyModel()->pose(franka::Frame::kEndEffector, q_array, lazy_F_T_EE(), lazy_EE_T_K());
  return Affine(Eigen::Matrix4d::Map(pose.data()));
}

Jacobian Robot::jacobian(const Vector7d &q) {
  std::array<double, 7> q_array;
  for (size_t i = 0; i < 7; ++i) { q_array[i] = q[i]; }

  std::array<double, 42> jacobian_array = lazyModel()->zeroJacobian(
    franka::Frame::kEndEffector, q_array, lazy_F_T_EE(), lazy_EE_T_K());
  return Eigen::Map<const Jacobian>(jacobian_array.data());
}

Vector7d Robot::inverseKinematics(const Affine& target, const Vector7d& q0) {
    Vector7d q = q0;
    Matrix7d I = Matrix7d::Identity();
    const double k0 = 0.01;
    const int max_iterations = 50;
    const double tol = 1e-4;
    const double min_step = 1e-6;
    const double pinv_reg = 0.1;

    for (int i = 0; i < max_iterations; ++i) {
        Affine x = this->forwardKinematics(q);
        Eigen::Matrix<double, 6, 1> e = computeError(x, target);

        double error_norm = e.norm();
        if (error_norm < tol)
            break;

        Jacobian j = this->jacobian(q);
        Eigen::Matrix<double, 7, 6> j_inv = j.transpose() * (j * j.transpose() + pinv_reg * Matrix6d::Identity()).inverse();

        Matrix7d N = I - j_inv * j;
        Vector7d dq_primary = -j_inv * e;
        Vector7d dq_null = N * (-k0 * std::exp(error_norm) * q);
        Vector7d dq = dq_primary + dq_null;

        double step_size = lineSearch(this, q, dq, e, target, 1.0, 0.8, 20);
        if (step_size < min_step) {
            break;
        }

        q += step_size * dq;
    }

    return q;
}

franka::RobotState Robot::state() {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  {
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    if (!is_in_control_unsafe()) {
      current_state_ = readOnce();
    }
  }
  return current_state_;
}

void Robot::setCollisionBehavior(const ScalarOrArray<7> &torque_threshold, const ScalarOrArray<6> &force_threshold) {
  setCollisionBehavior(torque_threshold, torque_threshold, force_threshold, force_threshold);
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold,
    const ScalarOrArray<7> &upper_torque_threshold,
    const ScalarOrArray<6> &lower_force_threshold,
    const ScalarOrArray<6> &upper_force_threshold) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold),
      expand<7>(upper_torque_threshold),
      expand<6>(lower_force_threshold),
      expand<6>(upper_force_threshold));
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold_acceleration,
    const ScalarOrArray<7> &upper_torque_threshold_acceleration,
    const ScalarOrArray<7> &lower_torque_threshold_nominal,
    const ScalarOrArray<7> &upper_torque_threshold_nominal,
    const ScalarOrArray<6> &lower_force_threshold_acceleration,
    const ScalarOrArray<6> &upper_force_threshold_acceleration,
    const ScalarOrArray<6> &lower_force_threshold_nominal,
    const ScalarOrArray<6> &upper_force_threshold_nominal) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold_acceleration),
      expand<7>(upper_torque_threshold_acceleration),
      expand<7>(lower_torque_threshold_nominal),
      expand<7>(upper_torque_threshold_nominal),
      expand<6>(lower_force_threshold_acceleration),
      expand<6>(upper_force_threshold_acceleration),
      expand<6>(lower_force_threshold_nominal),
      expand<6>(upper_force_threshold_nominal));

}

bool Robot::is_in_control_unsafe() const {
  return motion_generator_running_;
}

bool Robot::is_in_control() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return is_in_control_unsafe();
}

std::string Robot::fci_hostname() const {
  return fci_hostname_;
}

std::optional<ControlSignalType> Robot::current_control_signal_type() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  if (!is_in_control_unsafe())
    return std::nullopt;
  if (std::holds_alternative<MotionGenerator<franka::Torques>>(motion_generator_))
    return ControlSignalType::Torques;
  else if (std::holds_alternative<MotionGenerator<franka::JointVelocities>>(motion_generator_))
    return ControlSignalType::JointVelocities;
  else if (std::holds_alternative<MotionGenerator<franka::JointPositions>>(motion_generator_))
    return ControlSignalType::JointPositions;
  else if (std::holds_alternative<MotionGenerator<franka::CartesianVelocities>>(motion_generator_))
    return ControlSignalType::CartesianVelocities;
  else
    return ControlSignalType::CartesianPose;
}

RelativeDynamicsFactor Robot::relative_dynamics_factor() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return params_.relative_dynamics_factor;
}

void Robot::setRelativeDynamicsFactor(const RelativeDynamicsFactor &relative_dynamics_factor) {
  std::unique_lock<std::mutex> lock(control_mutex_);
  params_.relative_dynamics_factor = relative_dynamics_factor;
}

}  // namespace franky
