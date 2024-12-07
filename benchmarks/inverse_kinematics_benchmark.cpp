#include <chrono>
#include <random>
#include <iostream>
#include <vector>

#include "franky/robot.hpp"

using namespace franky;

void run_ik_benchmark(Robot* robot, const Affine& target, int num_trials) {
    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, 0.06);

    Vector7d base_config = robot->currentJointPositions();
    std::vector<Vector7d> initial_configs;
    initial_configs.reserve(num_trials);

    // Pre-generate all random configurations
    for (int i = 0; i < num_trials; i++) {
        Vector7d q0 = base_config;
        for (int j = 0; j < 7; j++) {
            q0[j] += d(gen);
        }
        initial_configs.push_back(q0);
    }

    // Run the benchmark
    for (int i = 0; i < num_trials; i++) {
        Vector7d result = robot->inverseKinematics(target, initial_configs[i]);

        // Verify solution
        Affine result_pose = robot->forwardKinematics(result);
        Vector6d error;
        error.head<3>() = result_pose.translation() - target.translation();
        error.tail<3>() = result_pose.rotation().eulerAngles(2,1,0) - target.rotation().eulerAngles(2,1,0);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <fci_ip>" << std::endl;
        return 1;
    }

    Robot::Params params;
    params.realtime_config = franka::RealtimeConfig::kIgnore;

    Robot robot(argv[1], params);
    Affine target = robot.forwardKinematics(robot.currentJointPositions());

    const int NUM_TRIALS = 300;
    run_ik_benchmark(&robot, target, NUM_TRIALS);
    return 0;
}