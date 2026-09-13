#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include "state.hpp"
#include "utilities.hpp"
#include "controller.cpp"

using namespace robot_arm;

namespace {

constexpr double kControllerRateHz = 250.0;
constexpr double kLoggerRateHz = 750.0;
constexpr double kSimulationDurationSeconds = 5.0;
constexpr std::array<double, 3> kLinkLengths{0.4, 0.4, 0.4};

void logHeader(std::ofstream& logStream) {
    logStream << "sim_time"
              << ",joint_1_x,joint_1_y,joint_1_angle,joint_1_ang_vel"
              << ",joint_2_x,joint_2_y,joint_2_angle,joint_2_ang_vel"
              << ",joint_3_x,joint_3_y,joint_3_angle,joint_3_ang_vel\n";
}

void logState(std::ofstream& logStream, double simulationTime, const Arm& arm) {
    logStream << std::fixed << std::setprecision(6) << simulationTime;
    for (const JointState& joint : arm.joints) {
        logStream << ',' << joint.position.x()
                  << ',' << joint.position.y()
                  << ',' << joint.angle
                  << ',' << joint.ang_vel;
    }
    logStream << '\n';
    logStream.flush();
}

void integrateArm(Arm& arm, double dt) {
    // The base is fixed at the world origin. Each iteration then makes the
    // next link start where the previous link ended.
    Eigen::Vector2d previousJointPosition = Eigen::Vector2d::Zero();
    for (std::size_t index = 0; index < arm.joints.size(); ++index) {
        // EulerIntegrator calculates the endpoint from the supplied pivot.
        arm.joints[index].position = previousJointPosition;
        arm.joints[index] = EulerIntegrator<JointState>::integrate(
            arm.joints[index], dt, kLinkLengths[index]);
        previousJointPosition = arm.joints[index].position;
    }
}

void runController(Arm& arm,
                   std::mutex& armMutex,
                   const Eigen::Vector2d& target,
                   std::atomic<bool>& running) {
    Controller controller(arm, target, 1.0);
    const auto controllerPeriod = std::chrono::milliseconds(
        static_cast<int>(1000.0 / kControllerRateHz));

    while (running) {
        {
            std::lock_guard<std::mutex> lock(armMutex);
            controller.update();
        }
        std::this_thread::sleep_for(controllerPeriod);
    }
}

} // namespace

int main() {
    std::filesystem::create_directories("output");
    std::ofstream dataLog("output/joint_states.csv");
    if (!dataLog) {
        std::cerr << "Could not open output/joint_states.csv for writing.\n";
        return 1;
    }

    Arm arm;
    arm.joints[0].angle = 0.78;
    arm.joints[1].angle = 0.35;
    arm.joints[2].angle = -1.57;

    const Eigen::Vector2d objectTarget{1.1, 0.4};
    std::mutex armMutex;

    const auto loggerPeriod = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / kLoggerRateHz));
    const std::size_t totalLoggerSamples =
        static_cast<std::size_t>(kSimulationDurationSeconds * kLoggerRateHz);
    std::atomic<bool> running{true};

    std::thread controllerThread(runController,
                                 std::ref(arm),
                                 std::ref(armMutex),
                                 std::cref(objectTarget),
                                 std::ref(running));

    logHeader(dataLog);
    for (std::size_t sample = 0; sample < totalLoggerSamples; ++sample) {
        {
            std::lock_guard<std::mutex> lock(armMutex);
            const double simulationTime = static_cast<double>(sample) / kLoggerRateHz;
            integrateArm(arm, 1.0 / kLoggerRateHz);
            logState(dataLog, simulationTime, arm);
        }
        std::this_thread::sleep_for(loggerPeriod);
    }

    running = false;
    controllerThread.join();

    std::cout << "Simulation finished. Log written to output/joint_states.csv\n";
    return 0;
}
