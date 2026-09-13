#include <algorithm>
#include <array>
#include <cmath>
#include <Eigen/Dense>

#include "utilities.hpp"
#include "state.hpp"

namespace robot_arm {

class Controller {
public:
    Controller(Arm& armState,
               const Eigen::Vector2d& target,
               double proportionalGain = 1.0)
        : arm_State_(armState), target_(target), k_p_(proportionalGain) {}

    /// Calculate one angular-velocity command per joint.
    /// Each command uses the joint's angle feedback and a wrapped P error.
    std::array<double, 3> calculateAng_vel() const {
        std::array<double, 3> angularVelocities{};
        const Eigen::Vector2d endEffector = arm_State_.joints.back().position;
        const Eigen::Vector2d targetOffset = target_ - endEffector;
        const double desiredAngle = std::atan2(targetOffset.y(), targetOffset.x());

        for (std::size_t index = 0; index < arm_State_.joints.size(); ++index) {
            const double angleError =
                wrapAngle(desiredAngle - arm_State_.joints[index].angle);
            angularVelocities[index] = k_p_ * angleError;
        }
        return angularVelocities;
    }

    /// Apply the controller output to all joints in the arm state.
    void update() {
        const std::array<double, 3> angularVelocities = calculateAng_vel();
        for (std::size_t index = 0; index < arm_State_.joints.size(); ++index) {
            arm_State_.joints[index].ang_vel = angularVelocities[index];
        }
    }

private:
    static double wrapAngle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    Arm& arm_State_;
    Eigen::Vector2d target_;
    double k_p_ = 1.0;
};

} // namespace robot_arm
