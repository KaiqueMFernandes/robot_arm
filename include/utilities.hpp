#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <functional>

namespace robot_arm {

template<typename JointState>
class EulerIntegrator{
public:
    /**
     * @brief performs Euler integration
     * @param state Current state vector
     * @param dt Time Step
     * @param L arm length
     * @return Integrated state vector
     */

    static JointState integrate (
        const JointState& state,
        double dt,
        double L
        ) {

        Eigen::Vector2d new_pos;
        Eigen::Vector2d cur_pos = state.position;

        double x = cur_pos(0), y = cur_pos(1);

        const double new_angle = state.angle + state.ang_vel * dt;
        new_pos[0] = x + std::cos(new_angle) * L;
        new_pos[1] = y + std::sin(new_angle) * L;

        JointState result = state;
        result.angle = new_angle;
        result.position = new_pos;
        return result;
        }
};

class InverseKinematics {
public:
    InverseKinematics(const Eigen::Vector2d& targetPos,
                      const Eigen::Vector2d& armLengths,
                      const Eigen::Vector2d& curAngle)
        : targetPos_(targetPos), armLengths_(armLengths), curAngle_(curAngle) {}

    /// Newton-Raphson: find two joint angles that place the end effector at targetPos_.
    Eigen::Vector2d newtonRaphsonSolver(
        int maxIterations = 50, double tolerance = 1e-6) const {
        Eigen::Vector2d angles = curAngle_;
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            const Eigen::Vector2d error = targetPos_ - forwardKinematics(angles);
            if (error.norm() < tolerance) {
                return angles;
            }

            const Eigen::Matrix2d J = jacobian(angles);
            const Eigen::FullPivLU<Eigen::Matrix2d> decomposition(J);
            if (!decomposition.isInvertible()) {
                throw std::runtime_error("Jacobian is singular while solving IK");
            }
            // Newton step: J * deltaAngles = endEffectorError.
            angles += decomposition.solve(error);
        }
        throw std::runtime_error("Inverse kinematics did not converge");
    }

    /// Jacobian evaluated at the supplied two joint angles.
    Eigen::Matrix2d jacobian(const Eigen::Vector2d& angles) const {
        const double theta1 = angles(0);
        const double theta12 = angles(0) + angles(1);
        Eigen::Matrix2d J;
        J << -armLengths_(0) * std::sin(theta1) - armLengths_(1) * std::sin(theta12),
             -armLengths_(1) * std::sin(theta12),
              armLengths_(0) * std::cos(theta1) + armLengths_(1) * std::cos(theta12),
              armLengths_(1) * std::cos(theta12);
        return J;
    }

private:
    Eigen::Vector2d targetPos_;
    Eigen::Vector2d armLengths_;
    Eigen::Vector2d curAngle_;

    Eigen::Vector2d forwardKinematics(const Eigen::Vector2d& angles) const {
        const double theta1 = angles(0);
        const double theta12 = angles(0) + angles(1);
        return {armLengths_(0) * std::cos(theta1) + armLengths_(1) * std::cos(theta12),
                armLengths_(0) * std::sin(theta1) + armLengths_(1) * std::sin(theta12)};
    }

};

}
