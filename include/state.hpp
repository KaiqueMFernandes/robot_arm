#pragma once

#include <cmath>
#include <array>
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace robot_arm {
    /*
    structure defines and handles variables 
    state 
    */
    struct JointState {
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
        double angle = 0.0;
        double ang_vel = 0.0; 
    };
    
    struct Arm {
        std::array<JointState, 3> joints;
    };
}
