cmake_minimum_required(VERSION 3.16)
project(robot_arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_executable(robot_arm src/engine.cpp)
target_include_directories(robot_arm PRIVATE include .)
target_link_libraries(robot_arm PRIVATE Eigen3::Eigen Threads::Threads)
