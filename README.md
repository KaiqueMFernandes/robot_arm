![under dev badge](https://camo.githubusercontent.com/0a69e0994839d7231356ced4fbad636efce3f65b61023c5ca7ec2860df72c910/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f5374617475732d496e253230446576656c6f706d656e742d6f72616e6765)

# Robot arm simulation

A small C++17 simulation of a planar, three-link robot arm. It runs a joint controller and a state logger on separate threads, then writes the simulated joint states to a CSV file. A Python script can display the latest logged pose and joint values while the simulation runs.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- Eigen 3
- CMake's Threads package
- Python 3 with `matplotlib` (only for the live plot)

## Build and run

From the repository root:

```sh
cmake -S . -B build
cmake --build build
./build/robot_arm
```

On Windows, run the generated executable from the `build` directory (for example, `build\\Debug\\robot_arm.exe` with a multi-configuration generator).

The executable simulates for five seconds and writes `output/joint_states.csv`, creating the `output/` directory if needed. Run it from the repository root so that the output file is placed there.

## Live plot

Install the plotting dependency if needed:

```sh
python3 -m pip install matplotlib
```

Start the simulation in one terminal, then in another terminal run:

```sh
python3 live_logger.py
```

The plot reads the most recent row from `output/joint_states.csv` approximately every 100 ms. It shows the arm pose, joint angles, and angular velocities. Run the script from the repository root.

## Plots

This section is reserved for additional plots and visualizations as they are added to the project.



## Output CSV

Each run creates or replaces `output/joint_states.csv`. It contains one header row followed by one row per logged sample, with time in seconds and the position, angle, and angular velocity for each of the three joints. Angles are in radians and angular velocities are in radians per second. The columns are:

```text
sim_time,
joint_1_x,joint_1_y,joint_1_angle,joint_1_ang_vel,
joint_2_x,joint_2_y,joint_2_angle,joint_2_ang_vel,
joint_3_x,joint_3_y,joint_3_angle,joint_3_ang_vel
```

## What the program does

- Models three 0.4-unit links in a 2D plane, initialized with joint angles of 0.78, 0.35, and -1.57 radians.
- Runs the controller at a nominal 250 Hz and logs state at a nominal 750 Hz for five seconds.
- Writes time, each link endpoint's x/y coordinates, each joint angle, and each joint angular velocity to the CSV file.
- Uses a proportional controller that points each joint angle toward the direction from the end effector to a fixed target at `(1.1, 0.4)`. This is a basic simulation example; it is not a complete pick-and-place planner or hardware driver.

`utilities.hpp` also contains a two-link forward-kinematics Jacobian and Newton-Raphson inverse-kinematics helper. That helper is not currently called by the simulation executable.

## Repository layout

```text
include/state.hpp   Joint and arm state definitions
src/engine.cpp      Simulation loop, integration, threading, and CSV output
src/controller.cpp  Proportional joint controller
utilities.hpp       Euler integration and two-link inverse kinematics
live_logger.py      Live plot of the generated CSV
CMakeLists.txt      CMake build configuration
```
