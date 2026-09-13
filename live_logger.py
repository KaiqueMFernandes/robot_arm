"""Live view of the latest state in output/joint_states.csv."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


CSV_PATH = Path("output/joint_states.csv")


def latest_state():
    """Return the most recently logged CSV row, or None while no data exists."""
    if not CSV_PATH.exists():
        return None

    with CSV_PATH.open(newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    return rows[-1] if rows else None


figure, (arm_axis, angle_axis, velocity_axis) = plt.subplots(1, 3, figsize=(12, 4))


def update(_frame):
    state = latest_state()
    if state is None:
        return

    positions = [(0.0, 0.0)]
    angles = []
    velocities = []
    for joint in range(1, 4):
        positions.append((float(state[f"joint_{joint}_x"]),
                          float(state[f"joint_{joint}_y"])))
        angles.append(float(state[f"joint_{joint}_angle"]))
        velocities.append(float(state[f"joint_{joint}_ang_vel"]))

    x_values, y_values = zip(*positions)
    arm_axis.clear()
    arm_axis.plot(x_values, y_values, "o-", linewidth=3)
    arm_axis.set_title(f"Arm position (t = {float(state['sim_time']):.2f} s)")
    arm_axis.set_xlabel("x")
    arm_axis.set_ylabel("y")
    arm_axis.set_xlim(-1.3, 1.3)
    arm_axis.set_ylim(-1.3, 1.3)
    arm_axis.set_aspect("equal")
    arm_axis.grid()

    joint_labels = ["J1", "J2", "J3"]
    angle_axis.clear()
    angle_axis.bar(joint_labels, angles)
    angle_axis.set_title("Joint angles")
    angle_axis.set_ylabel("radians")
    angle_axis.grid(axis="y")

    velocity_axis.clear()
    velocity_axis.bar(joint_labels, velocities)
    velocity_axis.set_title("Angular velocities")
    velocity_axis.set_ylabel("rad/s")
    velocity_axis.grid(axis="y")


animation = FuncAnimation(figure, update, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()
