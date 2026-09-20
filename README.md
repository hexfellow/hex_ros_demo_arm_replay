# hex_ros_demo_arm_replay — Arm Trajectory Recording and Replay Demo

[中文](README_cn.md) | **English**

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Topics](#topics)
- [Parameters](#parameters)
- [Trajectory File Format](#trajectory-file-format)
- [Project Structure](#project-structure)

## Overview

`hex_ros_demo_arm_replay` records and replays joint trajectories for HEXFELLOW robot arms. The recorder connects to a real arm and saves drag-teaching data, while the replay node loads a JSON trajectory and drives a simulation or real robot.

It provides:

- key-triggered and continuous real-arm recording modes;
- Archer Y6 and Firefly Y6 real-arm replay;
- MuJoCo simulation replay;
- first-waypoint alignment, keyboard start, and return-home exit.

This package supports **ROS 2 Humble** and is compatible with **ROS 1 Noetic**.

## Quick Start

> Complete [Installation](#installation). Replay an existing trajectory directly; otherwise [record a trajectory](#2-record-a-trajectory) first, then set `waypoints_path`.

### 1. Run Replay

- Set the absolute trajectory file path in the applicable `replay_param.yaml`:

  ```yaml
  waypoints_path: "/absolute/path/to/trajectory.json"
  ```

- The trajectory file must follow the [Trajectory File Format](#trajectory-file-format).
- See [Project Structure](#project-structure) for the ROS 1 and ROS 2 configuration file locations.

> **Warning:** The robot model, degrees of freedom, and joint order in the trajectory file must match the current robot arm.

#### ROS 2

**Complete real-robot launch:**

```shell
ros2 launch hex_ros_demo_arm_replay real_replay.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_type:=firefly robot_grip_type:=empty enable_keyboard:=true
```

**Complete simulation launch:**

```shell
ros2 launch hex_ros_demo_arm_replay sim_replay.launch.py viewer:=true rviz:=true
```

> Generate trajectory files by following [Record a Trajectory](#2-record-a-trajectory).

#### ROS 1

**Complete real-robot launch:**

```shell
roslaunch hex_ros_demo_arm_replay real_replay.launch \
    robot_host:=192.168.1.100 robot_port:=8439 robot_type:=firefly robot_grip_type:=empty enable_keyboard:=true
```

**Complete simulation launch:**

```shell
roslaunch hex_ros_demo_arm_replay sim_replay.launch viewer:=true rviz:=true
```

#### Launch Arguments

| Argument | Description |
|----------|-------------|
| `robot_host` / `robot_port` | Controller IP address and port |
| `robot_type` | `archer` or `firefly` |
| `robot_grip_type` | `gp100`, `gp80`, `gr100`, or `empty` |
| `enable_keyboard` | Start the keyboard node; used by `real_replay` only |
| `viewer` / `rviz` | Start the MuJoCo viewer / RViz |

#### Keyboard Controls

- **`s`** — Start trajectory replay
- **`q`** — Stop replay and return home

---

### 2. Record a Trajectory

#### Real-Robot Drag-Teaching Recording

Create the output directory first and pass a complete JSON file path to `--output`:

```shell
mkdir -p <your_ws>/trajectories
```

> Drag teaching connects to and controls real hardware. Clear the operating area, verify the emergency stop, and ensure `--robot-type` and `--grip-type` match the device before starting.

**Required arguments:**

| Argument | Type | Description |
|----------|------|-------------|
| `--ip` | str | Robot controller IP address |
| `--port` | int | Robot controller port |
| `--output` | str | Trajectory JSON output file path |

**Optional arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--mode` | `record` | `record` (keyboard-triggered recording) or `stream` (continuous streaming) |
| `--ctrl-rate` | 1000 | Recorder main-loop rate [Hz] |
| `--samp-rate` | 100 | Sampling rate [Hz] |
| `--robot-type` | `archer_y6` | Robot model: `archer_y6`, `firefly_y6`, or `firefly_y6_H2_40` |
| `--grip-type` | `empty` | Gripper model: `empty`, `gp80`, `gr100`, or `gp100` |

> Note: launch files use `robot_type:=archer` / `firefly`, while the recorder uses `--robot-type archer_y6` / `firefly_y6`; these values are intentionally different.

Complete Archer example:

```shell
python3 <your_ws>/src/hex_ros_demo_arm_replay/script/arm_record.py \
    --ip <robot_ip> --port <robot_port> \
    --output <your_ws>/trajectories/trajectory.json \
    --robot-type archer_y6 --grip-type empty --mode record
```

For other robot and gripper models, change `--robot-type` and `--grip-type` in the command above; see the argument table for supported values.

Recording mode details:

- **`record` mode (default):** Press `r` to record the current joint position and `c` to clear all recorded points. Record at least one point before finishing.
- **`stream` mode:** Continuously records every frame; no keypress is needed.
- **Finish and save:** Press `Ctrl+C` when recording is complete and wait for `[Recorder] Done: ...` or `[TrajStream] Done: ...`. The script closes the JSON and updates its point count during shutdown; do not simply close the terminal. The output file is selected with `--output`.

## Installation

### Prerequisites

- **ROS 2 Humble** is installed; use **ROS 1 Noetic** for ROS 1 compatibility.
- Python 3, `pip3`, Git, and the build tools for the selected ROS version are installed.
- Before real-robot recording or replay, ensure the device is reachable and prepare the actual IP address, port, arm model, and gripper model.

### 1. Install Python Dependencies

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-runtime>=0.1.0' \
    'hex-driver-robot>=0.1.0'
```

### 2. Create and Enter the Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. Clone ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
```

### 4. Build

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Published | `manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | Robot arm control message |
| Subscribed | `manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | Robot arm state message |
| Subscribed | `teleop_keyboard_state` | `hex_ros_msgs/msg/HexRosTeleopKeyboardStateStamped` | Keyboard state message |

> [Message type reference](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate_ros` | 1000.0 | Main control loop rate [Hz] |
| `rate_traj` | 500.0 | Trajectory publish rate (downsampled from rate_ros) [Hz] |
| `rate_teleop` | 100.0 | Keyboard monitor rate [Hz] |
| `model_urdf` | "" | URDF model file path |
| `model_frame_id` | `base_link` | Robot base frame |
| `pose_end_in_flange` | `[0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]` | End-effector pose in the flange frame `[x, y, z, qw, qx, qy, qz]` |
| `lim_vel` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | Joint velocity limits [rad/s] |
| `lim_acc` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | Joint acceleration limits [rad/s²] |
| `jnt_eff` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | Joint torque compensation [Nm] |
| `end_position` | `[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]` | End (return home) target position [rad] |
| `expected_time` | 2.0 | Expected duration for first-point/home motion [s] |
| `waypoints_path` | "" | Trajectory JSON file path (node exits if empty) |
> Defaults in this table come from `config/<ros_version>/replay_param.yaml`. Values declared in node code are fallbacks used only when YAML is not loaded.

---

## Trajectory File Format

The JSON file produced by the recording scripts follows the format below, parsed by `PointLoader` (the replay node):

```json
{
  "info": {"start_time_ns": 0, "end_time_ns": 1000000000, "total_points": 2,
           "dof": 6, "robot_type": "archer_y6", "gripper_type": "empty"},
  "point": {
    "1": {"ts_ns": 0, "arm": [0, -1.5, 3, 0, 0, 0], "grip": []},
    "2": {"ts_ns": 1000000000, "arm": [0.1, -1.5, 3, 0, 0, 0], "grip": []}
  }
}
```

| Path | Type | Description |
|------|------|-------------|
| `info.start_time_ns` | int | Start timestamp (ns) |
| `info.end_time_ns` | int | End timestamp (ns) |
| `info.total_points` | int | Total number of trajectory points |
| `info.dof` | int | Joint degrees of freedom |
| `info.robot_type` | string | Robot model selected during recording |
| `info.gripper_type` | string | Gripper model selected during recording |
| `point.<N>.ts_ns` | int | Relative timestamp [ns], 0-based at the first point |
| `point.<N>.arm` | float[6] | 6 joint positions [rad]; the recorder writes its current precision and the loader accepts ordinary JSON numbers |
| `point.<N>.grip` | float[] | Gripper joint position; empty when no gripper is used |

Notes:
- The replay node converts `ts_ns` to relative seconds as the time base for linear interpolation.

hex_ros_demo_arm_replay/
├── config/
│   ├── ros1/
│   │   └── replay_param.yaml                # ROS 1 replay parameters
│   └── ros2/
│       └── replay_param.yaml                # ROS 2 replay parameters
├── hex_ros_demo_arm_replay/
│   ├── replay_util/
│   │   ├── __init__.py                      # replay_util package initializer
│   │   ├── interface_base.py                # ROS interface base class
│   │   ├── ros1_interface.py                # ROS 1 interface
│   │   └── ros2_interface.py                # ROS 2 interface
│   ├── __init__.py                          # Python package initializer
│   ├── arm_replay.py                        # Replay node
│   ├── PointLoader.py                       # Trajectory file loader class
│   └── TrajectoryController.py              # Trajectory control classes
├── launch/
│   ├── ros1/
│   │   ├── arm_replay.launch                # ROS 1 replay-node launch file
│   │   ├── real_replay.launch               # ROS 1 real-robot replay launch file
│   │   └── sim_replay.launch                # ROS 1 simulation replay launch file
│   └── ros2/
│       ├── arm_replay.launch.py             # ROS 2 replay-node launch file
│       ├── real_replay.launch.py            # ROS 2 real-robot replay launch file
│       └── sim_replay.launch.py             # ROS 2 simulation replay launch file
├── resource/
│   └── hex_ros_demo_arm_replay
├── script/
│   ├── arm_record.py
│   ├── traj_recorder.py
│   └── traj_stream.py
├── .gitignore
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── README_cn.md
├── README.md
├── setup.cfg
└── setup.py
