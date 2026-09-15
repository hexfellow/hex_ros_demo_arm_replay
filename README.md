# hex_ros_demo_arm_replay
[**中文**](README_cn.md) | **English**

## Table of Contents

- [1. Introduction](#1-introduction)
- [2. Package Architecture](#2-package-architecture)
- [3. Topics](#3-topics)
- [4. Parameters](#4-parameters)
- [5. Dependencies](#5-dependencies)
- [6. Quick Start](#6-quick-start)

---

## 1. Introduction

This is the **trajectory demo package** for **HEXFELLOW** robot arms, consisting of trajectory recording and replay.

Two main features:

- **Trajectory Recording** — Recording scripts under `script/` connect to the real robot controller and record joint trajectories via drag teaching, outputting waypoints as JSON. Two recording modes: keyboard-triggered (`arm_record.py --mode record`, press `r` to record) and continuous streaming (`arm_record.py --mode stream`, frame-by-frame recording).
- **Trajectory Replay** — The `arm_replay` node loads a trajectory from a JSON waypoints file and drives the arm along the recorded path via linear interpolation. On startup, the arm first moves to the first waypoint and waits for a keyboard command. Press **`s`** to execute one full replay cycle; upon completion the arm holds at the final position. Press **`q`** at any time to return to the home position (configured via the `end_position` parameter).

Supports both **ROS 1** and **ROS 2**.

---

## 2. Package Architecture

```
hex_ros_demo_arm_replay/
├── config/                           # Parameter configuration
│   ├── ros1/
│   │   └── replay_param.yaml         #   ROS 1 parameters
│   └── ros2/
│       └── replay_param.yaml         #   ROS 2 parameters
├── launch/                           # ROS launch files
├── hex_ros_demo_arm_replay/               # Core code
│   ├── arm_replay.py                 #   Main node: trajectory replay control loop
│   ├── PointLoader.py                #   JSON trajectory file loader
│   ├── TrajectoryController.py       #   Trajectory planner (linear interpolation)
│   └── replay_util/                  #   Dual-ROS interface abstraction layer
│       ├── interface_base.py         #     Abstract base class (InterfaceBase)
│       ├── ros1_interface.py         #     ROS 1 DataInterface
│       └── ros2_interface.py         #     ROS 2 DataInterface
├── script/                           # Recording scripts
│   ├── arm_record.py                 #   Main recording entry point (record / stream mode)
│   ├── traj_recorder.py              #   Keyboard-triggered recorder
│   └── traj_stream.py                #   Continuous streaming recorder
├── jsons/                            # Example trajectory JSON files
│   ├── trajectory.json
│   ├── trajectory1.json
│   └── trajectory5.json
├── resource/                         # ament resource index
├── setup.py                          # Python packaging (ROS 2)
├── CMakeLists.txt                    # CMake packaging (ROS 1)
├── package.xml                       # ROS package manifest (dual-system conditional deps)
├── README.md                         # English documentation
└── README_cn.md                      # Chinese documentation
```

---

## 3. Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Published | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Arm + gripper control command (JNT position mode) |
| Subscribed | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Arm + gripper state feedback |
| Subscribed | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | Keyboard key states |

> [Message type reference](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate_ros` | 1000.0 | Main control loop rate [Hz] |
| `rate_traj` | 500.0 | Trajectory publish rate (downsampled from rate_ros) [Hz] |
| `rate_teleop` | 100.0 | Keyboard monitor rate [Hz] |
| `model_urdf` | "" | URDF model file path |
| `model_frame_id` | `base_link` | Robot base frame |
| `lim_vel` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | Joint velocity limits [rad/s] |
| `lim_acc` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | Joint acceleration limits [rad/s²] |
| `jnt_eff` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | Joint torque compensation [Nm] |
| `end_position` | `[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]` | End (return home) target position [rad] |
| `expected_time` | 5.0 | Expected travel duration [s] |
| `waypoints_path` | "" | Trajectory JSON file path (node exits if empty) |
> Parameters are set in `config/`

---

## 5. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/sim_archer_y6.git
git clone https://github.com/hexfellow/teleop_keyboard.git
```

---

## 6. Quick Start

### 1. Create Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/sim_archer_y6.git
git clone https://github.com/hexfellow/teleop_keyboard.git
```

### 3. Build

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. Using the Package

arm_replay provides launch files for several startup scenarios. Trajectory parameters are configured in `config/<ros_version>/replay_param.yaml`.

#### ROS 1

**Replay node only:**

```shell
roslaunch hex_ros_demo_arm_replay arm_replay.launch
```

**Complete real-robot launch:**

First edit the arguments in `launch/ros1/real_replay.launch`:

```xml
<arg name="robot_host" default="192.168.1.100"/>
<arg name="robot_port" default="8439"/>
<arg name="robot_grip_type" default="empty"/>
<arg name="robot_type" default="firefly"/>
<arg name="enable_keyboard" default="true"/>
```

Then launch:

```shell
roslaunch hex_ros_demo_arm_replay real_replay.launch
```

**Complete simulation launch:**

```shell
roslaunch hex_ros_demo_arm_replay sim_replay.launch viewer:=true rviz:=true
```

#### ROS 2

**Replay node only:**

```shell
ros2 launch hex_ros_demo_arm_replay arm_replay.launch.py
```

**Complete real-robot launch:**

First edit the declarations in `launch/ros2/real_replay.launch.py`:

```python
robot_host_arg = DeclareLaunchArgument(
    name='robot_host',
    default_value='192.168.1.100')
robot_port_arg = DeclareLaunchArgument(
    name='robot_port',
    default_value='8439')
robot_grip_type_arg = DeclareLaunchArgument(
    name='robot_grip_type',
    default_value='empty')
robot_type_arg = DeclareLaunchArgument(
    name='robot_type',
    default_value='firefly')
enable_keyboard_arg = DeclareLaunchArgument(
    name='enable_keyboard',
    default_value='true')
```

Then launch:

```shell
ros2 launch hex_ros_demo_arm_replay real_replay.launch.py
```

**Complete simulation launch:**

```shell
ros2 launch hex_ros_demo_arm_replay sim_replay.launch.py viewer:=true rviz:=true
```

> Edit the real-robot connection, robot type, gripper type, and `enable_keyboard` values in the corresponding launch file before starting.
> Trajectory JSON files can be generated by the recording scripts.

#### Keyboard Controls

- **`s`** — Start trajectory replay
- **`q`** — Stop replay and return home

---

### 5. Using the Recording Script

#### Trajectory Recording (Real Robot)

Connect to the real robot controller and run:

```shell
python3 script/arm_record.py --ip <robot_ip> --port <robot_port> --output <output_path> \
    --mode record --ctrl-rate 1000 --samp-rate 100
```

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
| `--ctrl-rate` | 1000 | Control loop rate [Hz] |
| `--samp-rate` | 100 | Sampling rate [Hz] |

Recording mode details:

- **`record` mode (default):** Press `r` to record the current joint position, `c` to clear all recorded points. Output file is specified by `--output`.
- **`stream` mode:** Continuously records every frame, no keypress needed. Output file is specified by `--output`.

### 6. Trajectory JSON File Format

The JSON file produced by the recording scripts follows the format below, parsed by `PointLoader` (the replay node):

```json
{
  "info": {
    "start_time_ns": 0,
    "end_time_ns": 29797817037,
    "total_points": 2973,
    "dof": 6,
    "robot_type": "archer_y6",
    "gripper_type": "empty"
  },
  "point": {
    "1": {"ts_ns": 0, "jnt": [-0.037, -1.573, 3.118, 0.028, -0.052, 0.129], "grip": []},
    "2": {"ts_ns": 87882924, "jnt": [-0.037, -1.573, 3.118, 0.028, -0.052, 0.129], "grip": []},
    ...
  }
}
```

| Path | Type | Description |
|------|------|-------------|
| `info.start_time_ns` | int | Start timestamp (ns) |
| `info.end_time_ns` | int | End timestamp (ns) |
| `info.total_points` | int | Total number of trajectory points |
| `info.dof` | int | Joint degrees of freedom |
| `point.<N>.ts_ns` | int | Relative timestamp [ns], 0-based at the first point |
| `point.<N>.jnt` | float[6] | 6 joint positions [rad] (2–3 decimal places) |

Notes:
- The replay node converts `ts_ns` to relative seconds as the time base for linear interpolation.
