# hex_ros_demo_arm_replay — 机械臂轨迹录制与回放演示

**中文** | [English](README.md)

## 目录

- [项目概述](#项目概述)
- [快速使用](#快速使用)
- [安装](#安装)
- [话题接口](#话题接口)
- [参数说明](#参数说明)
- [轨迹文件格式](#轨迹文件格式)
- [项目结构](#项目结构)

## 项目概述

`hex_ros_demo_arm_replay` 用于录制和回放 HEXFELLOW 机械臂关节轨迹。录制脚本连接真机并保存拖动示教数据，回放节点从 JSON 文件加载轨迹并驱动仿真或真机执行。

主要提供：

- 按键采点和连续采样两种真机录制模式；
- Archer Y6 / Firefly Y6 真机回放；
- MuJoCo 仿真回放；
- 轨迹首点对齐、键盘启动和归位退出。

本包支持 **ROS 2 Humble**，兼容 **ROS 1 Noetic**。

## 快速使用

> 请先完成[安装](#安装)。已有轨迹可直接运行回放；没有轨迹时，先完成[录制轨迹](#2-录制轨迹)，再设置 `waypoints_path`。

### 1. 运行回放

- 在对应的 `replay_param.yaml` 中设置轨迹文件的绝对路径：

  ```yaml
  waypoints_path: "/absolute/path/to/trajectory.json"
  ```

- 轨迹文件必须符合[轨迹文件格式](#轨迹文件格式)。
- ROS 1 和 ROS 2 的配置文件位置见[项目结构](#项目结构)。

> **警告：** 轨迹文件中的机器人型号、自由度和关节顺序必须与当前机械臂一致。

#### ROS 2

**真机完整启动：**

```shell
ros2 launch hex_ros_demo_arm_replay real_replay.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439 robot_type:=firefly robot_grip_type:=empty enable_keyboard:=true
```

**仿真完整启动：**

```shell
ros2 launch hex_ros_demo_arm_replay sim_replay.launch.py viewer:=true rviz:=true
```

> 轨迹文件可通过[录制轨迹](#2-录制轨迹)生成。

#### ROS 1

**真机完整启动：**

```shell
roslaunch hex_ros_demo_arm_replay real_replay.launch \
    robot_host:=192.168.1.100 robot_port:=8439 robot_type:=firefly robot_grip_type:=empty enable_keyboard:=true
```

**仿真完整启动：**

```shell
roslaunch hex_ros_demo_arm_replay sim_replay.launch viewer:=true rviz:=true
```

#### 启动参数

| 参数 | 说明 |
|------|------|
| `robot_host` / `robot_port` | 控制器 IP 与端口 |
| `robot_type` | `archer` 或 `firefly` |
| `robot_grip_type` | `gp100`、`gp80`、`gr100` 或 `empty` |
| `enable_keyboard` | 是否启动键盘节点；仅 `real_replay` 使用 |
| `viewer` / `rviz` | 是否启动 MuJoCo viewer / RViz |

#### 键盘控制

- **`s`** — 开始轨迹回放
- **`q`** — 停止回放并归位

---

### 2. 录制轨迹

#### 真机拖动示教录制

先创建输出目录，并为 `--output` 指定完整的 JSON 文件路径：

```shell
mkdir -p <your_ws>/trajectories
```

> 拖动示教会连接并控制真机。操作前请清空运动范围、确认急停有效，并确认所选 `--robot-type`、`--grip-type` 与设备一致。

**必选参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `--ip` | str | 机器人控制器 IP 地址 |
| `--port` | int | 机器人控制器端口 |
| `--output` | str | 轨迹 JSON 输出文件路径 |

**可选参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--mode` | `record` | `record`（键盘触发录制）或 `stream`（连续流式录制） |
| `--ctrl-rate` | 1000 | 录制脚本主循环频率 [Hz] |
| `--samp-rate` | 100 | 采样频率 [Hz] |
| `--robot-type` | `archer_y6` | 机器人型号：`archer_y6`、`firefly_y6` 或 `firefly_y6_H2_40` |
| `--grip-type` | `empty` | 夹爪型号：`empty`、`gp80`、`gr100` 或 `gp100` |

> 注意：launch 参数使用 `robot_type:=archer` / `firefly`，录制脚本则使用 `--robot-type archer_y6` / `firefly_y6`，两者取值不同。

Archer 完整示例：

```shell
python3 <your_ws>/src/hex_ros_demo_arm_replay/script/arm_record.py \
    --ip <robot_ip> --port <robot_port> \
    --output <your_ws>/trajectories/trajectory.json \
    --robot-type archer_y6 --grip-type empty --mode record
```

其他机器人和夹爪型号请修改上述命令中的 `--robot-type` 与 `--grip-type`，可选值见参数表。

录制模式说明：

- **`record` 模式（默认）：** 按 `r` 记录当前关节位置，按 `c` 清空所有记录点。结束前必须至少记录一个点。
- **`stream` 模式：** 逐帧连续记录，无需按键触发。
- **完成并保存：** 录制完成后按 `Ctrl+C`，等待终端输出 `[Recorder] Done: ...` 或 `[TrajStream] Done: ...`。脚本在退出阶段闭合 JSON 并更新轨迹点数；不要直接关闭终端。输出文件由 `--output` 指定。

## 安装

### 前置条件

- 已安装 **ROS 2 Humble**；使用 ROS 1 时安装 **ROS 1 Noetic**。
- 已安装 Python 3、`pip3`、Git，以及所选 ROS 版本的构建工具。
- 真机录制或回放前，需确认设备网络可达，并准备好实际 IP、端口、机械臂型号和夹爪型号。

### 1. 安装 Python 依赖

```shell
pip3 install \
    'hex-util-msg>=0.1.0' \
    'hex-util-runtime>=0.1.0' \
    'hex-driver-robot>=0.1.0'
```

### 2. 创建并进入工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 3. 克隆 ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
```

### 4. 编译包

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `manip_ctrl` | `hex_ros_msgs/msg/HexRosRoboManipCtrlStamped` | 机械臂控制消息 |
| 订阅 | `manip_state` | `hex_ros_msgs/msg/HexRosRoboManipStateStamped` | 机械臂状态消息 |
| 订阅 | `teleop_keyboard_state` | `hex_ros_msgs/msg/HexRosTeleopKeyboardStateStamped` | 键盘状态消息 |

> [消息类型描述](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_ros` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_traj` | 500.0 | 轨迹发布频率（从 rate_ros 降采样）[Hz] |
| `rate_teleop` | 100.0 | 键盘监听频率 [Hz] |
| `model_urdf` | "" | URDF 模型文件路径 |
| `model_frame_id` | `base_link` | 机器人基坐标系 |
| `pose_end_in_flange` | `[0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]` | 末端执行器在法兰坐标系中的位姿 `[x, y, z, qw, qx, qy, qz]` |
| `lim_vel` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | 关节速度限制 [rad/s] |
| `lim_acc` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | 关节加速度限制 [rad/s²] |
| `jnt_eff` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | 关节力矩补偿 [Nm] |
| `end_position` | `[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]` | 结束（归位）目标位置 [rad] |
| `expected_time` | 2.0 | 首点/归位运动预期时长 [s] |
| `waypoints_path` | "" | 轨迹 JSON 文件路径（为空时退出节点） |

> 表中默认值以 `config/<ros_version>/replay_param.yaml` 为准。节点代码中的声明值仅为未加载 YAML 时的兜底值。

---

## 轨迹文件格式

录制脚本输出的 JSON 文件格式如下，该格式由 `PointLoader`（回放节点）加载解析：

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

| 路径 | 类型 | 说明 |
|------|------|------|
| `info.start_time_ns` | int | 起点时间戳 |
| `info.end_time_ns` | int | 末点时间戳  |
| `info.total_points` | int | 轨迹点总数 |
| `info.dof` | int | 关节自由度 |
| `info.robot_type` | string | 录制时选择的机器人型号 |
| `info.gripper_type` | string | 录制时选择的夹爪型号 |
| `point.<N>.ts_ns` | int | 该点相对时间戳 [ns]，以首点为 0 基准 |
| `point.<N>.arm` | float[6] | 6 个关节位置 [rad]；录制脚本按当前精度写出，加载器接受普通 JSON 数值 |
| `point.<N>.grip` | float[] | 夹爪关节位置；未使用夹爪时为空数组 |

说明：
- 回放时节点会将 `ts_ns` 转化为相对秒数作为时间基准进行线性插值。

## 项目结构

```text
hex_ros_demo_arm_replay/
├── config/
│   ├── ros1/
│   │   └── replay_param.yaml                # ROS 1 回放参数
│   └── ros2/
│       └── replay_param.yaml                # ROS 2 回放参数
├── hex_ros_demo_arm_replay/
│   ├── replay_util/
│   │   ├── __init__.py                      # replay_util 包初始化文件
│   │   ├── interface_base.py                # ROS 接口基类
│   │   ├── ros1_interface.py                # ROS 1 接口
│   │   └── ros2_interface.py                # ROS 2 接口
│   ├── __init__.py                          # Python 包初始化文件
│   ├── arm_replay.py                        # 回放节点
│   ├── PointLoader.py                       # 轨迹文件加载类
│   └── TrajectoryController.py              # 轨迹控制类
├── launch/
│   ├── ros1/
│   │   ├── arm_replay.launch                # ROS 1 回放节点启动文件
│   │   ├── real_replay.launch               # ROS 1 真机回放启动文件
│   │   └── sim_replay.launch                # ROS 1 仿真回放启动文件
│   └── ros2/
│       ├── arm_replay.launch.py             # ROS 2 回放节点启动文件
│       ├── real_replay.launch.py            # ROS 2 真机回放启动文件
│       └── sim_replay.launch.py             # ROS 2 仿真回放启动文件
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
```
