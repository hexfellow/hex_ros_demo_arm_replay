# hex_ros_arm_replay
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 参数说明](#4-参数说明)
- [5. 依赖关系](#5-依赖关系)
- [6. 快速使用](#6-快速使用)

---

## 1. 包的简介

这是 **Archer Y6** 机械臂的**轨迹演示包**，包含轨迹录制与回放两个子系统。

本包提供两个主要功能：

- **轨迹录制** — `script/` 下的录制脚本连接到真机控制器，通过拖动示教方式记录机械臂关节轨迹，输出为 JSON 格式的路径点文件。支持两种录制模式：键盘触发式（`arm_record.py --mode record`，按 `r` 记录）和连续流式（`arm_record.py --mode stream`，逐帧记录）。
- **轨迹回放** — `arm_replay` 节点从 JSON 路径点文件加载轨迹，通过线性插值驱动机械臂沿记录路径运动。支持键盘控制（`s` 开始回放，`q` 停止并归位）。

同时支持 **ROS 1** 和 **ROS 2**。

---

## 2. 包架构

```
hex_ros_arm_replay/
├── config/                           # 参数配置
│   ├── ros1/
│   │   └── replay_param.yaml         #   ROS 1 参数
│   └── ros2/
│       └── replay_param.yaml         #   ROS 2 参数
├── launch/                           # ROS launch 启动文件
├── hex_ros_arm_replay/               # 核心代码
│   ├── arm_replay.py                 #   主节点：轨迹回放控制循环
│   ├── PointLoader.py                #   轨迹 JSON 文件加载器
│   ├── TrajectoryController.py       #   轨迹规划器（线性插值）
│   └── replay_util/                  #   双层 ROS 接口抽象层
│       ├── interface_base.py         #     抽象基类（InterfaceBase）
│       ├── ros1_interface.py         #     ROS 1 DataInterface
│       └── ros2_interface.py         #     ROS 2 DataInterface
├── script/                           # 录制脚本
│   ├── arm_record.py                 #   主录制入口（record / stream 模式）
│   ├── traj_recorder.py              #   键盘触发式录制器
│   └── traj_stream.py                #   连续流式录制器
├── jsons/                            # 轨迹 JSON 示例文件
│   ├── trajectory.json
│   ├── trajectory1.json
│   └── trajectory5.json
├── resource/                         # ament 资源索引
├── setup.py                          # Python 打包配置（ROS 2）
├── CMakeLists.txt                    # CMake 打包配置（ROS 1）
├── package.xml                       # ROS 包清单（双系统条件依赖）
├── README.md                         # 英文文档
└── README_CN.md                      # 中文文档
```

---

## 3. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | 机械臂 + 夹爪控制指令（JNT 位置模式） |
| 订阅 | `manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 机械臂 + 夹爪状态反馈 |
| 订阅 | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | 键盘按键状态 |

> [消息类型描述](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_ros` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_traj` | 500.0 | 轨迹发布频率（从 rate_ros 降采样）[Hz] |
| `rate_teleop` | 100.0 | 键盘监听频率 [Hz] |
| `model_urdf` | "" | URDF 模型文件路径 |
| `model_frame_id` | `base_link` | 机器人基坐标系 |
| `lim_vel` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | 关节速度限制 [rad/s] |
| `lim_acc` | `[10.0, 10.0, 10.0, 10.0, 10.0, 10.0]` | 关节加速度限制 [rad/s²] |
| `jnt_eff` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | 关节力矩补偿 [Nm] |
| `end_position` | `[0.0, -1.5, 3.0, 0.0, 0.0, 0.0]` | 结束（归位）目标位置 [rad] |
| `expected_time` | 5.0 | 往返运动预期时长 [s] |
| `waypoints_path` | "" | 轨迹 JSON 文件路径（为空时退出节点） |
> 参数在`config/`中设置
---

## 5. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/sim_archer_y6.git
git clone https://github.com/hexfellow/teleop_keyboard.git
```

---

## 6. 快速使用

### 1. 构建工作空间

```shell
mkdir -p hex_ws/src
cd hex_ws/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_arm_replay.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/sim_archer_y6.git
git clone https://github.com/hexfellow/teleop_keyboard.git
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd hex_ws
catkin_make
source devel/setup.bash --extend
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd hex_ws
colcon build
source install/setup.bash --extend
```



### 4. 使用包

启动 `arm_replay` 回放节点，加载轨迹 JSON 并驱动机械臂运动：

**ROS 1：**

```shell
rosrun hex_ros_arm_replay arm_replay _waypoints_path:={/path/to}/trajectory.json
```

**ROS 2：**

```shell
ros2 run hex_ros_arm_replay arm_replay --ros-args -p waypoints_path:={/path/to}/trajectory.json
```
> 请确保你的param参数设置无误

键盘控制：

- **`s`** — 开始轨迹回放
- **`q`** — 停止回放并归位

> 轨迹 JSON 文件可通过录制脚本生成。

### 5. 使用脚本

#### 轨迹录制（真机）

连接真机控制器后运行：

```shell
python3 script/arm_record.py --mode record --ctrl-rate 1000 --samp-rate 100
```

录制模式说明：

- **`record` 模式（默认）：** 按 `r` 记录当前关节位置，按 `c` 清空所有记录点。输出文件为 `jsons/trajectory.json`。
- **`stream` 模式：** 逐帧连续记录，无需按键触发。
