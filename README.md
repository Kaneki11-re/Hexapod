# hex2 — ROS 2 Hexapod

ROS 2 Jazzy simulation stack for the **Hexapod six-legged robot**.

This repository currently provides:

- Robot description and URDF/Xacro
- Gazebo Sim simulation
- `ros2_control` joint control
- Simulated IMU, LiDAR and odometry
- EKF state estimation
- 2D SLAM with `slam_toolbox`
- RViz visualization

> **Current scope:** simulation, robot description, control interfaces and SLAM.
>
> **Not yet included:** IK, gait generation, walking controller, teleop, Nav2 or MoveIt 2.

---

## Architecture

```text
hex_description
      │
      ▼
 Robot URDF/Xacro (RViz)
      │
      ▼
hex_gazebo ────────► Gazebo Sim
      │                 │
      │                 ├── /joint_states
      │                 ├── /imu
      │                 ├── /scan
      │                 └── /odom
      │
      ▼
hex_slam
      │
      ├── robot_localization
      │        │
      │        └── odom → base_footprint
      │
      └── slam_toolbox
               │
               └── map → odom
```

Resulting TF chain:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── legs
                ├── imu_link
                └── lidar_link
```

---

# Packages

## `hex_description`

Contains the robot model:

```text
hex_description/
├── config/
├── launch/
├── meshes/
├── rviz/
├── scripts/
└── urdf/
```

Main components:

- URDF/Xacro robot description
- Six 3-DOF legs
- CAD meshes
- Joint limits and mechanical parameters
- Rest/stand pose
- RViz configuration
- Stand-height utility

The robot has:

```text
6 legs × 3 joints = 18 joints
```

Each leg:

```text
coxa → femur → tibia
```

---

## `hex_gazebo`

Adds simulation functionality to the robot model.

Provides:

- Gazebo Sim world
- `gz_ros2_control`
- Joint controllers
- IMU
- LiDAR
- Simulated odometry
- ROS-Gazebo bridges

Main files:

```text
hex_gazebo/
├── config/
├── launch/
├── urdf/
└── worlds/
```

The Gazebo model reuses the robot description from `hex_description` rather than maintaining a second robot model.

---

## `hex_slam`

Provides localization and SLAM.

Pipeline:

```text
/odom + /imu
      │
      ▼
robot_localization
      │
      ▼
odom → base_footprint
      │
      ▼
slam_toolbox
      │
      ├── /map
      └── map → odom
```

`hex_slam` expects Gazebo to already be running.

---

# Requirements

Recommended:

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Sim
```

Important ROS packages:

```text
xacro
robot_state_publisher
joint_state_publisher_gui
rviz2
ros_gz_sim
ros_gz_bridge
gz_ros2_control
ros2_control
ros2_controllers
robot_localization
slam_toolbox
```

---

# Installation

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Place the packages directly inside `src`:

```text
~/ros2_ws/src/
├── hex_description
├── hex_gazebo
└── hex_slam
```

Build:

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep '^hex_'
```

Expected:

```text
hex_description
hex_gazebo
hex_slam
```

---

# Running the Robot

## 1. Robot Description / RViz

Start the robot without Gazebo:

```bash
ros2 launch hex_description display.launch.py
```

Useful for checking:

- Robot geometry
- Joint axes
- Joint limits
- TF
- Mesh placement

---

## 2. Gazebo Simulation

```bash
ros2 launch hex_gazebo gazebo.launch.py
```

This starts:

```text
Gazebo
   ↓
Hex2 robot
   ↓
gz_ros2_control
   ↓
Controllers
```

The robot uses an 18-joint position-control interface.

Check controllers:

```bash
ros2 control list_controllers
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

---

## 3. SLAM

Start Gazebo first:

```bash
ros2 launch hex_gazebo gazebo.launch.py
```

Then in another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hex_slam slam.launch.py rviz:=true
```

SLAM requires:

```text
/scan
/imu
/odom
TF
```

---

# Important ROS Interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Joint states |
| `/imu` | `sensor_msgs/Imu` | Simulated IMU |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR |
| `/odom` | `nav_msgs/Odometry` | Simulated odometry |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM map |

Useful checks:

```bash
ros2 topic list
```

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu
ros2 topic hz /scan
ros2 topic hz /odom
```

---

# Controllers

The main controller is:

```text
hex_trajectory_controller
```

It uses:

```text
joint_trajectory_controller/JointTrajectoryController
```

There is also a:

```text
hex_position_controller
```

for direct position commands.

Do **not** activate both controllers simultaneously because they command the same joint interfaces.

Check:

```bash
ros2 control list_controllers
```

and:

```bash
ros2 control list_hardware_interfaces
```

---

# TF

Expected structure:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                └── lidar_link
```

Check TF:

```bash
ros2 run tf2_tools view_frames
```

Check individual transforms:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

---

# Troubleshooting

## Package not found

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

If required:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Robot does not move

Check:

```bash
ros2 control list_controllers
```

Then:

```bash
ros2 topic echo /joint_states
```

If joint states are not changing, investigate `gz_ros2_control` and the controller before debugging higher-level motion.

---

## Robot falls over in Gazebo

Check the stand pose:

```text
hex_description/config/rest_pose.yaml
```

The stand configuration contains:

```yaml
stand:
  femur: ...
  tibia: ...
  base_height: ...
```

The repository also provides:

```bash
python3 hex_description/scripts/stand_height.py
```

to help calculate a suitable base height from the leg geometry.

---

## `/scan` is missing

Check Gazebo:

```bash
gz topic -l | grep scan
```

Then ROS:

```bash
ros2 topic list | grep scan
```

Expected:

```text
/scan
```

If the Gazebo topic exists but ROS does not see it, check the ROS-Gazebo bridge.

---

## `/imu` is missing

Check:

```bash
gz topic -l | grep imu
ros2 topic list | grep imu
```

Expected:

```text
/imu
```

Then check:

```bash
ros2 topic hz /imu
```

---

## SLAM does not generate a map

Check:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu
```

Then verify:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

and:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

The required chain is:

```text
map → odom → base_footprint → base_link → lidar_link
```

Fix missing sensor topics or TF before changing SLAM parameters.

---

## Simulation time problems

Check:

```bash
ros2 topic echo /clock
```

If `/clock` is not updating, check the Gazebo/ROS bridge.

---

# Development Roadmap

The current stack is the foundation for locomotion development.

Recommended next layer:

```text
Forward Kinematics
        ↓
Inverse Kinematics
        ↓
Foot Trajectory
        ↓
Gait Generator
        ↓
Walking Controller
        ↓
Teleoperation / Nav2
```

Eventually:

```text
Teleop / Nav2
      ↓
Motion Command
      ↓
Gait Generator
      ↓
Foot Trajectory
      ↓
Inverse Kinematics
      ↓
18 Joint Commands
      ↓
ros2_control
      ↓
Gazebo / Hardware
```

---

# Current Status

```text
[✓] Robot description
[✓] 18-joint hexapod model
[✓] RViz visualization
[✓] Gazebo simulation
[✓] ros2_control
[✓] Joint controllers
[✓] Simulated IMU
[✓] Simulated LiDAR
[✓] Simulated odometry
[✓] robot_localization
[✓] slam_toolbox

[ ] Forward kinematics
[ ] Inverse kinematics
[ ] Foot trajectory generation
[ ] Gait generation
[ ] Walking controller
[ ] Teleoperation
[ ] Nav2
[ ] MoveIt 2
[ ] Real hardware interface
```

---

# Quick Start

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hex_gazebo gazebo.launch.py
```

```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hex_slam slam.launch.py rviz:=true
```

Useful diagnostics:

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```
