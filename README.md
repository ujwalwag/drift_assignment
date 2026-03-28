# Driftbot — Autonomous Tidying Robot

A ROS 2 Jazzy + Gazebo Harmonic project. A four-wheeled differential-drive robot with a 3-joint magnetic arm picks up objects scattered around a simulated home and drops them in a collection bin.

## Packages

### `driftbot_description`

Robot definition. Contains:

- `urdf/` — xacro files: `base.xacro` (chassis, 4 wheels), `torso.xacro`, `arm_active.xacro` (shoulder/elbow/wrist joints), `arm_cosmetic.xacro`, `sensors.xacro` (camera, GPU lidar, IMU), `materials.xacro`, `inertia_macros.xacro`, `ros2_control.xacro`
- `urdf/robot.urdf.xacro` — top-level xacro that includes everything plus Gazebo plugins: diff-drive, joint state publisher, joint position controllers (arm), detachable joint system (magnet pick/drop), triggered publishers (magnet_off fan-out)
- `meshes/` — STL meshes for base, wheels, torso, arm links
- `launch/view_robot.launch.py` — RViz-only robot viewer
- `config/` — RViz configs for different stages
- `rviz/` — saved RViz layout

### `driftbot_gazebo`

Simulation world. Contains:

- `worlds/home.sdf` — two-room home with walls, furniture, three pickable objects (`toy_block_1`, `can`, `ball`), and a collection bin
- `models/` — SDF + DAE meshes for furniture (chairs, dining table, sofa, coffee table, side table, bookshelf, file cabinet, fridge) and the drop-off box
- `launch/home.launch.py` — starts Gazebo (ogre2), `robot_state_publisher`, spawns robot via `ros_gz_sim create`, fires initial `magnet_off` after 5 s

### `driftbot_task`

Mission logic. Contains:

- `driftbot_task/task_node.py` — `TaskNode` class: odom/IMU/scan/camera subscriptions, world-frame pose (odom position + IMU heading), motion primitives (`face_toward`, `drive_to`, `drive_straight`, `turn_by_imu`, `settle_heading`, `bug2_drive_to`), arm staging, `pick()`/`drop()`, mission dispatch
- `driftbot_task/gz_support.py` — `GzBridgeSupport` node: relays `joint_states_gz`→`joint_states`, broadcasts `odom`→`base_footprint` TF, relays `scan_raw`→`scan` with `lidar_link` frame
- `driftbot_task/missions/` — `__init__.py` (dispatcher), `mission_01.py` (straight-line block pick+drop), `mission_02.py` (Bug2 can+ball pick+drop)
- `config/gz_bridge.yaml` — 15 ROS↔Gazebo topic mappings
- `launch/mission.launch.py` — staggered launch of Gazebo, bridge, nodes, RViz
- `rviz/mission.rviz` — RViz layout with grid, robot model, TF, laser scan, camera

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_bridge`, `ros_gz_sim`
- `robot_state_publisher`, `tf2_ros`
- `rviz2`, `image_view`
- Python 3

No Nav2, no OpenCV, no external planners.

## Build

```bash
cd ~/drift_assignment/driftbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Files

### `driftbot_description` — view_robot

Opens RViz with the robot URDF model for inspection. No simulation.

```bash
ros2 launch driftbot_description view_robot.launch.py
```

### `driftbot_gazebo` — home

Starts Gazebo with the two-room home world, runs `robot_state_publisher`, and spawns the robot. Does not start any bridge or task nodes.

```bash
ros2 launch driftbot_gazebo home.launch.py
```

Optional args: `spawn_x`, `spawn_y`, `spawn_yaw`, `spawn_z`, `software_gl`.

### `driftbot_task` — mission

The main launch. Starts everything in sequence:

1. **0 s** — Gazebo world + robot spawn (includes `home.launch.py`)
2. **8 s** — ROS↔Gazebo bridge (clock, odom, cmd_vel, joints, scan, arm, IMU, camera, magnet topics)
3. **12 s** — `gz_support` node (joint relay, odom→TF, scan frame fix) + `task_node` (runs mission)
4. **15 s** — RViz + camera viewer

```bash
ros2 launch driftbot_task mission.launch.py
```

Optional args: `spawn_x`, `spawn_y`, `spawn_yaw`, `mission_id`.

## Missions

A single launch runs all phases in sequence:

**Mission 1 — Pick block (straight-line)**
Drives south to `toy_block_1`, stops at 60 cm, picks up with magnetic arm, turns 180°, locks heading north, drives to drop-off point, turns -90° to face east, locks heading, drops into collection bin.

**Mission 2 — Pick can + ball (Bug2)**
Uses Bug2 obstacle-avoidance to navigate to the can, picks it up, Bug2 back to bin, drops. Then repeats for the ball.

**Mission 3 — Computer vision object detection pipeline**
TBD.

## World Layout

Robot spawns at `(7.5, 9.0)` facing south (`yaw = -π/2`).

| Object | Position |
|--------|----------|
| toy_block_1 | (7.5, 5.0) |
| can | (1.5, 9.0) |
| ball | (1.5, 2.0) |
| collection bin | (8.5, 9.0) |
| drop-off point | (7.5, 9.0) |

## References

- Rover STL meshes and URDF adapted from [GGomezMorales/waver](https://github.com/GGomezMorales/waver/tree/humble)
- Arm URDF and meshes adapted from [AL5D description](https://github.com/Daniella1/urdf_files_dataset/tree/main/urdf_files/robotics-toolbox/al5d_description)
- Furniture models sourced/adapted from [Gazebo Fuel](https://app.gazebosim.org/fuel/models), [osrf/gazebo_models](https://github.com/osrf/gazebo_models), and [ipa320/cob_simulation](https://github.com/ipa320/cob_simulation)

