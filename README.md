# Hexapod-Robot

ROS 2 workspace for an 18-DOF hexapod robot with:

- gait generation and servo output
- BNO055 IMU publishing over UART
- BNO055 fused-quaternion heading plus EKF-filtered local odometry
- servo calibration tools
- simple scripted motion tests
- RPLIDAR A1 bringup, 2D lidar SLAM, scan safety, and gap-following exploration

This README reflects the current state of the `bno055` branch, not an aspirational roadmap.

## Repository Layout

- `ros2/hexapod_locomotion`: locomotion, odometry, IMU, servo driver, calibration, predefined path planner, and crab-style path following
- `ros2/hexapod_slam`: `slam_toolbox` launch/config plus a lidar gap-following explorer
- `ros2/hexapod_bringup`: robot-level launch files that combine locomotion, RPLIDAR, EKF odometry, SLAM, safety, and exploration
- `Sample_Code/`: reference or legacy vendor code that is separate from the ROS 2 launch flows above

## What The Code Does Right Now

- `pose_stack.launch.py` is the recommended robot launch. It starts locomotion, the RPLIDAR A1 driver, EKF odometry, `slam_toolbox`, scan safety, and the gap-following explorer.
- `hexapod_core.launch.py` starts the hardware-facing locomotion stack: `bno055_publisher` (under the node name `imu_publisher`), `locomotion`, and `servo_driver`.
- `path_plan.launch.py` includes the core stack and adds a `path_plan` node that publishes a simple linear or square `cmd_vel` test path.
- `bno055_publisher` reads the BNO055 over UART, runs in `NDOF_MODE` by default, publishes the BNO055 fused orientation quaternion on `/imu/data_raw`, publishes `sensor_msgs/MagneticField` on `/imu/mag`, and reports yaw trust diagnostics.
- `robot_localization` fuses raw gait odometry velocities from `odom/raw` with the BNO055 fused IMU yaw/yaw-rate into filtered `odom` and owns the `odom -> base_link` transform in the recommended stack.
- `slam.launch.py` starts `slam_toolbox` and, by default, also starts:
  - `gap_following_explorer`, which looks for open lidar gaps and publishes a short rolling `nav_msgs/Path`
  - `crab_path_follower`, which converts that path into `cmd_vel`
- In the recommended stack, the locomotion node publishes raw gait odometry on `odom/raw` without TF. In standalone core mode, locomotion can still publish `odom` and `odom -> base_link` directly.
- The locomotion node uses IMU roll/pitch for balance compensation when enabled and uses the filtered BNO055 yaw estimate from `/imu/data_raw` to hold a constant heading while translating without a yaw command.
- The exploration behavior is still crab-motion based. The follower commands `linear.x` and `linear.y` to move sideways/forward in body coordinates. By default the follower does not add yaw correction; locomotion heading hold is the single yaw owner.
- The explorer stops when an obstacle gets too close, publishes a stop/decision point, waits briefly, and chooses a new heading. If it cannot find a traversable gap after repeated replans, it halts.
- The explorer and scan safety defaults are tuned for the larger real robot footprint and intervene earlier near side walls than the older setup.

## Hardware And Software Assumptions

The current code assumes the following when run on the real robot:

- ROS 2 Python workspace built with `colcon`
- Linux or Raspberry Pi style access to UART, I2C, and GPIO
- BNO055 IMU connected on UART `/dev/ttyAMA5`
- `dtoverlay=uart5` enabled in the Raspberry Pi `config.txt`
- `python3-serial` available on the robot for the BNO055 UART driver
- two PCA9685 boards at I2C addresses `0x40` and `0x41`
- optional servo power control on GPIO `4`
- RPLIDAR A1 available through the `rplidar_ros` package and `rplidar_a1_launch.py`
- `slam_toolbox` installed separately in the ROS environment
- `robot_localization` installed separately in the ROS environment

## Build

From the repository root:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you are working from PowerShell instead of Bash, source the ROS 2 environment using the matching PowerShell setup scripts for your installation.

## Main Launch Flows

### 1. Recommended Robot Pose + Mapping Stack

Use this on the real robot when you want the intended full stack:

```bash
ros2 launch hexapod_bringup pose_stack.launch.py
```

This launches:

- RPLIDAR A1 using `rplidar_ros` / `rplidar_a1_launch.py`
- BNO055 IMU publishing `/imu/data_raw` and `/imu/mag`
- locomotion and servo output
- EKF local odometry from raw gait velocities plus BNO055 fused yaw
- `slam_toolbox` for `map -> odom`
- scan safety, gap-following exploration, and the crab path follower

Useful variants:

```bash
ros2 launch hexapod_bringup pose_stack.launch.py servo_dry_run:=true
ros2 launch hexapod_bringup pose_stack.launch.py enable_explorer:=false
```

`servo_dry_run:=true` is the safest first test. `enable_explorer:=false` brings up sensing, EKF odom, safety, and SLAM without autonomous motion commands.

### 2. Core Locomotion Stack

Use this to bring up IMU publishing, locomotion, and the servo driver:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py servo_dry_run:=true
```

Notes:

- `servo_dry_run:=true` is the safest first test and keeps the robot from moving.
- The launch file defaults to `servo_dry_run:=false`, so do not omit that flag unless you want hardware output.
- `apply_offsets:=true` by default, so saved calibration offsets are applied unless you disable them.
- The default IMU bring-up uses `imu_uart_port:=/dev/ttyAMA5`, `imu_mode:=NDOF_MODE`, `imu_publish_rate_hz:=50.0`, and `imu_use_external_crystal:=false`.
- The default BNO055 UART recovery settings are `imu_read_retry_count:=3`, `imu_retry_backoff_sec:=0.01`, and `imu_yaw_filter_time_constant_sec:=0.5`.
- The core stack now publishes both `/imu/data_raw` and `/imu/mag`.
- Standalone core mode publishes `odom` and `odom -> base_link` directly by default. In `pose_stack.launch.py`, core is automatically rewired to publish raw gait odometry on `odom/raw` while the EKF owns filtered `odom` and TF.

### 3. Servo Calibration

There is a calibration launch file:

```bash
ros2 launch hexapod_locomotion calibration.launch.py
```

Current behavior:

- this launches `servo_driver` in dry-run mode
- this launches the interactive `calibration` node
- the `calibration` node itself still attempts hardware access unless you change its `dry_run` parameter, but it falls back to dry-run if hardware init fails
- the calibration file used by the locomotion stack is `ros2/hexapod_locomotion/config/servo_calibration.yaml`

### 4. Scripted Motion Tests

The path planner is useful for bench testing the gait controller with simple commanded motion:

```bash
ros2 launch hexapod_locomotion path_plan.launch.py servo_dry_run:=true path_type:=square
```

Supported path types in the current code:

- `linear`: move forward, then backward
- `square`: forward, strafe right, backward, strafe left

This is still open-loop motion driven by `cmd_vel`.

### 5. SLAM Only Or Lower-Level Debugging

If you only want `slam_toolbox` and do not want the exploration nodes:

```bash
ros2 launch hexapod_slam slam.launch.py enable_explorer:=false
```

This expects a live `scan` topic and a valid `odom` / `base_link` TF tree.

If you want to debug the two lower-level pieces separately, use this split:

Terminal 1:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py odom_topic:=odom/raw publish_odom_tf:=false use_imu_for_odom:=false
```

Terminal 2:

```bash
ros2 launch hexapod_slam slam.launch.py enable_robot_localization:=true raw_odom_topic:=odom/raw odom_topic:=odom
```

For normal robot use, prefer `pose_stack.launch.py` so those topic and TF ownership rules are handled for you.

### 6. Mapping Bringup Internals

`mapping_bringup.launch.py` is the lower-level robot-composition launch file used by `pose_stack.launch.py`. It is useful when you want more explicit control over the pieces:

```bash
ros2 launch hexapod_bringup mapping_bringup.launch.py launch_lidar:=true lidar_launch_package:=rplidar_ros lidar_launch_file:=rplidar_a1_launch.py
```

Expected behavior from the recommended full stack:

- `slam_toolbox` builds a 2D map from the lidar scan
- `gap_following_explorer` picks an open direction from the lidar
- `crab_path_follower` turns that rolling path into `cmd_vel`
- `locomotion` consumes `cmd_vel`, walks the robot, and publishes raw gait odometry on `odom/raw`
- `robot_localization` publishes filtered `odom` and the `odom -> base_link` transform

If `scan`, `odom`, or the `odom -> base_link` transform are missing or stale, the explorer waits and the robot does not move.

## Important Topics And Frames

Current interfaces used by the main ROS 2 nodes:

- `cmd_vel`: motion commands consumed by `locomotion`
- `body_pose`: optional body roll/pitch/yaw offsets for `locomotion`
- `body_shift`: optional body x/y/z shifts for `locomotion`
- `servo_targets`: joint targets produced by `locomotion` and consumed by `servo_driver`
- `imu/data_raw`: IMU output from the BNO055 publisher, including accel, gyro, and the BNO055 fused orientation quaternion in the default `NDOF_MODE`
- `imu/mag`: raw magnetometer output from the BNO055 publisher
- `odom/raw`: raw gait odometry from locomotion in the recommended EKF stack
- `odom`: filtered EKF odometry in the recommended stack, or direct locomotion odometry in standalone core mode
- `scan`: lidar input for `slam_toolbox` and `gap_following_explorer`
- `path`: rolling path from `gap_following_explorer` to `crab_path_follower`
- `decision_point`: point published when the explorer stops to choose a new direction
- `map`, `odom`, `base_link`: frames expected by the SLAM stack

## Current Limitations

- X/Y odometry still starts from commanded gait motion. The EKF improves consistency and yaw ownership, but it does not magically measure true x/y translation.
- BNO055 fused yaw still depends on real calibration and on the BNO055 axes matching the robot's expected frame conventions.
- `slam.launch.py` is still tightly coupled to crab-style exploration. It holds heading while translating, but it does not rotate the body to face the path direction.
- `slam_toolbox` corrects map-level drift through LiDAR scan matching, but the immediate wall-avoidance behavior still comes from scan safety and the explorer.
- `slam.launch.py` is intentionally a lower-level launch file; use `pose_stack.launch.py` for full robot bringup.
- Hardware-specific nodes depend on Linux UART/I2C/GPIO access and will not behave like a full robot bring-up on a desktop machine without those devices.
