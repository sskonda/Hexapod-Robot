# Hexapod-Robot

ROS 2 workspace for an 18-DOF hexapod robot with:

- gait generation and servo output
- BNO055 IMU publishing over UART
- open-loop odometry and TF with IMU-assisted yaw hold
- servo calibration tools
- simple scripted motion tests
- 2D lidar SLAM with an optional gap-following exploration loop

This README reflects the current state of the `bno055` branch, not an aspirational roadmap.

## Repository Layout

- `ros2/hexapod_locomotion`: locomotion, odometry, IMU, servo driver, calibration, predefined path planner, and crab-style path following
- `ros2/hexapod_slam`: `slam_toolbox` launch/config plus a lidar gap-following explorer
- `Sample_Code/`: reference or legacy vendor code that is separate from the ROS 2 launch flows above

## What The Code Does Right Now

- `hexapod_core.launch.py` starts the hardware-facing locomotion stack: `bno055_publisher` (under the node name `imu_publisher`), `locomotion`, and `servo_driver`.
- `path_plan.launch.py` includes the core stack and adds a `path_plan` node that publishes a simple linear or square `cmd_vel` test path.
- `bno055_publisher` reads the BNO055 over UART, publishes `sensor_msgs/Imu` on `/imu/data_raw`, publishes `sensor_msgs/MagneticField` on `/imu/mag`, and uses the BNO055 fused NDOF quaternion for orientation by default.
- `slam.launch.py` starts `slam_toolbox` and, by default, also starts:
  - `gap_following_explorer`, which looks for open lidar gaps and publishes a short rolling `nav_msgs/Path`
  - `crab_path_follower`, which converts that path into `cmd_vel`
- The locomotion node publishes `odom` and the `odom -> base_link` transform from commanded gait motion. Position is still open-loop, but yaw is anchored to IMU orientation when available instead of being purely integrated from commanded turn rate.
- The locomotion node uses IMU roll/pitch for balance compensation when enabled and uses the filtered BNO055 yaw estimate from `/imu/data_raw` with the shared PI yaw-hold controller to hold a constant heading while translating without a yaw command.
- The exploration follower commands `linear.x` and `linear.y` in body coordinates and uses the same bounded PI yaw-hold controller. `slam.launch.py` defaults to `crab_follower_yaw_hold_target_mode:=path_heading` so the active SLAM path vector becomes the yaw target; use `initial` to preserve pure crab-style body yaw.
- The explorer stops when an obstacle gets too close, publishes a stop/decision point, waits briefly, and chooses a new heading. If it cannot find a traversable gap after repeated replans, it halts.
- `slam.launch.py` can optionally start `robot_localization` to fuse raw odometry and IMU data, but that path is still optional and off by default.

## Hardware And Software Assumptions

The current code assumes the following when run on the real robot:

- ROS 2 Python workspace built with `colcon`
- Linux or Raspberry Pi style access to UART, I2C, and GPIO
- BNO055 IMU connected on UART `/dev/ttyAMA5`
- `dtoverlay=uart5` enabled in the Raspberry Pi `config.txt`
- `python3-serial` available on the robot for the BNO055 UART driver
- two PCA9685 boards at I2C addresses `0x40` and `0x41`
- optional servo power control on GPIO `4`
- a 2D lidar publishing `sensor_msgs/LaserScan` on `scan` or a remapped equivalent
- `slam_toolbox` installed separately in the ROS environment

## Build

From the repository root:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you are working from PowerShell instead of Bash, source the ROS 2 environment using the matching PowerShell setup scripts for your installation.

## Main Launch Flows

### 1. Core Locomotion Stack

Use this to bring up IMU publishing, locomotion, and the servo driver:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py servo_dry_run:=true
```

Notes:

- `servo_dry_run:=true` is the safest first test and keeps the robot from moving.
- The launch file defaults to `servo_dry_run:=false`, so do not omit that flag unless you want hardware output.
- `apply_offsets:=true` by default, so saved calibration offsets are applied unless you disable them.
- The default IMU bring-up uses `imu_uart_port:=/dev/ttyAMA5`, `imu_mode:=NDOF_MODE`, `imu_publish_rate_hz:=50.0`, `imu_use_external_crystal:=false`, and `yaw_correction_gain:=0.6`.
- The default BNO055 UART recovery settings are `imu_read_retry_count:=3` and `imu_retry_backoff_sec:=0.01`; the local yaw filter settings are now only fallback knobs for non-fusion BNO055 modes.
- The core stack now publishes both `/imu/data_raw` and `/imu/mag`.

### 2. Servo Calibration

There is a calibration launch file:

```bash
ros2 launch hexapod_locomotion calibration.launch.py
```

Current behavior:

- this launches `servo_driver` in dry-run mode
- this launches the interactive `calibration` node
- the `calibration` node itself still attempts hardware access unless you change its `dry_run` parameter, but it falls back to dry-run if hardware init fails
- the calibration file used by the locomotion stack is `ros2/hexapod_locomotion/config/servo_calibration.yaml`

### 3. Scripted Motion Tests

The path planner is useful for bench testing the gait controller with simple commanded motion:

```bash
ros2 launch hexapod_locomotion path_plan.launch.py servo_dry_run:=true path_type:=square
```

Supported path types in the current code:

- `linear`: move forward, then backward
- `square`: forward, strafe right, backward, strafe left

This is still open-loop motion driven by `cmd_vel`.

### 4. SLAM Only

If you only want `slam_toolbox` and do not want the exploration nodes:

```bash
ros2 launch hexapod_slam slam.launch.py enable_explorer:=false
```

This expects a live `scan` topic and a valid `odom` / `base_link` TF tree.

If you want to enable the optional EKF in `slam.launch.py`, make sure the locomotion odometry topic matches the EKF input topic. One working combination is:

Terminal 1:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py odom_topic:=odom/raw
```

Terminal 2:

```bash
ros2 launch hexapod_slam slam.launch.py enable_robot_localization:=true raw_odom_topic:=odom/raw odom_topic:=odom
```

### 5. SLAM With Autonomous Exploration

This is the most important caveat in the current repo:

`slam.launch.py` does not launch the locomotion stack, lidar driver, or servo driver by itself.

To run the full stack on the robot as the repository currently works, you need the core locomotion system running separately from the SLAM launch. A typical setup is:

Terminal 1:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py
```

Terminal 2:

```bash
ros2 launch hexapod_slam slam.launch.py
```

Expected behavior when all prerequisites are available:

- `slam_toolbox` builds a 2D map from the lidar scan
- `gap_following_explorer` picks an open direction from the lidar
- `crab_path_follower` turns that rolling path into `cmd_vel`
- `locomotion` consumes `cmd_vel`, walks the robot, and publishes open-loop `odom`

If `scan`, `odom`, or the `odom -> base_link` transform are missing or stale, the explorer waits and the robot does not move.

## Important Topics And Frames

Current interfaces used by the main ROS 2 nodes:

- `cmd_vel`: motion commands consumed by `locomotion`
- `body_pose`: optional body roll/pitch/yaw offsets for `locomotion`
- `body_shift`: optional body x/y/z shifts for `locomotion`
- `servo_targets`: joint targets produced by `locomotion` and consumed by `servo_driver`
- `imu/data_raw`: IMU output from the BNO055 publisher, including accel, gyro, and the BNO055 fused orientation quaternion
- `imu/mag`: raw magnetometer output from the BNO055 publisher
- `odom`: locomotion odometry by default, or filtered odometry if you enable `robot_localization` and wire topics accordingly
- `scan`: lidar input for `slam_toolbox` and `gap_following_explorer`
- `path`: rolling path from `gap_following_explorer` to `crab_path_follower`
- `decision_point`: point published when the explorer stops to choose a new direction
- `map`, `odom`, `base_link`: frames expected by the SLAM stack

## Current Limitations

- X/Y odometry is still generated from commanded gait motion, so map quality can still drift even though yaw hold is better.
- Trusted yaw depends on healthy BNO055 yaw calibration (`gyro=3`, `mag>=2`, `sys>=2`, `accel>=2`) and on the BNO055 axes matching the robot's expected frame conventions.
- `slam.launch.py` is still tightly coupled to the gap-following exploration/follower path. Use `crab_follower_yaw_hold_target_mode:=initial` if you want the robot to crab without rotating toward the path direction.
- `slam.launch.py` still does not launch the locomotion stack, lidar driver, or servo driver by itself.
- Hardware-specific nodes depend on Linux UART/I2C/GPIO access and will not behave like a full robot bring-up on a desktop machine without those devices.
efn
