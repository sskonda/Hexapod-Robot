# Hexapod-Robot

ROS 2 workspace for an 18-DOF hexapod robot with:

- gait generation and servo output
- MPU6050 IMU publishing
- open-loop odometry and TF
- servo calibration tools
- simple scripted motion tests
- 2D lidar SLAM with an optional gap-following exploration loop

This README reflects the code that is currently in the repository today, not an aspirational roadmap.

## Repository Layout

- `ros2/hexapod_locomotion`: locomotion, odometry, IMU, servo driver, calibration, predefined path planner, and crab-style path following
- `ros2/hexapod_slam`: `slam_toolbox` launch/config plus a lidar gap-following explorer
- `Sample_Code/`: reference or legacy vendor code that is separate from the ROS 2 launch flows above

## What The Code Does Right Now

- `hexapod_core.launch.py` starts the hardware-facing locomotion stack: `imu_publisher`, `locomotion`, and `servo_driver`.
- `path_plan.launch.py` includes the core stack and adds a `path_plan` node that publishes a simple linear or square `cmd_vel` test path.
- `slam.launch.py` starts `slam_toolbox` and, by default, also starts:
  - `gap_following_explorer`, which looks for open lidar gaps and publishes a short rolling `nav_msgs/Path`
  - `crab_path_follower`, which converts that path into `cmd_vel`
- The locomotion node publishes `odom` and the `odom -> base_link` transform from commanded gait motion. This is open-loop odometry, not fused localization.
- The locomotion node can apply roll and pitch stabilization from the IMU using the PID gains in `ros2/hexapod_locomotion/config/locomotion.yaml`.
- The exploration behavior is crab-motion based. The follower commands `linear.x` and `linear.y`, but keeps `angular.z = 0`, so the robot does not rotate while following the rolling path.
- The explorer stops when an obstacle gets too close, publishes a stop/decision point, waits briefly, and chooses a new heading. If it cannot find a traversable gap after repeated replans, it halts.

## Hardware And Software Assumptions

The current code assumes the following when run on the real robot:

- ROS 2 Python workspace built with `colcon`
- Linux or Raspberry Pi style access to I2C and GPIO
- MPU6050 IMU at I2C address `0x68`
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
- `imu/data_raw`: IMU output from `imu_publisher`
- `pid/roll/state`: PID roll tuning state as `geometry_msgs/Vector3Stamped` where `x=setpoint_deg`, `y=measurement_deg`, `z=error_deg`
- `pid/roll/error_terms`: PID roll contributions as `geometry_msgs/Vector3Stamped` where `x=P`, `y=I`, `z=D`
- `pid/roll/output`: PID roll correction output as `std_msgs/Float64`
- `pid/pitch/state`: PID pitch tuning state as `geometry_msgs/Vector3Stamped` where `x=setpoint_deg`, `y=measurement_deg`, `z=error_deg`
- `pid/pitch/error_terms`: PID pitch contributions as `geometry_msgs/Vector3Stamped` where `x=P`, `y=I`, `z=D`
- `pid/pitch/output`: PID pitch correction output as `std_msgs/Float64`
- `odom`: open-loop odometry from `locomotion`
- `scan`: lidar input for `slam_toolbox` and `gap_following_explorer`
- `path`: rolling path from `gap_following_explorer` to `crab_path_follower`
- `decision_point`: point published when the explorer stops to choose a new direction
- `map`, `odom`, `base_link`: frames expected by the SLAM stack

## PID Tuning

The locomotion node now exposes PID telemetry for body roll and pitch tuning. The gains are loaded from `ros2/hexapod_locomotion/config/locomotion.yaml` using:

- `roll_pid_kp`, `roll_pid_ki`, `roll_pid_kd`, `roll_pid_i_saturation`
- `pitch_pid_kp`, `pitch_pid_ki`, `pitch_pid_kd`, `pitch_pid_i_saturation`
- `publish_pid_debug`

If all three gains for an axis are left at `0.0`, that axis falls back to the older `balance_gain` behavior instead of PID output.

Typical tuning tools:

```bash
ros2 topic echo /pid/roll/state
ros2 topic echo /pid/roll/error_terms
ros2 topic echo /pid/pitch/state
```

For live plots in `rqt_plot`, useful fields are:

- `/pid/roll/state/vector/z`
- `/pid/roll/error_terms/vector/x`
- `/pid/roll/error_terms/vector/y`
- `/pid/roll/error_terms/vector/z`
- `/pid/roll/output/data`
- `/pid/pitch/state/vector/z`

## Current Limitations

- Odometry is generated from commanded gait motion, so SLAM quality will drift without better state estimation.
- `slam.launch.py` is tightly coupled to crab-style exploration and does not command body yaw while following a path.
- `hexapod_core.launch.py` assumes the IMU node can talk to real hardware unless you modify the stack for a non-hardware environment.
- Hardware-specific nodes depend on Linux I2C/GPIO access and will not behave like a full robot bring-up on a desktop machine without those devices.
