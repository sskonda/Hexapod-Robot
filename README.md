# Hexapod-Robot

ROS 2 workspace for an 18-DOF hexapod robot with:

- gait generation and servo output
- BNO055 IMU publishing over UART
- servo calibration tools
- simple scripted motion tests
- RPLIDAR A1 bringup and lidar-based 2D mapping with `slam_toolbox`

This README reflects the current source tree. Generated `build/`, `install/`,
and `log/` folders may still contain stale artifacts until you clean and rebuild.

## Repository Layout

- `ros2/hexapod_locomotion`: locomotion, odometry, IMU, servo driver, calibration, predefined path planner, and crab-style path following
- `ros2/hexapod_bringup`: robot-level launch files that combine locomotion, RPLIDAR, static sensor TF, and `slam_toolbox`
- `Sample_Code/`: reference or legacy vendor code that is separate from the ROS 2 launch flows above

The old custom SLAM package is no longer a source dependency. SLAM Toolbox is launched directly from `hexapod_bringup`.

## What The Code Does Right Now

- `pose_stack.launch.py` is the recommended robot launch. It starts locomotion, the RPLIDAR A1 driver, the `base_link -> laser` static TF, and `slam_toolbox`.
- `mapping_bringup.launch.py` is the lower-level composition launch used by `pose_stack.launch.py`.
- `hexapod_core.launch.py` starts the hardware-facing locomotion stack: `bno055_publisher` under the node name `imu_publisher`, `locomotion`, and `servo_driver`.
- `bno055_publisher` reads the BNO055 over UART, runs in `NDOF_MODE` by default, publishes `/imu/data_raw`, publishes `/imu/mag`, and reports yaw trust diagnostics through covariance and diagnostic fields.
- The mapping bringup defaults to gait odometry for `odom -> base_link` and leaves `locomotion_use_imu_for_odom:=false`, because the BNO055 is not always trusted.
- If the BNO055 is well calibrated and stable in your environment, you can opt into IMU yaw inside locomotion odometry with `locomotion_use_imu_for_odom:=true`.
- Future QR-code camera localization should be added as a separate perception/fiducial layer; it should not replace the lidar scan input to `slam_toolbox`.

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
- RPLIDAR A1 USB adapter exposed as `/dev/ttyUSB0` unless overridden at launch
- `slam_toolbox` installed in the ROS environment

## Build

From the repository root:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you are working from PowerShell instead of Bash, source the ROS 2 environment using the matching PowerShell setup scripts for your installation.

After removing the old SLAM package, a clean rebuild is recommended:

```bash
rm -rf build install log ros2/build ros2/install ros2/log
colcon build --symlink-install
source install/setup.bash
```

## Main Launch Flows

### 1. Recommended Robot Pose + Mapping Stack

Use this on the real robot when you want lidar mapping:

```bash
ros2 launch hexapod_bringup pose_stack.launch.py
```

This launches:

- RPLIDAR A1 using `rplidar_ros` / `rplidar_a1_launch.py`
- BNO055 IMU publishing `/imu/data_raw` and `/imu/mag`
- locomotion and servo output
- locomotion odometry and the `odom -> base_link` transform
- static `base_link -> laser` transform
- `slam_toolbox` for `map -> odom` and `/map`

Useful variants:

```bash
ros2 launch hexapod_bringup pose_stack.launch.py servo_dry_run:=true
ros2 launch hexapod_bringup pose_stack.launch.py launch_lidar:=false
ros2 launch hexapod_bringup pose_stack.launch.py enable_slam_toolbox:=false
ros2 launch hexapod_bringup pose_stack.launch.py locomotion_use_imu_for_odom:=true
ros2 launch hexapod_bringup pose_stack.launch.py lidar_serial_port:=/dev/serial/by-id/<your-rplidar-id>
```

`servo_dry_run:=true` is the safest first test. Use `launch_lidar:=false` only when another node is already publishing `/scan`.

RPLIDAR A1 defaults passed into `rplidar_ros`:

- `lidar_launch_package:=rplidar_ros`
- `lidar_launch_file:=rplidar_a1_launch.py`
- `lidar_channel_type:=serial`
- `lidar_serial_port:=/dev/ttyUSB0`
- `lidar_serial_baudrate:=115200`
- `laser_frame:=laser`
- `laser_yaw:=-1.5708` for the current sideways LiDAR mount
- `lidar_inverted:=false`
- `lidar_angle_compensate:=true`
- `lidar_scan_mode:=Sensitivity`

### 2. Basic LiDAR Open-Space Exploration

Use this to run the full pose + mapping stack plus a simple reactive explorer:

```bash
ros2 launch hexapod_bringup exploration_stack.launch.py
```

The launch defaults to `servo_dry_run:=true`, so the first run should publish
commands without moving the robot. When `/scan`, `/map`, and `/cmd_vel` look
reasonable, run real motion explicitly:

```bash
ros2 launch hexapod_bringup exploration_stack.launch.py servo_dry_run:=false
```

The explorer subscribes to `/scan`, scores candidate LiDAR directions by
range and angular clearance, and publishes slow `cmd_vel` commands toward the
most open direction. It is a basic reactive wanderer, not full Nav2-style
frontier exploration.

Useful tuning knobs:

```bash
ros2 launch hexapod_bringup exploration_stack.launch.py \
  explorer_max_speed_mps:=0.025 \
  explorer_obstacle_stop_distance_m:=0.35 \
  explorer_desired_clearance_m:=0.65
```

### 3. Core Locomotion Stack

Use this to bring up IMU publishing, locomotion, and the servo driver:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py servo_dry_run:=true
```

Notes:

- `servo_dry_run:=true` keeps the robot from moving.
- The launch file defaults to `servo_dry_run:=false`, so do not omit that flag unless you want hardware output.
- `apply_offsets:=true` by default, so saved calibration offsets are applied unless you disable them.
- The default IMU bringup uses `imu_uart_port:=/dev/ttyAMA5`, `imu_mode:=NDOF_MODE`, `imu_publish_rate_hz:=50.0`, and `imu_use_external_crystal:=false`.
- The default BNO055 UART recovery settings are `imu_read_retry_count:=3`, `imu_retry_backoff_sec:=0.01`, and `imu_yaw_filter_time_constant_sec:=0.5`.
- Standalone core mode publishes `odom` and `odom -> base_link` directly by default.

### 4. Servo Calibration

There is a calibration launch file:

```bash
ros2 launch hexapod_locomotion calibration.launch.py
```

Current behavior:

- this launches `servo_driver` in dry-run mode
- this launches the interactive `calibration` node
- the `calibration` node itself still attempts hardware access unless you change its `dry_run` parameter, but it falls back to dry-run if hardware init fails
- the calibration file used by the locomotion stack is `ros2/hexapod_locomotion/config/servo_calibration.yaml`

### 5. Scripted Motion Tests

The path planner is useful for bench testing the gait controller with simple commanded motion:

```bash
ros2 launch hexapod_locomotion path_plan.launch.py servo_dry_run:=true path_type:=square
```

Supported path types in the current code:

- `linear`: move forward, then backward
- `square`: forward, strafe right, backward, strafe left

This is still open-loop motion driven by `cmd_vel`.

### 6. SLAM-Only Or Lower-Level Debugging

For normal robot use, prefer `pose_stack.launch.py`. If you want to debug the pieces separately, use this split.

Terminal 1:

```bash
ros2 launch hexapod_locomotion hexapod_core.launch.py servo_dry_run:=true
```

Terminal 2, if `/scan` is already live:

```bash
ros2 launch hexapod_bringup mapping_bringup.launch.py use_locomotion:=false launch_lidar:=false
```

The second launch still publishes the static `base_link -> laser` transform and starts `slam_toolbox`. It expects a valid `odom -> base_link` transform from another source.

## Lidar SLAM Mapping Plan

Implement mapping around the standard 2D SLAM Toolbox data path:

- Publish a clean `sensor_msgs/LaserScan` on `/scan`.
- Ensure the scan `frame_id` matches the launch argument `laser_frame`, default `laser`.
- Measure the lidar pose on the robot and pass it as `laser_x`, `laser_y`, `laser_z`, `laser_roll`, `laser_pitch`, and `laser_yaw`. The current default `laser_yaw` is `-1.5708` radians because the LiDAR is mounted 90 degrees from `base_link`.
- Keep exactly one local odometry owner for `odom -> base_link`. The current default owner is `locomotion`.
- Let `slam_toolbox` own `map -> odom` and publish `/map`.
- Drive slowly while mapping. The current gait odometry is approximate, so map quality will come mostly from lidar scan matching and loop closures.
- Save finished maps with the SLAM Toolbox RViz plugin, SLAM Toolbox services, or `nav2_map_server`'s map saver if it is installed.

The active SLAM parameters live in:

```text
ros2/hexapod_bringup/config/slam_toolbox.yaml
```

Start with the default async mapping config. Tune only after verifying that `/scan`, `odom -> base_link`, and `base_link -> laser` are stable in RViz.

## BNO055 And Future QR Camera

Treat the BNO055 as helpful but conditional:

- Keep publishing `/imu/data_raw`, `/imu/mag`, and diagnostics for visibility.
- Leave `locomotion_use_imu_for_odom:=false` while yaw trust is intermittent.
- Turn `locomotion_use_imu_for_odom:=true` only after the BNO055 is calibrated, frame-aligned, and not showing yaw covariance spikes.
- If you later add `robot_localization`, use covariance-aware IMU fusion and avoid having both locomotion and the EKF publish `odom -> base_link`.

The camera/QR layer should come later as a separate localization aid:

- detect QR/fiducial tags in the camera frame
- publish camera TF and tag observations with timestamps
- use known tag poses to correct localization or validate the map
- keep QR detection outside `slam_toolbox`; SLAM Toolbox should continue to consume lidar scans and odometry/TF

## Important Topics And Frames

Current interfaces used by the main ROS 2 nodes:

- `cmd_vel`: motion commands consumed by `locomotion`
- `body_pose`: optional body roll/pitch/yaw offsets for `locomotion`
- `body_shift`: optional body x/y/z shifts for `locomotion`
- `servo_targets`: joint targets produced by `locomotion` and consumed by `servo_driver`
- `/imu/data_raw`: IMU output from the BNO055 publisher
- `/imu/mag`: raw magnetometer output from the BNO055 publisher
- `odom`: locomotion odometry in the current mapping stack
- `/scan`: lidar input for `slam_toolbox`
- `/map`: occupancy grid produced by `slam_toolbox`
- `lidar_open_space_explorer`: optional reactive explorer that consumes `/scan` and publishes `cmd_vel`
- `map`, `odom`, `base_link`, `laser`: frames expected by the mapping stack

## Current Limitations

- X/Y odometry still starts from commanded gait motion and is only an approximation.
- BNO055 fused yaw depends on calibration, magnetic environment, and frame conventions.
- The open-space explorer is reactive only: it does not do frontier selection, global planning, recovery behaviors, or full collision checking.
- `slam_toolbox` can correct map-level drift through lidar scan matching, but it still needs a sane local `odom -> base_link` transform.
- Hardware-specific nodes depend on Linux UART/I2C/GPIO access and will not behave like a full robot bringup on a desktop machine without those devices.
