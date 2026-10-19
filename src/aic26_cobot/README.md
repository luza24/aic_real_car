# aic26_cobot

Hardware backend for the AIC26 real robot. It is the real-robot counterpart of
`aic26_gazebo`: the drivers publish the same topics and frame names as the
simulation, so `aic26_navigation`, `aic26_perception` and `aic26_decision` run
unchanged.

This package starts the device drivers, `robot_state_publisher` and the
diagnostics monitor. Composition (navigation, perception, mission) belongs to
`aic26_bringup`, just as it does for the Gazebo backend.

## Hardware map

| Interface | Device on the car | Linux device | ROS interface |
| --- | --- | --- | --- |
| Chassis | STM32F429, four motors, four encoders | `/dev/carserial` (115200) | `/cmd_vel`, `/vel_raw`, `/odom`, TF `odom -> base_footprint` |
| IMU | WIT normal protocol | `/dev/imu` (9600) | `/imu/data`, optional `/imu/mag`, frame `imu_link` |
| Lidar | Leishen N10_P | `/dev/lslidar` (460800) | `/scan`, frame `laser` |
| Camera | Orbbec DaBai DCW2 RGB (UVC) | `/dev/video0` | `/camera/image_raw`, `/camera/camera_info`, frame `camera` |

The udev rules that create the stable names are outside this workspace:

```text
/etc/udev/rules.d/85-car-serial.rules   -> /dev/carserial, /dev/imu
/etc/udev/rules.d/lslidar.rules         -> /dev/lslidar
```

The Orbbec depth interface is a vendor-specific USB interface. The AIC26
perception stack only consumes the colour image, so this package implements the
RGB UVC stream with OpenCV and does not open the depth stream.

## Topics and frames

```text
/cmd_vel        geometry_msgs/Twist       input to the chassis
/vel_raw        geometry_msgs/Twist       measured chassis velocity
/odom           nav_msgs/Odometry         integrated odometry
/scan           sensor_msgs/LaserScan     lidar, frame "laser"
/imu/data       sensor_msgs/Imu           IMU, frame "imu_link"
/camera/image_raw        sensor_msgs/Image       frame "camera"
/camera/camera_info      sensor_msgs/CameraInfo  matching intrinsics
/diagnostics    diagnostic_msgs/DiagnosticArray  topic freshness monitor
```

`/imu/mag` is published only when `publish_mag` is enabled and contains raw
magnetometer counts.

`aic26_description/urdf/cobot.xacro` provides the fixed TF tree
`base_footprint -> base_link -> wheels/case/laser/camera_mount/imu_link` plus
`camera_mount -> camera -> camera_optical_frame`. The dimensions are the
measured robot dimensions, so the real and simulated robots share frame names.
The chassis node publishes `odom -> base_footprint`; `robot_state_publisher`
publishes the fixed links.

The chassis starts at the simulated spawn pose `(1.6, 1.7, pi)`
(`initial_x/y/yaw` in `config/chassis.yaml`). With AMCL using the same initial
pose this keeps `map -> odom` near identity, so the real robot, odom frame and
local costmap appear at the same coordinates in RViz as in the simulation.
`reset_odom` resets to this configured pose, not to `(0, 0, 0)`.

The STM32 sends one feedback frame at about 20 Hz, but the USB-serial adapter
delivers frames in bursts. The node therefore integrates each frame with the
fixed `1 / feedback_rate` period instead of the wall-clock arrival interval;
this removes the pose jumps that otherwise appear as RViz jitter. Small
feedback values below `linear_deadband` / `angular_deadband` are treated as
zero at standstill.

`use_imu_yaw:=true` is safe again: the IMU offset is taken relative to the
configured start yaw (so a start yaw of `pi` is preserved instead of being
pulled to zero), and corrections are low-pass filtered and applied only
`imu_correction_rate` times per second with a clamped step.

If a consumer expects `/odom_raw`, remap it in the consumer launch file instead
of duplicating the odometry stream:

```xml
<remap from="/odom" to="/odom_raw"/>
```

## Launch

Use the project entry point for a complete real-robot mission:

```bash
roslaunch aic26_bringup bringup.launch mode:=mission backend:=real
```

This selects the hardware backend and runs the same navigation, perception and
`mission_controller` stack as the simulation mission. The other modes
(`demo`, `capture`, `waypoints`, `mapping`) also accept `backend:=real`.

Start only the hardware backend:

```bash
roslaunch aic26_cobot hardware.launch
```

One device at a time:

```bash
roslaunch aic26_cobot chassis.launch
roslaunch aic26_cobot imu.launch
roslaunch aic26_cobot lidar.launch
roslaunch aic26_cobot camera.launch
roslaunch aic26_cobot status.launch
```

Navigation without the mission, on an already running backend:

```bash
roslaunch aic26_cobot hardware.launch
roslaunch aic26_navigation navigation.launch \
  laser_min_range:=0.2 laser_max_range:=4.0
```

AMCL initializes at `(1.6, 1.7, pi)` by default. Override
`initial_pose_x/y/a` if the robot starts elsewhere, or use
`set_initial_pose:=false` and set the pose once with the RViz
"2D Pose Estimate" tool. The hardware path uses the real lidar range (4 m), not
the 12 m simulation range.

Useful switches:

```bash
# disable individual devices
roslaunch aic26_cobot hardware.launch camera:=false

# enable the IMU yaw fusion (off by default on real hardware)
roslaunch aic26_cobot hardware.launch use_imu_yaw:=true

# enable the keyboard teleop node
roslaunch aic26_cobot hardware.launch teleop:=true

# override the device paths
roslaunch aic26_cobot hardware.launch \
  chassis_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyACM0

# use a different map with the hardware mission
roslaunch aic26_bringup bringup.launch mode:=mission backend:=real \
  map_file:=/path/map.yaml
```

## Integration notes

- The hardware mission sets `light_fallback:=traffic_light`. The traffic light
  detector publishes one generic model name, which `mission_controller` uses
  when no simulation truth is available. Use dedicated detector instances with
  `light_name:=traffic_light_1/2` if the installation can separate the lights.
- The hardware mission initializes AMCL at `(1.6, 1.7, pi)` and uses the real
  lidar range; see `aic26_bringup/launch/mission.launch` and
  `aic26_bringup/launch/backend.launch`. Override `initial_pose_x/y/a` for a
  different start pose, or set `set_initial_pose:=false` and set it in RViz.
- The real camera publishes 1280x720 images, matching the simulation camera.

## Services

The chassis node exposes two private services:

```bash
rosservice call /chassis/enable "data: false"   # stop and disable motors
rosservice call /chassis/enable "data: true"    # allow motion again
rosservice call /chassis/reset_odom             # reset to initial_x/y/yaw
```

`reset_odom` returns the odometry to the configured start pose (default
`(1.6, 1.7, pi)`), matching the Gazebo spawn pose. Use the `initial_x`,
`initial_y` and `initial_yaw` launch arguments to start somewhere else.

The chassis watchdog sends a zero velocity frame when `/cmd_vel` stops
arriving for `cmd_timeout` seconds, and the shutdown handler sends 20 zero
frames before closing the port.

## Emergency stop

If ROS or the chassis node has already died, send zero-velocity frames
directly to the serial port. This does not need a ROS master:

```bash
rosrun aic26_cobot emergency_stop.py
# or without sourcing a workspace:
python3 ~/aic26_ws/src/aic26_cobot/scripts/emergency_stop.py \
  --port /dev/carserial
```

The script writes zero frames for 3 seconds by default. If the wheels still
turn, cut the motor power. A host process cannot stop a chassis whose firmware
keeps the last command, so the STM32 firmware must also implement its own
command timeout (for example 300 ms without a valid frame).

## Configuration

All defaults live in `config/` and can be overridden by launch arguments or
private parameters:

- `config/chassis.yaml`: port, baud, topics, frames, start pose, feedback
  timing/deadbands, scales, limits, IMU yaw fusion, command timeout.
- `config/imu.yaml`: port, baud, topics, frame, magnetic field switch.
- `config/lidar.yaml`: port, baud, topic, frame, bins, range limits, mounting
  correction and the 90..270 degree mask.
- `config/camera.yaml`: V4L2 device, autodetect, resolution, fps, fourcc,
  topics, frame, intrinsics and flips.
- `config/status.yaml`: monitored topics and staleness timeout.

The N10_P packet format is the single-echo 108-byte `A5 5A` frame with a CRC8
sum byte. The command frame is `AA BB 0A 12 02` followed by signed 16-bit
little-endian `x`, `y`, `yaw` in mm/s and mrad/s; the feedback frame is
`AA BB 0A 0A` plus the same three values. Both are documented in
`scripts/hardware_common.py`.

## Tests

The protocol encoders and decoders are pure functions with no ROS dependency,
so they can be tested without a master or hardware:

```bash
cd ~/aic26_ws/src/aic26_cobot
python3 -m unittest discover -s test -v

# or through catkin
cd ~/aic26_ws
catkin_make run_tests_aic26_cobot
```

## Safety notes

- The STM32 firmware must stop the motors on its own when valid command frames
  stop. The ROS watchdog only works while the chassis node is alive.
- Test with the wheels off the ground first.
- Verify `/cmd_vel` signs and the `angular_scale` value before driving.
- The two CH340 adapters are identical; keep them in the USB ports that match
  the udev rules, otherwise `/dev/carserial` and `/dev/imu` swap.
- The lidar is a full 360 degree N10_P. The mask parameters default to `-1`
  (disabled), matching the previously working car_bringup stack. Only set
  `mask_start_deg`/`mask_end_deg` if the mechanical mounting really blocks a
  sector; a half circle is not enough for gmapping/AMCL.
- The IMU magnetic field is published as raw counts unless `mag_scale` is
  calibrated; it is disabled by default.
- If the camera cannot be opened, check `v4l2-ctl --list-devices` and
  `/dev/video*`. A UVC camera behind a low-speed/unpowered hub can enumerate
  without a video node. The node scans `/dev/video*` and retries MJPG/YUYV when
  `autodetect` is true.
