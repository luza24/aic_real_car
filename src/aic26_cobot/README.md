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

`urdf/aic26_cobot.urdf.xacro` provides the fixed TF tree
`base_footprint -> base_link -> laser/camera/imu_link` plus
`camera -> camera_optical_frame`. The dimensions mirror `aic26_description`,
so the real and simulated robots share frame names. The chassis node publishes
`odom -> base_footprint`; `robot_state_publisher` publishes the fixed links.

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
  set_initial_pose:=false laser_min_range:=0.2 laser_max_range:=4.0
```

With `set_initial_pose:=false`, set the robot pose once with the RViz
"2D Pose Estimate" tool. The hardware path uses the real lidar range (4 m), not
the 12 m simulation range.

Useful switches:

```bash
# disable individual devices
roslaunch aic26_cobot hardware.launch camera:=false

# do not fuse the IMU yaw into the chassis odometry
roslaunch aic26_cobot hardware.launch use_imu_yaw:=false

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
- The hardware mission leaves AMCL without an initial pose and uses the real
  lidar range; see `aic26_bringup/launch/mission.launch` and
  `aic26_bringup/launch/backend.launch`.
- The real camera publishes 1280x720 images, matching the simulation camera.

## Services

The chassis node exposes two private services:

```bash
rosservice call /chassis/enable "data: false"   # stop and disable motors
rosservice call /chassis/enable "data: true"    # allow motion again
rosservice call /chassis/reset_odom             # reset x, y, yaw to zero
```

The chassis watchdog sends a zero velocity frame when `/cmd_vel` stops
arriving for `cmd_timeout` seconds, and the shutdown handler sends several zero
frames before closing the port.

## Configuration

All defaults live in `config/` and can be overridden by launch arguments or
private parameters:

- `config/chassis.yaml`: port, baud, topics, frames, scales, limits, IMU yaw
  fusion, command timeout.
- `config/imu.yaml`: port, baud, topics, frame, magnetic field switch.
- `config/lidar.yaml`: port, baud, topic, frame, bins, range limits, mounting
  correction and the 90..270 degree mask.
- `config/camera.yaml`: V4L2 device, resolution, fps, fourcc, topics, frame,
  intrinsics and flips.
- `config/status.yaml`: monitored topics and staleness timeout.

The N10_P packet format is the single-echo 108-byte `A5 5A` frame with a CRC8
sum byte. The chassis frame is `AA BB 0A 12 02` followed by signed 16-bit
little-endian `x`, `y`, `yaw` in mm/s and mrad/s, and the feedback frame is
`AA BB 0A 12` plus the same three values. Both are documented in
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

- Test with the wheels off the ground first.
- Verify `/cmd_vel` signs and the `angular_scale` value before driving.
- The two CH340 adapters are identical; keep them in the USB ports that match
  the udev rules, otherwise `/dev/carserial` and `/dev/imu` swap.
- The lidar mask defaults to 90..270 degrees to match the car mounting. Set
  both mask values to `-1` to publish the full circle.
- The IMU magnetic field is published as raw counts unless `mag_scale` is
  calibrated; it is disabled by default.
