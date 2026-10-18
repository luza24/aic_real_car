<p align="center">
  <img src="logo.svg" width="260" alt="AIC26 logo">
</p>

# AIC26 Smart Community Inspection Robot

ROS 1 (Noetic) catkin workspace for the AIC26 Smart Community challenge. A
four-wheel skid-steer robot (`armbot`) drives through a simulated community,
follows a fixed inspection route, waits at traffic lights, counts community
people, reads car license plates and reports the results.

The simulation backend runs in Gazebo: URDF/xacro robot, a community
simulation with traffic lights, a saved 2D map, AMCL localization, move_base,
and three int8 ONNX detectors. The `aic26_cobot` package provides the same
topics and frames for the real robot.

## Highlights

- One-command bringup with five modes (`mission`, `demo`, `capture`,
  `waypoints`, `mapping`) and a `backend:=sim|real` switch.
- A simulation backend (`aic26_gazebo`) and a real-robot backend
  (`aic26_cobot`) that publish the same topics and frames.
- Closed-loop waypoint navigation with TEB local planning, plus in-place
  rotation for inspection stops.
- Simulation-truth traffic lights with a visual plugin that changes the lens
  materials, and a day/night cycle driven by `/sky_time`.
- ONNX detectors for traffic lights, community people (with intruder classes)
  and cars (with license plate OCR).
- Dataset capture mode that drives the real route and saves labeled
  screenshots while varying the lighting.
- Pixel-sized Tk/OpenCV panels that keep their size on X11-forwarded displays.

## Repository layout

```text
aic26_ws/
|-- src/
|   |-- aic26_bringup/       top-level launch entry points
|   |-- aic26_cobot/         real-robot drivers (chassis, IMU, lidar, camera)
|   |-- aic26_decision/      mission controller, waypoint follower, demo/capture
|   |-- aic26_description/   URDF/xacro robot model
|   |-- aic26_gazebo/        Gazebo assets, models, plugins, controller and tools
|   |-- aic26_msgs/          traffic light command and state messages
|   |-- aic26_navigation/    AMCL and move_base configuration
|   |-- aic26_perception/    ONNX detectors and model files
|   |-- aic26_slam/          gmapping launch and the saved map
|   `-- aic26_utils/         shared utilities and camera viewer
|-- logo.svg
`-- README.md
```

`build/`, `devel/`, `.datasets/`, `.drafts/`, `.vscode/` and `.cline/` are
generated or local-only and are ignored by Git.

## Requirements

- Ubuntu 20.04 with ROS Noetic and Gazebo 11.
- ROS packages: `gazebo_ros`, `gazebo_plugins`, `robot_state_publisher`,
  `xacro`, `rviz`, `amcl`, `map_server`, `move_base`, `global_planner`,
  `teb_local_planner`, `gmapping`, `teleop_twist_keyboard`, `cv_bridge`.
- Python 3 packages: `rospy`, `rospkg`, `numpy`, `PyYAML` and OpenCV
  (`python3-opencv` from ROS is recommended).
- `python3-tk` for the Tk panels and `pillow` for the result viewer image
  panel (the panel is disabled without Pillow).
- `openvino` (preferred on the Intel N100) or `onnxruntime` for the int8
  models, and `easyocr` for license plate recognition.
- `python3-serial` for the `aic26_cobot` hardware drivers (real robot only).

Install the declared ROS dependencies from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Install the optional Python runtime dependencies:

```bash
# preferred on the Intel CPU: ~10 FPS full pipeline on the N100
python3 -m pip install --user openvino easyocr pillow

# portable fallback: ~3-4 FPS on the same machine
# python3 -m pip install --user onnxruntime
```

`easyocr` pulls in PyTorch (a large download) and fetches its detector and
recognition weights on first use (`~/.EasyOCR`), so the first plate read needs
network access or a pre-populated cache.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Quick start

```bash
roslaunch aic26_bringup bringup.launch
```

This starts the default `mission` mode on the simulation backend: Gazebo,
navigation, the three perception nodes and `mission_controller` with its result
viewer. The robot drives the inspection route defined in
`aic26_decision/config/tasks.yaml`.

## Run modes

`bringup.launch` is the single entry point. `mode` selects what runs and
`backend` selects where it runs.

```bash
roslaunch aic26_bringup bringup.launch mode:=mission backend:=sim
```

`backend` accepts two values:

| backend | Description |
| --- | --- |
| `sim` (default) | Gazebo simulation, simulated robot, simulated sensors |
| `real` | `aic26_cobot` hardware drivers on the real robot |

The five modes are:

| mode | What it runs |
| --- | --- |
| `mission` (default) | navigation + perception + `mission_controller` |
| `demo` | navigation + `demo_cycle` (faked results, no models) |
| `capture` | navigation + `capture_cycle` (dataset screenshots) |
| `waypoints` | navigation + `waypoint_follower` |
| `mapping` | gmapping + teleop + rviz (no navigation stack) |

`mode:=real` is accepted as an alias for `mode:=mission`.

Examples:

```bash
# mission in the simulation (default)
roslaunch aic26_bringup bringup.launch

# mission on the real robot
roslaunch aic26_bringup bringup.launch mode:=mission backend:=real
roslaunch aic26_bringup bringup.launch mode:=mission backend:=real teleop:=true

# other modes, on either backend
roslaunch aic26_bringup bringup.launch mode:=demo backend:=sim day_night:=night
roslaunch aic26_bringup bringup.launch mode:=capture backend:=sim use_rviz:=false
roslaunch aic26_bringup bringup.launch mode:=waypoints backend:=real
roslaunch aic26_bringup bringup.launch mode:=mapping backend:=real
```

`demo` walks the route continuously, publishes a faked detection result and
announcement, and advances the sky by 4.5 hours per pass. `capture` repeats
the route until every stop has its requested number of screenshots; its extra
settings (`sky_step`, `until_done`) live in
`src/aic26_bringup/launch/capture.launch`. On the hardware backend, `sky_step`
defaults to `0.0` because there is no Gazebo sky to advance.

In `mapping` mode the chassis odometry starts at `start_x/y/yaw` (default
`1.6 / 1.7 / pi`) instead of `(0, 0, 0)`. GMAPPING therefore builds the map
with the robot at the same coordinates as the simulated spawn/navigation map,
so the car does not jump from `(0, 0)` to `(1.6, 1.7)` when switching between
mapping and mission mode.

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `mode` | `mission` | one of the five modes above (`real` aliases `mission`) |
| `backend` | `sim` | `sim` for Gazebo, `real` for `aic26_cobot` |
| `sim` | `sim` | simulation launch file under `aic26_gazebo/launch` |
| `day_night` | `day` | simulation sky: `day` or `night` |
| `sky_time` | empty | simulation time of day in hours, overrides `day_night` |
| `map_file` | `aic26_slam/maps/world.yaml` | map for AMCL / map_server (not used in mapping mode) |
| `use_rviz` | `true` | start RViz with the navigation display (also honored in mapping) |
| `show_view` | `true` | start the simulation camera viewer |
| `nav_delay` | `15.0` | seconds before navigation (or gmapping) starts |
| `task_delay` | `20.0` | seconds before the mission / task nodes start (unused in mapping mode) |
| `set_initial_pose` | `true` | initialize AMCL at `initial_pose_*` |
| `initial_pose_x` | `1.6` | initial map x |
| `initial_pose_y` | `1.7` | initial map y |
| `initial_pose_a` | `3.141592` | initial map yaw in radians |

The delays are applied as a per-node start-up wait
(`launch-prefix="bash -c 'sleep <delay>; $0 $@'"`); set either to `0` to
disable the wait. This lets the backend come up before navigation, and
navigation before the mission nodes. The defaults are deliberately generous
for slower machines; on a fast machine use e.g. `nav_delay:=5 task_delay:=10`.

`sim`, `day_night`, `sky_time` and `show_view` only apply to
`backend:=sim`. `backend:=real` adds these arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_imu_yaw` | `false` | fuse the WIT yaw into the chassis odometry (off by default; magnetic yaw is noisy indoors) |
| `chassis_port` | `/dev/carserial` | STM32 chassis serial device |
| `imu_port` | `/dev/imu` | WIT IMU serial device |
| `lidar_port` | `/dev/lslidar` | N10_P lidar serial device |
| `camera_device` | `/dev/video0` | UVC RGB camera device |
| `teleop` | `false` | start `teleop_twist_keyboard` |
| `gui` | `true` | start the mission result viewer |
| `light_fallback` | `traffic_light` on real, empty on sim | generic vision light name used when there is no simulation truth |

## Packages

| Package | Role |
| --- | --- |
| `aic26_bringup` | top-level launch entry points for every run mode |
| `aic26_cobot` | real-robot hardware drivers: chassis, IMU, lidar, camera, diagnostics |
| `aic26_description` | xacro/URDF robot: base, four wheels, laser, camera, headlights |
| `aic26_gazebo` | community simulation assets, models, traffic light controller, day/night and C++ visual plugins |
| `aic26_msgs` | `TrafficLightCommand` and `TrafficLightState` messages |
| `aic26_slam` | gmapping launch and the saved 2D map |
| `aic26_navigation` | map_server, AMCL and move_base (global_planner + TEB) |
| `aic26_decision` | mission controller, waypoint follower, demo and capture cycles |
| `aic26_perception` | traffic light, people and car/plate ONNX detectors |
| `aic26_utils` | shared utilities (camera viewer) |

## Robot and simulation

- `armbot` is a four-wheel skid-steer robot. It is spawned at
  `(1.6, 1.7, yaw=pi)` and publishes `/odom` and `/tf`.
- Sensors: a 1280x720 camera at 30 Hz and a 720-beam laser with a 270 degree
  field of view and 12 m range.
- The saved map `aic26_slam/maps/world.yaml` is 224 x 224 cells at 0.05 m per
  cell (11.2 m x 11.2 m), with origin `(-6, -6)`. AMCL starts at the spawn
  pose.
- The simulated community contains two traffic lights, three car boards,
  sixteen community person boards and two non-community boards.
- Traffic light timing is configured in `aic26_gazebo/launch/sim.launch`:
  RED 10 s, GREEN 15 s, YELLOW 3 s, with light 2 offset by 10 s. The
  standalone controller uses 30/25/5 s and a 15 s offset when no parameters
  are given.
- The sky starts at 10:00. `day_night` selects 12:00 or 00:00, and `sky_time`
  sets any hour. Sunrise is 6:00 and sunset is 20:00; the sun and the robot
  headlights switch accordingly.

The real-robot backend in `aic26_cobot` uses the same map and frame names:
`odom -> base_footprint -> base_link -> laser/camera/imu_link`. Its lidar has a
4 m range and publishes the full 360 degree scan (the mask is disabled), the
camera publishes 1280x720 RGB, and AMCL starts at the configured initial pose
(`initial_pose_x/y/a`). The chassis odometry starts at the same pose, so
`map -> odom` stays near identity and the real robot appears at the same map
coordinates as the simulated spawn pose `(1.6, 1.7, pi)`. See
`src/aic26_cobot/README.md` for the device ports and udev rules.

The chassis feedback is integrated at the measured 20 Hz frame rate rather
than the USB arrival time (the adapter delivers frames in bursts), and IMU yaw
corrections are filtered and limited to `imu_correction_rate` updates per
second. The IMU offset is relative to the configured start yaw, so enabling
`use_imu_yaw:=true` does not rotate the robot back to yaw zero. This keeps the
odometry TF smooth, which in turn stops GMAPPING/AMCL from publishing
high-frequency pose corrections that look like a shaking robot in RViz.

AMCL is tuned for the N100 in `aic26_navigation/config/amcl_params.yaml`:
300-1200 particles, 45 laser beams, `kld_err: 0.08` and a 2 Hz particle-cloud
publish rate. In a forced-update benchmark this reduced AMCL CPU by about
25-30% while the map pose stayed within a few millimetres of the start pose.
The laser model, resampling interval and `update_min_*` thresholds are
unchanged.

## Mission configuration

- `aic26_decision/config/waypoints.yaml` defines named poses with `x`, `y`,
  `yaw` and `dwell` fields.
- `aic26_decision/config/tasks.yaml` is the ordered task list used by
  `mission_controller`:
  - `goto`: navigate to a waypoint and continue.
  - `traffic_light`: wait at the stop line until the light is GREEN, then pass.
  - `inspect`: navigate, optionally turn in place, dwell, trigger a detector
    and report the result.
  - `report`: announce a collected result without moving.
- `aic26_decision/config/screenshot_cycle.yaml` mirrors the mission route for
  `capture_cycle`, stop by stop, so captures use the same viewpoints. Each
  stop has a `label`, `shots`, `shots_per_pass` and a faked `result` JSON.
- `mission_controller` publishes announcements on `/speech`
  (`std_msgs/String`). No text-to-speech node is included; connect any TTS
  consumer to that topic.
- On real hardware there is no simulation light truth. `backend:=real` sets
  `light_fallback:=traffic_light` for `mode:=mission`, so the mission uses the
  generic vision state when no state exists for the requested
  `traffic_light_1/2` model.
- With `~gui:=true` (the default in `aic26_decision/launch/mission.launch`),
  the mission opens a Tk result viewer. It shows the latest annotated camera
  frame and a summary, and writes the latest results to
  `aic26_perception/results/`.

## Perception

The detectors run int8 ONNX models included in
`aic26_perception/models/`. All models use the YOLOv5/v8 output layout
`[1, 4 + num_classes, num_anchors]` (or its transpose) with a letterboxed
640x640 input.

| Model | Detector | Classes |
| --- | --- | --- |
| `traffic_int8.onnx` | `traffic_light_detector.py` | `red_off`, `red_on`, `green_off`, `green_on`, `yellow_off`, `yellow_on` |
| `people_int8.onnx` | `people_detector.py` | `p1`..`p16`, `pF1`, `pF2` (`pF1`/`pF2` are non-community) |
| `car_int8.onnx` | `car_detector.py` | `car` |

Notes:

- OpenVINO is the primary backend (CPU, about 10 FPS for the full
  letterbox + inference + decode pipeline on the N100). `onnxruntime` is the
  fallback (about 3-4 FPS). `cv2.dnn` is kept for float models only; it cannot
  execute the int8 quantisation used here.
- There is no usable GPU on the car PC: the N100 has no NVIDIA GPU and its
  Intel iGPU has no OpenCL runtime, so OpenVINO runs on the CPU. Set
  `AIC26_OV_THREADS` to change the OpenVINO thread count (default 2).
- A detector runs only while its target is requested on `/perception/detect`
  (`traffic_light`, `people` or `car`). Any other message stops it. While
  active it processes the latest frame at `~max_rate` (default 5 Hz), so the
  30 FPS camera stream does not build a queue.
- People are counted once per class within a trigger, so repeated boxes of the
  same person do not inflate the count.
- The car detector crops a fixed plate region from the car box
  (`~plate_roi`, default `[0.30, 0.40, 0.70, 0.60]`) and reads it with
  easyocr. Plate text is published as UTF-8.
- A missing model produces a warning and empty / `UNKNOWN` results instead of
  crashing the node.
- `traffic_light_detector` reports the model name from `~node_name` (launch
  argument `light_name`). Set it to `traffic_light_1` or `traffic_light_2` for
  a detector dedicated to one light. The real-robot launch keeps the generic
  `traffic_light` name and sets `light_fallback` so the mission can use it
  without simulation truth.

## Interfaces

The `aic26_gazebo` and `aic26_cobot` backends share the core topics and frame
names. The table lists both producers where they differ.

| Topic | Type | Producer / consumer |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera / `aic26_cobot` camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera / `aic26_cobot` camera |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo laser / `aic26_cobot` lidar |
| `/odom` | `nav_msgs/Odometry` | skid-steer plugin / `aic26_cobot` chassis |
| `/vel_raw` | `geometry_msgs/Twist` | `aic26_cobot` chassis (measured velocity) |
| `/imu/data` | `sensor_msgs/Imu` | `aic26_cobot` IMU |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `aic26_cobot` status monitor |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base, in-place rotation, teleop |
| `/map` | `nav_msgs/OccupancyGrid` | map_server |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher, Gazebo / chassis, AMCL |
| `/traffic_light/command` | `aic26_msgs/TrafficLightCommand` | traffic_controller to visual plugin (simulation) |
| `/traffic_light/state` | `aic26_msgs/TrafficLightState` | traffic_controller (simulation truth) |
| `/perception/traffic_light` | `aic26_msgs/TrafficLightState` | traffic_light_detector |
| `/perception/result` | `std_msgs/String` (JSON) | people/car detectors and the demo/capture cycles |
| `/perception/detect` | `std_msgs/String` | mission_controller, demo_cycle, capture_cycle |
| `/perception/traffic_light/image` | `sensor_msgs/Image` | traffic light debug frame |
| `/perception/people/image` | `sensor_msgs/Image` | people debug frame |
| `/perception/car/image` | `sensor_msgs/Image` | car debug frame |
| `/speech` | `std_msgs/String` | mission_controller and demo_cycle |

## GUI over SSH and HiDPI displays

If the robot model in RViz stays still while the map/costmaps move, set
**Global Options → Fixed Frame** to `map` (not `base_footprint`, `base_link` or
`odom`). The supplied `navigation.rviz` already uses `map`. Also check:

```bash
rosparam get /use_sim_time                 # must be false on real hardware
rosrun tf tf_echo map base_footprint       # robot pose in the map
rosrun tf tf_echo odom base_footprint      # wheel-odometry pose
rostopic echo -n1 /odom/pose/pose/position
```

If `odom -> base_footprint` does not change while the robot moves, the chassis
odometry/encoder feedback is not updating. If `map -> odom` changes but the
robot model does not, fix the RViz fixed frame first.

The Tk panels (`day_night.py`, the result viewer) and the camera viewer are
sized in pixels, so they keep their size on a local screen and over
`ssh -X`. Scale them with `AIC26_GUI_SCALE` (or the `~gui_scale` parameter):

```bash
AIC26_GUI_SCALE=1.5 roslaunch aic26_bringup bringup.launch
```

The camera window is `aic26_utils/scripts/camera_view.py` with `view_width`
and `view_height` arguments (default 640x360). It is started for both
`backend:=sim` and `backend:=real` when `show_view:=true`. The stock
`image_view` node is not used: it creates its window at OpenCV's 320x240
default and resizes it only after mapping, which an X11-forwarded display
ignores.

`camera_view` is CPU-rendered here: OpenCV 4.2 on this machine is built
without OpenGL and the N100 has no usable GPU, so an OpenGL/RViz viewer would
still fall back to llvmpipe. To keep it cheap in mapping, the camera runs at
640x480@15 and the viewer converts at 15 Hz; mission/demo keep the full
1280x720@30 stream for image recognition.

The result viewer needs `tkinter`; its image panel additionally needs Pillow
and is disabled with a warning when Pillow is unavailable.

## Tools

```bash
# move the sky (one shot)
rosrun aic26_gazebo day_night.py _time:=22.0
rosrun aic26_gazebo day_night.py _mode:=night

# drag the sky live
rosrun aic26_gazebo day_night.py _gui:=true

# standalone traffic light controller (default 30/25/5 s, 15 s offset)
rosrun aic26_gazebo traffic_controller.py

# camera viewer with pixel-sized window
rosrun aic26_utils camera_view.py

# emergency stop: zero velocity frames, no ROS master needed
rosrun aic26_cobot emergency_stop.py
python3 src/aic26_cobot/scripts/emergency_stop.py --port /dev/carserial
```

## Running the steps separately

```bash
# simulation only: sim, robot, traffic controller, day/night GUI
roslaunch aic26_gazebo sim.launch

# real hardware backend only: chassis, IMU, lidar, camera, diagnostics
roslaunch aic26_cobot hardware.launch

# mapping with gmapping and the spawned teleop
roslaunch aic26_slam gmapping.launch

# localization and navigation on top of the simulation
roslaunch aic26_navigation navigation.launch

# localization and navigation on top of the real hardware backend
roslaunch aic26_navigation navigation.launch \
  laser_min_range:=0.2 laser_max_range:=4.0 \
  initial_pose_x:=1.6 initial_pose_y:=1.7 initial_pose_a:=3.141592

# mission on top of a running navigation stack
roslaunch aic26_decision mission.launch

# the other decision nodes, each on top of a running navigation stack
roslaunch aic26_decision waypoints.launch
roslaunch aic26_decision demo.launch
roslaunch aic26_decision capture.launch
```

## Output directories

- `.datasets/screenshots/<label>/<label>_%04i.jpg`: images saved by
  `capture_cycle` (git-ignored, resumable and never overwritten). The output
  directory is the `output_dir` argument of
  `aic26_decision/launch/capture.launch`.
- `aic26_perception/results/`: latest detection results and annotated frames
  written by the mission result viewer. The committed files are example
  snapshots and are overwritten at runtime.

## Conventions

- UTF-8 source files with LF line endings and a final newline (enforced by
  `.gitattributes`); no trailing whitespace.
- Comments and identifiers are ASCII. Chinese characters appear only in the
  license plate allowlist and the plate test data, where they are required.
- Python follows PEP 8: four-space indentation, lines up to 79 columns.
- Python entry points carry no shebang; `catkin_install_python` generates the
  interpreter wrapper when the workspace is built.
- XML, launch, xacro and SDF files use two-space indentation.

## Troubleshooting

- `catkin_make` cannot find `teb_local_planner` or `global_planner`: install
  the ROS dependencies with
  `rosdep install --from-paths src --ignore-src -r -y`.
- The detectors start but never return results: install `openvino`
  (preferred) or `onnxruntime`. `cv2.dnn` cannot run the int8 models.
- The car detector returns an empty plate: install `easyocr` and allow the
  first run to download its weights, or populate `~/.EasyOCR` in advance.
- No GUI appears over SSH: use `ssh -X`, or disable the windows with
  `show_view:=false use_rviz:=false`. Use `AIC26_GUI_SCALE` if they are too
  small or too large.
- A hardware device cannot be opened: check `python3-serial`, the udev rules
  and the user's membership in the `dialout` group. Use
  `roslaunch aic26_cobot hardware.launch chassis_port:=...` to test a path.
- The camera warning `cannot open /dev/video0`: check
  `v4l2-ctl --list-devices` and `ls /dev/video*`. If no video node exists, the
  USB link is the problem (unpowered hub, low-speed enumeration, or power);
  connect the camera directly to a USB 3 port. The camera node scans
  `/dev/video*` and retries MJPG/YUYV, so a different index is handled
  automatically. Set `camera_device:=/dev/videoN` to force one.
- The chassis keeps moving after roslaunch exits: run
  `rosrun aic26_cobot emergency_stop.py`, or
  `python3 src/aic26_cobot/scripts/emergency_stop.py` when ROS is down, then
  cut motor power if it still turns. The STM32 firmware needs its own command
  timeout.
- AMCL does not localize on hardware: make sure `initial_pose_x/y/a` matches the
  robot's physical start pose. The default is `(1.6, 1.7, pi)`. Set
  `set_initial_pose:=false` to initialize manually in RViz. The real lidar
  range is 4 m, so keep `laser_max_range:=4.0`.
- The robot still shakes in RViz on real hardware: check that the chassis
  `initial_x/y/yaw` match the AMCL `initial_pose_*` (the mode launch files pass
  them through automatically). The real AMCL config uses small
  `initial_cov_*` values and `update_min_d/a: 0.10` so the first scan does not
  pull the pose away from the spawn point and corrections are not published on
  every scan. Mapping uses larger GMAPPING `linearUpdate/angularUpdate` values
  for the same reason.
- Traffic lights stay dark in simulation: gzclient must finish loading the
  simulation before the visual plugin can apply materials. The controller waits
  `~startup_delay` (6 s in `sim.launch`) before starting.
- Task nodes start too early: increase `nav_delay` / `task_delay`, or check
  that the `map -> base_footprint` transform is available.
- An unknown `mode` or `backend` starts no nodes; use one of the five modes and
  `backend:=sim|real`.

## Scope

This repository contains a simulated competition stack and the hardware
drivers for the real robot. The simulation needs Gazebo; `backend:=real`
needs the actual chassis, IMU, lidar and camera. Robot hardware drivers beyond
these devices, a SLAM back end beyond gmapping, and a text-to-speech engine are
not included; `/speech` is an interface for an external consumer.
