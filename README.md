<p align="center">
  <img src="logo.svg" width="260" alt="AIC26 logo">
</p>

# AIC26 Smart Community Inspection Robot

ROS 1 (Noetic) catkin workspace for the AIC26 Smart Community challenge. A
four-wheel skid-steer robot (`armbot`) drives through a simulated community,
follows a fixed inspection route, waits at traffic lights, counts community
people, reads car license plates and reports the results.

The stack runs entirely in Gazebo: URDF/xacro robot, a textured community
world with traffic lights, a saved 2D map, AMCL localization, move_base, and
three int8 ONNX detectors.

## Highlights

- One-command bringup with five run modes: `real`, `demo`, `capture`,
  `waypoints` and `mapping`.
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
|   |-- aic26_decision/      mission controller, waypoint follower, demo/capture
|   |-- aic26_description/   URDF/xacro robot model
|   |-- aic26_gazebo/        world, models, plugins, controller and GUI tools
|   |-- aic26_msgs/          traffic light command and state messages
|   |-- aic26_navigation/    AMCL and move_base configuration
|   |-- aic26_perception/    ONNX detectors and model files
|   |-- aic26_slam/          gmapping launch and the saved map
|   `-- aic26_utils/         placeholder package (currently empty)
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
- `onnxruntime` for the int8 models and `easyocr` for license plate
  recognition.

Install the declared ROS dependencies from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Install the optional Python runtime dependencies:

```bash
python3 -m pip install --user onnxruntime easyocr pillow
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

This starts the full `real` mission: Gazebo, navigation, the three perception
nodes and `mission_controller` with its result viewer. The robot drives the
inspection route defined in `aic26_decision/config/tasks.yaml`.

## Run modes

`bringup.launch` is the single entry point. The `mode` argument selects the
launch file and nodes to run.

| mode | What it runs |
| --- | --- |
| `real` (default) | simulation + navigation + perception + `mission_controller` |
| `demo` | simulation + navigation + `demo_cycle` (faked results, no models) |
| `capture` | simulation + navigation + `capture_cycle` (dataset screenshots) |
| `waypoints` | simulation + navigation + `waypoint_follower` |
| `mapping` | simulation + gmapping + teleop + rviz (no navigation stack) |

Examples:

```bash
roslaunch aic26_bringup bringup.launch mode:=real
roslaunch aic26_bringup bringup.launch mode:=demo day_night:=night
roslaunch aic26_bringup bringup.launch mode:=capture use_rviz:=false
roslaunch aic26_bringup bringup.launch mode:=waypoints show_view:=false
roslaunch aic26_bringup bringup.launch mode:=mapping
```

`demo` walks the route continuously, publishes a faked detection result and
announcement, and advances the sky by 4.5 hours per pass. `capture` repeats
the route until every stop has its requested number of screenshots; its extra
settings (`sky_step`, `until_done`) live in
`src/aic26_bringup/launch/capture.launch`.

## Common launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `world` | launch file base name under `aic26_gazebo/launch` |
| `day_night` | `day` | initial sky: `day` or `night` |
| `sky_time` | empty | explicit time of day in hours (wrapped into 0-24), overrides `day_night` |
| `map_file` | `aic26_slam/maps/world.yaml` | map for AMCL / map_server (not used in mapping mode) |
| `use_rviz` | `true` | start RViz with the navigation display (mapping always starts RViz) |
| `show_view` | `true` | start the camera viewer window |
| `nav_delay` | `8.0` | seconds before navigation (or gmapping) starts |
| `task_delay` | `10.0` | seconds before the mission / task nodes start (unused in mapping mode) |

The delays are applied as a per-node start-up wait
(`launch-prefix="bash -c 'sleep <delay>; $0 $@'"`); set either to `0` to
disable the wait. This lets the simulation come up before navigation, and
navigation before the mission nodes.

## Packages

| Package | Role |
| --- | --- |
| `aic26_bringup` | top-level launch entry points for every run mode |
| `aic26_description` | xacro/URDF robot: base, four wheels, laser, camera, headlights |
| `aic26_gazebo` | community world, models, traffic light controller, day/night and camera tools, C++ visual plugins |
| `aic26_msgs` | `TrafficLightCommand` and `TrafficLightState` messages |
| `aic26_slam` | gmapping launch and the saved 2D map |
| `aic26_navigation` | map_server, AMCL and move_base (global_planner + TEB) |
| `aic26_decision` | mission controller, waypoint follower, demo and capture cycles |
| `aic26_perception` | traffic light, people and car/plate ONNX detectors |
| `aic26_utils` | empty placeholder package |

## Robot and world

- `armbot` is a four-wheel skid-steer robot. It is spawned at
  `(1.6, 1.7, yaw=pi)` and publishes `/odom` and `/tf`.
- Sensors: a 1280x720 camera at 30 Hz and a 720-beam laser with a 270 degree
  field of view and 12 m range.
- The saved map `aic26_slam/maps/world.yaml` is 224 x 224 cells at 0.05 m per
  cell (11.2 m x 11.2 m), with origin `(-6, -6)`. AMCL starts at the spawn
  pose.
- The world contains two traffic lights, three car boards, sixteen community
  person boards and two non-community boards.
- Traffic light timing is configured in `aic26_gazebo/launch/world.launch`:
  RED 10 s, GREEN 15 s, YELLOW 3 s, with light 2 offset by 10 s. The
  standalone controller uses 30/25/5 s and a 15 s offset when no parameters
  are given.
- The sky starts at 10:00. `day_night` selects 12:00 or 00:00, and `sky_time`
  sets any hour. Sunrise is 6:00 and sunset is 20:00; the sun and the robot
  headlights switch accordingly.

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

- `onnxruntime` is the primary backend. `cv2.dnn` is kept as a fallback but
  cannot execute the int8 quantisation used by these models.
- A detector runs only while its target is requested on `/perception/detect`
  (`traffic_light`, `people` or `car`). Any other message stops it.
- People are counted once per class within a trigger, so repeated boxes of the
  same person do not inflate the count.
- The car detector crops a fixed plate region from the car box
  (`~plate_roi`, default `[0.30, 0.40, 0.70, 0.60]`) and reads it with
  easyocr. Plate text is published as UTF-8.
- A missing model produces a warning and empty / `UNKNOWN` results instead of
  crashing the node.
- `traffic_light_detector` reports the model name from `~node_name` (launch
  argument `light_name`). Set it to `traffic_light_1` or `traffic_light_2` for
  a detector dedicated to one light; the mission falls back to simulation
  truth when no vision state matches the requested light.

## Interfaces

| Topic | Type | Producer / consumer |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo laser |
| `/odom` | `nav_msgs/Odometry` | skid-steer drive plugin |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base and in-place rotation |
| `/map` | `nav_msgs/OccupancyGrid` | map_server |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher, Gazebo, AMCL |
| `/traffic_light/command` | `aic26_msgs/TrafficLightCommand` | traffic_controller to visual plugin |
| `/traffic_light/state` | `aic26_msgs/TrafficLightState` | traffic_controller (simulation truth) |
| `/perception/traffic_light` | `aic26_msgs/TrafficLightState` | traffic_light_detector |
| `/perception/result` | `std_msgs/String` (JSON) | people/car detectors and the demo/capture cycles |
| `/perception/detect` | `std_msgs/String` | mission_controller, demo_cycle, capture_cycle |
| `/perception/traffic_light/image` | `sensor_msgs/Image` | traffic light debug frame |
| `/perception/people/image` | `sensor_msgs/Image` | people debug frame |
| `/perception/car/image` | `sensor_msgs/Image` | car debug frame |
| `/speech` | `std_msgs/String` | mission_controller and demo_cycle |

## GUI over SSH and HiDPI displays

The Tk panels (`day_night.py`, the result viewer) and the camera viewer are
sized in pixels, so they keep their size on a local screen and over
`ssh -X`. Scale them with `AIC26_GUI_SCALE` (or the `~gui_scale` parameter):

```bash
AIC26_GUI_SCALE=1.5 roslaunch aic26_bringup bringup.launch
```

The camera window is `aic26_gazebo/scripts/camera_view.py` with `view_width`
and `view_height` arguments (default 640x360). The stock `image_view` node is
not used: it creates its window at OpenCV's 320x240 default and resizes it
only after mapping, which an X11-forwarded display ignores.

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
rosrun aic26_gazebo camera_view.py
```

## Running the steps separately

```bash
# simulation only: world, robot, traffic controller, day/night GUI
roslaunch aic26_gazebo world.launch

# mapping with gmapping and the spawned teleop
roslaunch aic26_slam gmapping.launch

# localization and navigation on top of the simulation
roslaunch aic26_navigation navigation.launch

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
- XML, launch, xacro and SDF files use two-space indentation.

## Troubleshooting

- `catkin_make` cannot find `teb_local_planner` or `global_planner`: install
  the ROS dependencies with
  `rosdep install --from-paths src --ignore-src -r -y`.
- The detectors start but never return results: install `onnxruntime`.
  `cv2.dnn` cannot run the int8 models.
- The car detector returns an empty plate: install `easyocr` and allow the
  first run to download its weights, or populate `~/.EasyOCR` in advance.
- No GUI appears over SSH: use `ssh -X`, or disable the windows with
  `show_view:=false use_rviz:=false`. Use `AIC26_GUI_SCALE` if they are too
  small or too large.
- Traffic lights stay dark: gzclient must finish loading the world before the
  visual plugin can apply materials. The controller waits `~startup_delay`
  (6 s in `world.launch`) before starting.
- Task nodes start too early: increase `nav_delay` / `task_delay`, or check
  that the `map -> base_footprint` transform is available.
- An unknown `mode` starts no nodes; use one of the five modes listed above.

## Scope

This repository is a simulation-only competition stack. It does not include
robot hardware drivers, a SLAM back end beyond gmapping, or a text-to-speech
engine; `/speech` is an interface for an external consumer.
