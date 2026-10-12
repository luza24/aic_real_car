
# aic26_ws

**ROS1 catkin workspace for the AIC26 Smart Community competition.**

A TurtleBot-like differential drive robot (`armbot`) drives a simulated
community block: it follows a fixed inspection route, waits at traffic lights,
counts community people, reads car license plates and announces the results.
Everything runs in Gazebo with a saved 2D map for localization.

***Build:***

```bash
  source /opt/ros/noetic/setup.bash
  catkin_make 
  source devel/setup.bash
```

***Packages:***

| Package | Role |
| --- | --- |
| aic26_description | robot URDF/xacro (base, laser, camera, head, lights) |
| aic26_gazebo | world, models, traffic light controller + day/night sky scripts and Gazebo visual plugins |
| aic26_msgs | custom messages (TrafficLightCommand, TrafficLightState) |
| aic26_slam | gmapping launch and the saved map |
| aic26_navigation | map_server, amcl and move_base (global_planner + teb) |
| aic26_decision | mission controller, waypoint follower, showcase and dataset capture nodes |
| aic26_perception | ONNX detectors: traffic light, people, car + license plate OCR |
| aic26_utils | helper package (empty stub) |
| aic26_bringup | top level launch (single entry point) |

***Run the inspection mission (one click):***

```bash
  roslaunch aic26_bringup bringup.launch
```

`bringup.launch` is the single entry point. It takes a `mode` argument and
includes only the nodes that mode needs:

| mode | What it runs |
| --- | --- |
| `real` (default) | simulation + navigation + perception + `mission_controller` |
| `demo` | showcase loop: faked detection and spoken announcement, sky advances 4.5h per pass |
| `capture` | loop screenshot capture for the dataset (faked vision, no model needed) |
| `waypoints` | traverse every waypoint once, to check map / localization |
| `mapping` | gmapping on the simulated laser (drive with the spawned teleop) |

```bash
  # examples
  roslaunch aic26_bringup bringup.launch mode:=real
  roslaunch aic26_bringup bringup.launch mode:=demo day_night:=night
  roslaunch aic26_bringup bringup.launch mode:=capture
  roslaunch aic26_bringup bringup.launch mode:=waypoints use_rviz:=true
  roslaunch aic26_bringup bringup.launch mode:=mapping
```

***Common arguments*** (all modes):

- `world`       : world launch name under `aic26_gazebo/launch` (default `world`)
- `day_night`   : `day` or `night` initial sky (default `day`)
- `sky_time`    : explicit time of day in hours, overrides `day_night` (default empty)
- `map_file`    : map yaml for localization (default `aic26_slam/maps/world.yaml`)
- `use_rviz`    : start rviz (default `true`)
- `show_view`   : show the Gazebo camera viewer (default `true`)
- `nav_delay`   : seconds to wait before the navigation stack starts (default `8.0`)
- `task_delay`  : seconds to wait before the task nodes start (default `10.0`)

The two delays let the simulation come up before navigation, and navigation
before the task nodes. They are applied as a per-node start-up wait
(`launch-prefix="bash -c 'sleep <delay>; $0 $@'"`); set either to `0` to
disable the wait.

***Run the steps separately:***

```bash
  # simulation only (world + robot + traffic lights)
  roslaunch aic26_gazebo world.launch

  # mapping (teleop is spawned by the launch, drive around to map)
  roslaunch aic26_slam gmapping.launch

  # localization and navigation on top of the simulation
  roslaunch aic26_navigation navigation.launch

  # inspect mission on top of navigation (real vision + mission controller)
  roslaunch aic26_decision mission.launch

  # the other decision nodes, each on top of a running nav stack
  roslaunch aic26_decision waypoints.launch        # visit every waypoint once
  roslaunch aic26_decision demo.launch             # showcase loop
  roslaunch aic26_decision capture.launch # dataset capture loop
```

***Mission configuration:***

- `aic26_decision/config/waypoints.yaml` : named poses (`x`, `y`, `yaw`, `dwell`)
- `aic26_decision/config/tasks.yaml` : ordered task list for `mission_controller`;
  task types are `goto`, `traffic_light`, `inspect` and `report`
- `aic26_decision/config/capture_cycle.yaml` : dataset capture route, mirrors
  `tasks.yaml` waypoint by waypoint so captures match the real run

***Traffic light / day-night tools:***

```bash
  rosrun aic26_gazebo day_night.py _time:=22.0     # move the sky (one shot)
  rosrun aic26_gazebo day_night.py _gui:=true       # drag the sky live
  rosrun aic26_gazebo traffic_controller.py         # standalone light controller
```

***Perception:***

Detection nodes live in `aic26_perception`. They run int8 ONNX models
(`onnxruntime`, with a `cv2.dnn` fallback) and are launched by the mission.

```bash
  roslaunch aic26_perception traffic_light.launch   # aic26_msgs/TrafficLightState
  roslaunch aic26_perception people.launch          # counts community people
  roslaunch aic26_perception car.launch             # car box + license plate OCR
```

The model files are **not** shipped with the repository; see
`aic26_perception/models/README.md` for the expected file names, class order
and the `onnxruntime` / `easyocr` runtime dependencies. Nodes log a warning
and publish empty / `UNKNOWN` results when a model is missing.

***Interfaces:***

| Topic | Type | Producer |
| --- | --- | --- |
| /camera/image_raw | sensor_msgs/Image | gazebo camera |
| /scan | sensor_msgs/LaserScan | gazebo laser |
| /traffic_light/command | aic26_msgs/TrafficLightCommand | traffic_controller |
| /traffic_light/state | aic26_msgs/TrafficLightState | simulation truth |
| /perception/traffic_light | aic26_msgs/TrafficLightState | traffic_light_detector |
| /perception/result | std_msgs/String (json) | people_detector / car_detector |
| /perception/detect | std_msgs/String | mission_controller (detection trigger) |
| /perception/{traffic_light,people,car}/image | sensor_msgs/Image | detector debug frames |
| /speech | std_msgs/String | mission_controller / demo_cycle |
| /cmd_vel | geometry_msgs/Twist | move_base / in-place rotation |

***Output directories:***

- `.datasets/screenshots/` : labelled images saved by `capture_cycle` and the
  Gazebo capture viewer (git-ignored)
- `aic26_perception/results/` : live detection output written by the
  `mission_controller` GUI (see `aic26_perception/results/README.md`)
