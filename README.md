
# aic26_ws

**ROS1 catkin workspace for the AIC26 Smart Community competition.**

***Build:***

```bash
  source /opt/ros/noetic/setup.bash
  catkin_make 
  source devel/setup.bash
```

***Packages:***

| Package | Role |
| --- | --- |
| aic26_description | robot URDF/xacro (base, laser, camera, lights) |
| aic26_gazebo | world, models and traffic light controller |
| aic26_msgs | custom messages (TrafficLightCommand, TrafficLightState) |
| aic26_slam | gmapping launch and saved map |
| aic26_navigation | map_server, amcl and move_base (global_planner + teb) |
| aic26_decision | mission controller and waypoint follower |
| aic26_perception | vision stub (interface only) |
| aic26_ocr | ocr stub (interface only) |
| aic26_bringup | top level launch |

***Run the inspection mission (one click):***

```bash
  roslaunch aic26_bringup aic26_bringup.launch
```

Optional arguments: `use_navigation`, `use_mission`, `use_perception`,
`use_rviz` (all default to `true`) and `map_file`.

***Run the steps separately:***

```bash
  # simulation only
  roslaunch aic26_gazebo aic26-1.launch

  # mapping (needs teleop to drive around)
  roslaunch aic26_slam gmapping.launch

  # localization and navigation
  roslaunch aic26_navigation navigation.launch

  # inspection mission on top of navigation
  roslaunch aic26_decision mission.launch
```

***Mission configuration:***

- `aic26_decision/config/waypoints.yaml` : named poses (`x`, `y`, `yaw`, `dwell`)
- `aic26_decision/config/tasks.yaml` : ordered task list, task types are
  `goto`, `traffic_light`, `inspect` and `report`

***Interfaces reserved for vision:***

| Topic | Type | Producer |
| --- | --- | --- |
| /camera/image_raw | sensor_msgs/Image | gazebo camera |
| /traffic_light/state | aic26_msgs/TrafficLightState | simulation truth |
| /perception/traffic_light | aic26_msgs/TrafficLightState | aic26_perception |
| /perception/result | std_msgs/String (json) | aic26_perception |
| /perception/plate | std_msgs/String | aic26_ocr |
| /perception/detect | std_msgs/String | mission controller |
| /speech | std_msgs/String | mission controller |
