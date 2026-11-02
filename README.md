<p align="center">
  <img src="logo.svg" width="260" alt="AIC26 logo">
</p>

# AIC26 Smart Community Inspection Robot

ROS 1 (Noetic) catkin workspace for the AIC26 Smart Community challenge. A
four-wheel omnidirectional (mecanum) robot (`armbot`) drives through a
simulated community, follows a fixed inspection route, waits at traffic
lights, counts community people, reads car license plates and reports the
results.

The simulation backend runs in Gazebo: URDF/xacro robot, a community
simulation with traffic lights, a saved 2D map, AMCL localization, move_base,
and three int8 ONNX detectors. The `aic26_cobot` package provides the same
topics and frames for the real robot.

## Highlights

- One-command bringup with six modes (`mission`, `demo`, `capture`,
  `waypoints`, `mapping`, `view_map`) and a `backend:=sim|real` switch.
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
`aic26_decision/config/sim_tasks.yaml`.

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

The six modes are:

| mode | What it runs |
| --- | --- |
| `mission` (default) | navigation + perception + `mission_controller` |
| `demo` | navigation + `demo_cycle` (faked results, no models) |
| `capture` | navigation + `capture_cycle` (dataset screenshots) |
| `waypoints` | navigation + `waypoint_follower` |
| `mapping` | gmapping + teleop + rviz (no navigation stack) |
| `view_map` | backend + `map_server` + rviz, to inspect a recorded map (no localization or navigation stack); the robot is placed at the configured start pose through `map -> odom` |

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
roslaunch aic26_bringup bringup.launch mode:=view_map backend:=real
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

`view_map` places the robot at the same start pose without AMCL:
`src/aic26_bringup/launch/view_map.launch` publishes a static `map -> odom`
transform. On the hardware backend the transform is the identity, because the
chassis odometry already starts at `start_x/y/yaw`; in Gazebo the planar-move
plugin integrates from `(0, 0, 0)` at the spawn point, so `map -> odom` carries
`start_x/y/yaw` (the spawn pose, passed down to `sim.launch`). RViz therefore
draws `/map`, `/scan` and the robot model in the same frame, and the start pose
can be checked against the recorded map.

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `mode` | `mission` | one of the six modes above (`real` aliases `mission`) |
| `backend` | `sim` | `sim` for Gazebo, `real` for `aic26_cobot` |
| `sim` | `sim` | simulation launch file under `aic26_gazebo/launch` |
| `day_night` | `day` | simulation sky: `day` or `night` |
| `sky_time` | empty | simulation time of day in hours, overrides `day_night` |
| `map_file` | `maps/world_sim.yaml` (sim) / `maps/world_real.yaml` (real) | map for AMCL / map_server (not used in mapping mode, loaded by view_map); selected through `map_name`, see [Maps](#maps) |
| `use_rviz` | `true` | start RViz with the navigation display (also honored in mapping) |
| `show_view` | `true` | start the simulation camera viewer |
| `nav_delay` | `15.0` | seconds before navigation (or gmapping) starts |
| `task_delay` | `20.0` | seconds before the mission / task nodes start (unused in mapping mode) |
| `set_initial_pose` | `true` | initialize AMCL at `initial_pose_*` |
| `initial_pose_x` | `1.6` | initial map x |
| `initial_pose_y` | `1.7` | initial map y |
| `initial_pose_a` | `3.141592` | initial map yaw in radians |
| `initial_cov` | `0.0004` real / `0.01` sim | standard deviation of the AMCL start pose; the simulation needs 10 cm because its odometry is dead reckoning |
| `virtual_walls_file` | `aic26_navigation/config/virtual_walls.yaml` | road boundary lines (the lidar cannot see them) used as lethal costmap obstacles by both costmaps |
| `disable_strafe` | `false` | temporary non-holonomic mode: TEB gets `max_vel_y` / `acc_lim_y` = 0 and `weight_kinematics_nh` = 1000, the chassis lateral command scale is forced to 0, and the decision nodes switch to the `*_nostrafe` waypoint / task / route files |

The delays are applied as a per-node start-up wait
(`launch-prefix="bash -c 'sleep <delay>; $0 $@'"`); set either to `0` to
disable the wait. This lets the backend come up before navigation, and
navigation before the mission nodes. The defaults are deliberately generous
for slower machines; on a fast machine use e.g. `nav_delay:=5 task_delay:=10`.

`sim`, `day_night`, `sky_time` and `show_view` only apply to
`backend:=sim`. `show_view:=false` is fully headless: it now also keeps the
Gazebo GUI client from starting (`sim.launch` passes `show_view` as the
`gui` argument of `empty_world.launch` and as the `~gui` parameter of the
day/night node, which otherwise opened a window and fell back to a single sky
update). A gzclient renders through llvmpipe on this box, which costs about a
gigabyte and has crashed the simulation with a segfault in `swrast_dri.so`, so
headless runs are also the stable ones. `backend:=real` adds these arguments:

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
| `aic26_description` | xacro/URDF robot models: Gazebo armbot and real cobot (base, four wheels, laser, camera) |
| `aic26_gazebo` | community simulation assets, models, traffic light controller, day/night and C++ visual plugins |
| `aic26_msgs` | `TrafficLightCommand` and `TrafficLightState` messages |
| `aic26_slam` | gmapping launch and the saved 2D map |
| `aic26_navigation` | map_server, AMCL, move_base (global_planner + TEB), `virtual_walls` publisher and the road-boundary geometry |
| `aic26_decision` | mission controller, waypoint follower, demo and capture cycles |
| `aic26_perception` | traffic light, people and car/plate ONNX detectors |
| `aic26_utils` | shared utilities (camera viewer, stop tool) |

## Robot and simulation

- `armbot` is a four-wheel omnidirectional robot. It is spawned at
  `(1.6, 1.7, yaw=pi)` and publishes `/odom` and `/tf`. The Gazebo model uses
  the planar-move plugin, which executes the full `/cmd_vel` twist including
  the lateral component, and the TEB configuration is holonomic
  (`max_vel_y`, `acc_lim_y`, `weight_kinematics_nh: 0`).
- Sensors: a 1280x720 camera at 30 Hz and a 720-beam laser with a 270 degree
  field of view and 12 m range.
- The costmaps are tuned for a responsive display and for what the CPU can
  sustain: the global costmap updates at 5 Hz and publishes at 10 Hz, the local
  one at 10/20 Hz. `publish_frequency` must stay above `update_frequency` (see
  Troubleshooting). The local update costs 0.10-0.20 s on the 4-core test
  machine (720 beam raytracing plus inflation), so 15 Hz was never reached and
  the control loop stalled; 10 Hz is what it can hold. The footprint is per
  backend and comes from the mode launch files: 0.284 x 0.272 m for the Gazebo
  armbot, 0.315 x 0.285 m for the real chassis (base plate 31.5 x 17.5 cm,
  wheels reaching y = +-14.25 cm).
- The global costmap is map + virtual walls + inflation (`static_layer`,
  `virtual_walls`, `inflation_layer`). Its obstacle layer was removed after it
  was measured closing the narrow corridor: laser marks of the moving actors,
  plus a few centimetres of `map -> odom` error, persist in that map-fixed grid
  (raytrace clearing only reaches cells the robot re-observes), and half a
  dozen stale marks plus the 0.136 m inscribed ring turned the whole 0.62 m
  passage lethal, so the global planner reported "Failed to get a plan" and the
  goal aborted. Dynamic obstacles are the local costmap's job. `rosrun ...
  rostopic echo` a corridor cell to see it: `/map` is free there while
  `/move_base/global_costmap/costmap` is 99/100 (the costmap topic publishes
  costs scaled to 0..100, 99 is lethal).
- Both costmaps consume the road boundary through a virtual-wall layer. The
  track is bounded by lines painted on the floor; the lidar sits 16 cm above
  the ground and never returns them, and `ObstacleLayer::raytraceFreespace()`
  writes FREE_SPACE straight into the master grid, so a scan-only costmap lets
  the local planner cut the corner and drive across a line (on the real track
  that also means hitting whatever stands outside the road).
  `scripts/virtual_walls.py` reads the line segments from
  `config/virtual_walls.yaml` (the geometry mirrors the simulation's invisible
  walls plus the arena boundary), rasterises them at 0.05 m (863 cells,
  values 0/100), publishes `/virtual_walls` in the map frame and the same lines
  as a `LINE_LIST` on `/virtual_walls_markers` for RViz. Both costmaps add the
  grid through a `StaticLayer` named `virtual_walls`, listed *after* the
  obstacle layer with `use_maximum: true`, so the combined grid is
  `max(recorded line, live scan)`: the scan can add obstacles but can never
  erase a line, and the lines are part of the costmap itself, so they show up
  in the global and local costmap displays in RViz.
  The cells are published at 100 with `lethal_cost_threshold: 50`, i.e. they
  become LETHAL_OBSTACLE (254). That is required twice over: TEB only turns
  cells with cost 254 into obstacles
  (`updateObstacleContainerWithCostmap`), and the footprint collision check
  uses 253. The previous implementation republished the recorded map at
  60/100 hoping for a "soft" wall; `StaticLayer::interpretValue()` maps
  everything below the lethal threshold to FREE_SPACE, so those walls were
  never in the local costmap at all.
- The local costmap is `obstacle_layer` + `virtual_walls` + `inflation_layer`.
  The recorded map is no longer copied into it: the virtual walls carry every
  line of the road, the lidar carries the boards and actors, and a copied
  gmapping map only added stale cells on top.
- TEB settings that go with the lethal lines: `min_obstacle_dist: 0.10` is the
  smallest value that still keeps the whole body (0.284 m wide) off a 0.05 m
  line while letting the robot through the traffic-light gantry - two 2.5 cm
  posts 0.59 m apart inside a 0.66 m corridor, i.e. 0.153 m per side. At 0.14
  the passage needs 0.564 m of the 0.565 m it has, TEB stalls in front of the
  gantry and every goal behind it (`station_1`) times out. `inflation_dist:
  0.20`, `weight_obstacle: 25` and `weight_inflation: 1.0` keep the footprint
  off the lethal cells; 40 was tried and is too much: at the `corner_1` goal,
  which sits at the inside corner of the ring gap, it made the goal infeasible
  (69 "not feasible" resets and one aborted goal in a single cycle) because the
  penalty then dominates the path term. The price of 25 is a rear corner that
  comes within 6 mm of a line tip at that corner (ground truth, never across
  the line);
  `feasibility_check_no_poses: 0` stops a transiently infeasible pose from
  locking the optimizer out; `oscillation_recovery: false` disables TEB's own
  oscillation guard, which is independent of move_base's `oscillation_timeout`
  and used to fire inside the narrow car chamber. Note that `0` is not "off":
  TEB checks the *current* pose only, so a robot that is already inside the
  margin (a centimetre of localisation error is enough) deadlocks - that is
  what the back-off retry in `mission_common.go_to()` is for.
- The venue map `aic26_slam/maps/world.yaml` is 224 x 224 cells at 0.05 m per
  cell (11.2 m x 11.2 m), with origin `(-6, -6)`; AMCL starts at the spawn
  pose. Its walls were thinned to one cell with
  `src/aic26_slam/scripts/thin_map_walls.py` (gmapping had drawn them 2-3 cells,
  10-15 cm, thick, against 2 cm boards, which left 0.50 m instead of 0.60 m in
  the narrowest corridor). The tool erodes one cell and keeps a one-cell
  skeleton so 2-cell walls are not opened, refuses to write if the free space
  reaches previously unknown cells, and is idempotent (1167 -> 608 occupied
  cells). It contains the road boundary lines as walls, i.e. it is the map of
  the *venue*, whose invisible lane walls are collidable there.
- The simulation runs without those lane walls (the `unvisible_walls` include
  is commented out in `community.world`, the lines are paint), so the venue
  map holds 608 walls where the lidar can see none of them. AMCL then locks
  onto the phantom walls and settles with a constant `map -> odom` offset:
  measured 0.106 m in x, 0.022 m in y, i.e. a third of a 0.66 m lane. The
  robot believed it kept 0.14 m to a line while its rear corner was 10 cm
  *past* it, and TEB (which sees the line at exactly `min_obstacle_dist`)
  declared every trajectory infeasible - 537 planner resets and an abort chain
  in one run. `backend:=sim` therefore uses
  `aic26_slam/maps/world_sim.yaml`, built from the simulated world itself with
  `src/aic26_slam/scripts/build_map_from_sim.py`: it teleports the robot over
  a 0.45 m grid of free poses (43 of them, avoiding every visible model),
  reads one laser scan per pose, and accumulates endpoints (occupied) and rays
  (free) into the same 224 x 224 grid. Ground truth poses come from
  `/gazebo/model_states`, so there is no odometry drift and no scan matcher to
  tune. The result holds only what a beam can hit - arena boards, the people
  and car boards - and AMCL's `map -> odom` error drops to 0.01 m. Both maps
  share resolution, origin and frame, so `map_file` can be swapped freely.
  Two traps in the tool: a PGM stores row 0 at the *top* (map_server flips it
  back on load, so writing the grid bottom-up mirrored the map about
  y = -0.425 and walled off whole corridors), and cells need two hits
  (`occ >= 2`) or the 2.5 cm gantry posts end up in the map and their 0.30 m
  inflation rings meet in the middle of the corridor.
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
than the USB arrival time (the adapter delivers frames in bursts).
`use_imu_yaw:=true` additionally blends the bias-corrected WIT gyro z rate into
the odometry yaw; the method's magnetic heading is not used. This keeps the
odometry TF smooth, which in turn stops GMAPPING/AMCL from publishing
high-frequency pose corrections that look like a shaking robot in RViz.

AMCL is tuned for the N100 in `aic26_navigation/config/amcl_params.yaml`:
300-1200 particles, 45 laser beams, `kld_err: 0.08` and a 2 Hz particle-cloud
publish rate. In a forced-update benchmark this reduced AMCL CPU by about
25-30% while the map pose stayed within a few millimetres of the start pose.
The laser model, resampling interval and `update_min_*` thresholds are
unchanged.

### Maps

Three maps ship with the repository; the mode launch files pick one through
`map_name` (`world_real.yaml` for `backend:=real`, `world_sim.yaml` for
`backend:=sim`) and pass it on as `map_file`:

| file | content | used by |
| --- | --- | --- |
| `aic26_slam/maps/world_real.yaml` | the venue as the real lidar sees it: 774 occupied cells, 4.85 x 5.25 m, 89% of them in straight runs | `backend:=real` |
| `aic26_slam/maps/world_sim.yaml` | what the simulated lidar can hit: arena boards, people and car boards, 503 occupied cells | `backend:=sim` |
| `aic26_slam/maps/world.yaml` | the venue record with the painted road lines as walls, 608 occupied cells | reference for the line geometry |

The road lines are painted on the floor and invisible to the lidar. They are in
`world.yaml` for the costmaps only; AMCL must use a map built by `mapping` mode
on the field, otherwise it has almost no feature to match and the pose follows
the wheel odometry.

Both maps are 224 x 224 cells at 0.05 m with origin `(-6, -6)`, so everything
that consumes a map (AMCL, both costmaps, the virtual walls) works with either.
The simulation must not use the venue map: it contains lane walls that do not
exist in the simulation world, AMCL matches the beams against them and settles
with a 0.11 m offset, which is a third of a lane and enough to drive the robot
across a painted line while the costmap believes there is clearance. Details
and the resulting failure mode are in
[Robot and simulation](#robot-and-simulation).

Rebuild the simulation map after changing `community.world`:

```bash
# terminal 1: world + robot, no navigation
roslaunch aic26_bringup backend.launch backend:=sim show_view:=false
# terminal 2: teleport, scan, write  ~1 min
python3 src/aic26_slam/scripts/build_map_from_sim.py --write
```

The tool prints the cell statistics; a healthy map is ~1% occupied, ~13% free
and the rest unknown outside the arena. Check it by looking at the shared
boundary: `world_sim` is centred on the same origin, so the arena boards must
sit at x = +-2.05 and y = +-2.05..2.10 exactly as in `world.yaml`.

### AMCL must use the omni motion model

The chassis is omnidirectional (`vx, vy, wz`), so `odom_model_type` has to be
`omni`. With the AMCL default (`diff`) a sideways move is decomposed into
`rot1 = +90 deg`, `trans`, `rot2 = -90 deg`, and every filter update injects
`alpha1 * pi/2 = 0.31 rad` of rotation noise into the particle cloud. Measured
against the Gazebo ground truth on a route that strafes 0.5 m and then runs
3.4 m along the corridor, with odometry that stayed within 3 mm of truth:

| motion model | mean error | max error | samples > 0.15 m |
| --- | --- | --- | --- |
| `diff` (AMCL default) | 1.148 m | 2.040 m | 88% |
| `omni` | 0.016 m | 0.023 m | 0% |

This is the drift that showed up in RViz as the robot sliding away from the
global costmap. It is not a costmap problem: the global costmap is map fixed
and static (`static_layer` + `inflation_layer` only), so the walls stay put and
only the pose moves. End to end, the corridor goal (`corner_4`) dropped from
83-123 s to 21 s. Do not tune the alphas to work around a lateral-motion drift;
check `odom_model_type` first.

### Simulation navigation stability (measured)

With `odom_model_type: omni`, a 10 Hz local costmap, `max_vel_y: 0.15`,
`min_obstacle_dist: 0.10`, `xy_goal_tolerance: 0.06`, the lethal virtual walls
and the simulation map, the whole route from `sim_waypoints.yaml` (traffic
light, three stations, traffic light, car_1..3, corner_1..4, start - 13 goals)
completes back to back: **26/26 goals in two consecutive cycles** (166 s and
167 s, ~13 s per leg) and 39/39 over three cycles, with `Aborting because` = 0,
TEB "not feasible" = 1, planner "Failed to get a plan" = 1 and no back-off
retry in the whole run. AMCL `map -> odom` error against the Gazebo ground
truth is 0.005-0.011 m (0.106 m with the venue map, see above). A
ground-truth monitor (footprint corner distance to every line, 10 Hz) reports
**no line crossing**: 0 samples below 0 in ~570k samples over four runs;
the minimum is a corner that touches a line's centreline once at the inside
corner of the ring gap (0.000 m, i.e. the corner can overlap the 0.05 m paint
by up to 2.5 cm for a fraction of a second), and 539 of 175k samples in the
2-6 mm band, all at that same corner. Tightening `weight_obstacle` to 40 to
remove it breaks `corner_1` instead.

What each knob is for:

| setting | why |
| --- | --- |
| `odom_model_type: omni` | the chassis strafes; `diff` scatters the particles |
| local costmap 10 Hz (not 15) | 15 Hz is not reachable on 4 cores, the loop stalls |
| `raytrace_range: 3.5` | the 720 beam raytracing is the expensive half of the update |
| `max_vel_y: 0.15` | caps the sideways share, less wheel slip, easier centring |
| `min_obstacle_dist: 0.10` | footprint clearance: keeps the body off the 0.05 m lines and still fits the 0.59 m gantry passage (0.153 m per side); at 0.14 TEB stalls in front of the gantry |
| `inflation_dist: 0.20`, `weight_obstacle: 25`, `weight_inflation: 1.0` | keep the footprint off the lethal cells and centre the robot in the lane; 40 made `corner_1` infeasible (69 resets, 1 abort) |
| `xy_goal_tolerance: 0.06` | 0.10 let the robot stop 9 cm short, inside the wall inflation, and the final rotation became infeasible |
| `oscillation_timeout: 0` | move_base's oscillation recovery cleared the costmaps every few seconds while the robot crept through the car chamber |
| `oscillation_recovery: false` | TEB's own guard (independent of the above) switched the optimizer to "prefer current turning direction" in the chamber |
| `feasibility_check_no_poses: 0` | TEB checks only the current pose instead of the first three; a transiently infeasible pose no longer discards every trajectory |
| `virtual_walls_file` | lethal road lines in both costmaps; the lidar cannot see a line, and a soft wall (the old 60/100) is mapped to FREE_SPACE by StaticLayer and ignored by TEB |
| `map_name` / `map_file` | both backends use a localisation map that holds only what the lidar can hit; the venue record with the painted lines (`world.yaml`) is not used for AMCL |

### Real robot (measured)

`waypoints.launch backend:=real` with the full 13-goal route, guarded by a
watchdog that cancels the goal, zeroes the velocity and stops the follower when
the planner asks for motion the chassis does not deliver, when `map -> odom`
slides 0.20 m / 0.30 rad, or when the robot leaves the map:

| local costmap | time | TEB "not feasible" | 13/13 | watchdog |
| --- | --- | --- | --- | --- |
| scan only, `feasibility_check_no_poses: 3` | 103 s | 49 | yes | idle |
| + recorded walls (k = 0.6), 3 poses | 109 s | 61 | yes | idle |
| + recorded walls (k = 0.6), 1 pose | 98 s | 13 | yes | idle |

Executed / commanded velocity over the run (0.2 s samples, median): `vx` 0.86,
`vy` 0.53, `wz` 0.66. The steady-state chassis gain measured with a 3-4 s
constant command is 0.96 / 0.92 / 0.83; the lower mission medians are the
0.3-0.5 s ramp of the STM32 seeing a command that TEB changes every 100 ms, not
a scaling error. A pure `vy = 0.12 m/s` command moves the robot 0.091 m at +84
degrees per the laser scan-to-scan match (0.093 m at +90 per the odometry), so
the sideways axis is wired correctly - the fore/aft movement seen during a
strafe is TEB mixing `vx` into the lateral leg, not a swapped axis.

## Mission configuration

The inspection files come in a `sim` and a `real` variant. The `backend`
argument of the decision launch files selects the set, and
`aic26_bringup` forwards its own `backend` value to them:

| backend | waypoints | tasks / route |
| --- | --- | --- |
| `sim` (default) | `sim_waypoints.yaml` | `sim_tasks.yaml`, `sim_screenshot_cycle.yaml` |
| `real` | `real_waypoints.yaml` | `real_tasks.yaml`, `real_screenshot_cycle.yaml` |
| `sim` + `disable_strafe:=true` | `sim_nostrafe_waypoints.yaml` | `sim_nostrafe_tasks.yaml`, `sim_nostrafe_screenshot_cycle.yaml` |
| `real` + `disable_strafe:=true` | `real_nostrafe_waypoints.yaml` | `real_nostrafe_tasks.yaml`, `real_nostrafe_screenshot_cycle.yaml` |

- `disable_strafe:=true` is a temporary test mode: `navigation.launch` gives
  TEB `max_vel_y: 0`, `acc_lim_y: 0` and `weight_kinematics_nh: 1000`, so the
  planner only rotates and drives forward / backward. On the hardware backend
  `chassis.launch` additionally forces `lateral_cmd_scale: 0`, which drops any
  lateral `/cmd_vel` component (teleop included). The decision launches then
  select the `*_nostrafe` set instead of the regular one. The
  `*_nostrafe_waypoints.yaml` files keep the same poses and names but turn
  `car_1`..`car_3` to face +y, because that row is otherwise driven sideways.
  The in-place `turn` at those stops may therefore need retuning in the
  `*_nostrafe_tasks.yaml` / `*_nostrafe_screenshot_cycle.yaml` files.

- `aic26_decision/config/sim_waypoints.yaml` and `real_waypoints.yaml` define
  named poses with `x`, `y`, `yaw` and `dwell` fields. They must use the same
  names, because the task and route files refer to waypoints by name.
- `aic26_decision/config/sim_tasks.yaml` and `real_tasks.yaml` are the ordered
  task lists used by `mission_controller`:
  - `goto`: navigate to a waypoint and continue.
  - `traffic_light`: wait at the stop line until the light is GREEN, then pass.
  - `inspect`: navigate, optionally turn in place, dwell, trigger a detector
    and report the result.
  - `report`: announce a collected result without moving.
- `aic26_decision/config/sim_screenshot_cycle.yaml` and
  `real_screenshot_cycle.yaml` mirror the corresponding mission route for
  `capture_cycle` / `demo_cycle`, stop by stop, so captures use the same
  viewpoints. Each stop has a `label`, `shots`, `shots_per_pass` and a faked
  `result` JSON.
- The `_real` files currently mirror the simulation values. Measure the poses
  and tune dwell / wait times on the real field in those files; no launch or
  code change is needed. `tasks_file`, `waypoints_file` and `route_file` can
  also be overridden explicitly on any decision launch file.
- `mission_controller` publishes announcements on `/speech`
  (`std_msgs/String`). No text-to-speech node is included; connect any TTS
  consumer to that topic.
- On real hardware there is no simulation light truth. `backend:=real` sets
  `light_fallback:=traffic_light` for `mode:=mission`, so the mission uses the
  generic vision state when no state exists for the requested
  `traffic_light_1/2` model.
- `mission_common.RobotDriver.go_to()` retries a failed goal once, after
  reversing `~back_off_distance` metres (default 0.12 m at 0.05 m/s, closed
  loop on tf, time bounded). TEB deadlocks in front of a wall whenever the
  estimated pose is already inside `min_obstacle_dist`: every trajectory is
  "not feasible", and the move_base recovery behaviours only rotate in place
  or clear the costmaps, so the robot has no way back. The reverse step is
  short and slow on purpose - it also has to stay legal with respect to the
  painted lines. `~back_off_speed` tunes it.
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
| `/odom` | `nav_msgs/Odometry` | planar-move plugin / `aic26_cobot` chassis |
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

The viewer writes frames to disk with `s` (one snapshot); `save_fps` adds
that many automatic saves per second (default 0, key only). Files go to
`save_dir` (default `$(find aic26_utils)/../../.datasets/capture`, created on
start) and are named after the `filename` pattern (default
`frame_{index:04d}.jpg`; `{index}` counts and never overwrites, `{stamp}` is
`YYYYmmdd_HHMMSS`, `{label}` is the `label` argument). `label` is also drawn
in the window, together with the key hints and the last saved file. Images
keep the camera resolution, not the window size; `save:=false` disables the
capture completely:

```bash
roslaunch aic26_utils camera_view.launch label:='room A'     # 's' saves
roslaunch aic26_utils camera_view.launch label:='room A' save_fps:=2.0 \
  save_dir:=$HOME/aic26_capture filename:='{label}_{index:04d}.jpg'
roslaunch aic26_utils camera_view.launch save:=false          # view only
```

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

# camera viewer with pixel-sized window; 's' saves a frame into
# .datasets/capture (rosrun: _save:=false _save_fps:=1.0 _save_dir:=...
# _filename:='cam_{index:04d}.jpg' _label:=...)
rosrun aic26_utils camera_view.py

# stop the car and every node that commands it (goal cancel, zero /cmd_vel,
# /chassis/enable false, shutdown of the /cmd_vel + /move_base/goal senders)
rosrun aic26_utils stop_robot.py
rosrun aic26_utils stop_robot.py --no-kill          # keep the nodes alive
rosrun aic26_utils stop_robot.py --keep move_base   # stop only the others
python3 src/aic26_utils/scripts/stop_robot.py --serial   # ROS down: UART zeros

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

# a recorded map only: maps/<map_name>.yaml + rviz, no backend
roslaunch aic26_slam view_map.launch map_name:=world_sim

# localization and navigation on top of the simulation
roslaunch aic26_navigation navigation.launch

# localization and navigation on top of the real hardware backend
roslaunch aic26_navigation navigation.launch \
  laser_min_range:=0.2 laser_max_range:=4.0 \
  initial_pose_x:=1.6 initial_pose_y:=1.7 initial_pose_a:=3.141592

# same stack without lateral motion (TEB max_vel_y / acc_lim_y = 0)
roslaunch aic26_navigation navigation.launch disable_strafe:=true

# mission on top of a running navigation stack
roslaunch aic26_decision mission.launch

# mission with the non-holonomic waypoint set
roslaunch aic26_decision mission.launch disable_strafe:=true

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
- `.datasets/capture/`: frames written by `camera_view` (`save`, `save_fps`,
  `save_dir` and `filename` arguments of
  `aic26_utils/launch/camera_view.launch`; the folder is created on start,
  git-ignored, resumable and never overwritten while `{index}` counts).
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
- The costmaps stutter or trail the robot in RViz: keep `publish_frequency`
  above `update_frequency` in `aic26_navigation/config/*_costmap_params.yaml`
  (now 5/10 Hz global, 10/20 Hz local). costmap_2d tests
  `elapsed > 1/publish_frequency` once per update cycle, so equal values miss
  every second cycle and halve the effective rate. The global costmap sends the
  full `/costmap` grid only for the first message and then publishes
  `/costmap_updates`; RViz subscribes to both.
- The robot drifts away from the global costmap in RViz, in the simulation and on
  hardware: check `odom_model_type` in `amcl_params.yaml` first. The chassis is
  omnidirectional, so it must be `omni`; with the AMCL default `diff` every
  sideways move injects `alpha1 * pi/2` of rotation noise per update and the map
  pose walks away (measured 1.15 m mean / 2.04 m max against 0.016 / 0.023 with
  `omni`, while the odometry itself stayed within 3 mm of the Gazebo truth). After
  that, compare `/amcl_pose` with `/gazebo/model_states` before touching any
  alpha: heavier settings (beam model, more particles) made it worse on a loaded
  4-core machine, and a `map -> base_footprint` lookup that mixes AMCL's
  `transform_tolerance` (0.5 s into the future) with a lagging odometry TF shows
  a phantom error of `speed * 0.5`; stamp-compare instead.
- `use_imu_yaw` uses the WIT gyro z rate, not its magnetic heading, so it can
  be enabled on real hardware to stop encoder yaw drift; AMCL stays the
  absolute reference. Check the sign with `imu_gyro_sign` (bench test: rotate
  the robot left, `/imu/data` `angular_velocity.z` must be positive). The
  simulation has no IMU node, so an IMU cannot explain a drift seen in both
  backends; `odom_model_type` can.
- The robot still shakes in RViz on real hardware: check that the chassis
  `initial_x/y/yaw` match the AMCL `initial_pose_*` (the mode launch files pass
  them through automatically). AMCL starts from `initial_cov` (2 cm on the real
  robot, 10 cm in the simulation, where the odometry is dead reckoning and a
  collision leaves a permanent offset) and `update_min_d/a: 0.10` so corrections
  are not published on every scan. Mapping uses larger GMAPPING
  `linearUpdate/angularUpdate` values for the same reason.
- The mission aborts near the car boards with "Failed to get a plan": the
  global costmap has become lethal in the 0.62 m corridor. That came from the
  laser obstacle layer there (see "Robot and simulation"); check with
  `rostopic echo /move_base/global_costmap/costmap_updates` against `/map`. If
  the layer is ever restored, either limit its marking to the immediate
  surroundings or clear the global costmap before each goal.
- Traffic lights stay dark in simulation: gzclient must finish loading the
  simulation before the visual plugin can apply materials. The controller waits
  `~startup_delay` (6 s in `sim.launch`) before starting.
- Task nodes start too early: increase `nav_delay` / `task_delay`, or check
  that the `map -> base_footprint` transform is available.
- An unknown `mode` or `backend` starts no nodes; use one of the six modes and
  `backend:=sim|real`.

## Scope

This repository contains a simulated competition stack and the hardware
drivers for the real robot. The simulation needs Gazebo; `backend:=real`
needs the actual chassis, IMU, lidar and camera. Robot hardware drivers beyond
these devices, a SLAM back end beyond gmapping, and a text-to-speech engine are
not included; `/speech` is an interface for an external consumer.
