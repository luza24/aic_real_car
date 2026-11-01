# 直接底盘控制：绕过 TEB 的点到点航点控制器

- 日期：2026-11-01
- 工作区：`/home/mowen/aic26_ws_speech`（git worktree，分支 `feature/speech-announcement`）
- 车辆：`mowen@192.168.1.15`（Intel N100，Ubuntu 20.04 + ROS Noetic）
- 状态：代码完成、离线验证通过（假激光 + 假底盘闭环），**尚未实车试跑**

---

## 1. 背景与目标

实车在 0.55–0.61 m 窄道里用 move_base + TEB 时反复出现：

- `TebLocalPlannerROS: trajectory is not feasible`、`NO PATH!`
- 到点附近反复修正/振荡、恢复行为里原地旋转
- 压线问题与停靠点几何强相关（见 `2026-10-28` 压线分析）

因此改为**不用 TEB、不经 move_base**：控制器自己读取 map 帧航点，用 AMCL 位姿闭环，直接向底盘发布 `/cmd_vel`。

本次确定的四个设计选择：

| 项 | 选择 |
|---|---|
| 控制形态 | 点到点闭环：原地转向 → 航向闭环直行 → 接近减速 → 终点 yaw 对准 |
| 位姿反馈 | AMCL map 帧：TF `map→base_footprint` |
| 障碍处理 | 激光前向扇区急停，不做绕障 |
| 落点 | speech 工作区，**独立脚本**，不接入 mission_controller |

---

## 2. 改动清单

全部位于 speech 工作区：

| 文件 | 动作 | 作用 |
|---|---|---|
| `src/aic26_decision/scripts/direct_waypoint_controller.py` | 新增 | 独立 rospy 节点，直接发布 `/cmd_vel`；不 import `mission_common`，不创建 move_base action client |
| `src/aic26_decision/config/direct_controller.yaml` | 新增 | 控制、安全、超时的全部参数默认值 |
| `src/aic26_decision/config/direct_test_waypoints.yaml` | 新增 | 首轮验证用安全子集航点（顶部长廊 3 点） |
| `src/aic26_bringup/launch/direct.launch` | 新增 | 一键启动：backend + map_server/amcl（不含 move_base）+ 控制器 |
| `src/aic26_navigation/launch/navigation.launch` | 修改 | 新增 `use_move_base` 参数（默认 `true`），把 move_base 节点包进 `<group if>`；原有模式行为不变 |
| `src/aic26_decision/CMakeLists.txt` | 修改 | `catkin_install_python` 增加新脚本 |

没有修改 chassis、AMCL、costmap、TEB 的任何参数；没有改任务表和现有航点。

---

## 3. 系统结构

```
chassis_node ──/cmd_vel──> 串口 STM32          (0.5 s 无命令自动零速)
      │
      ├── /odom + TF odom→base_footprint
      └── /vel_raw

lidar_node ──/scan──┐
map_server ─/map────┼──> AMCL ──TF map→odom + /amcl_pose
                    └──> direct_waypoint_controller ──/cmd_vel
robot_state_publisher: base_footprint→base_link→laser
```

关键事实：

- 底盘 `max_linear=0.5 m/s`、`max_angular=1.5 rad/s`、`cmd_timeout=0.5 s`、串口下发 `cmd_rate=10 Hz`、无加速度限制。
- 控制器以 **20 Hz** 运行并**持续发布**；任何循环阻塞超过 0.5 s 都会让底盘先自行停车。
- 多发布者没有 mux：move_base / teleop / mission_controller 任一活跃都会覆盖本节点；控制器会检查并拒绝运行。
- 不使用 move_base 时，`map_server + amcl + chassis + lidar + robot_state_publisher` 仍然必需；costmap、virtual_walls、global planner、TEB 全部不启动。

---

## 4. 状态机

每个航点独立执行：

```
WAIT_READY → [段预检] → ROTATE → DRIVE_SETTLE → DRIVE
           → ALIGN_SETTLE → ALIGN_YAW → DONE_SETTLE → DWELL → 下一点
                         ↑
                BLOCKED（可恢复，恢复后从 ROTATE 重来）
```

| 阶段 | 进入/行为 | 退出条件 | 超时 |
|---|---|---|---|
| `WAIT_READY` | 等 TF、`/scan`、`/map`、后端节点、`/cmd_vel` 独占检查；完成后静置 | 全部就绪 | 60 s |
| 段预检 | 当前位姿 → 航点 的直线走廊用 `/map` 检查 | 通过才继续 | 一次 |
| `ROTATE` | 对准“当前位姿→目标点”的方位角；距离 < 0.25 m 可跳过 | 连续 3 tick 误差 ≤ 0.06 rad | 20 s |
| `DRIVE_SETTLE` | 发布零速，等车停稳 | 0.30 s | — |
| `DRIVE` | 航向闭环直行，近点减速 | 距离 ≤ 0.08 m 连续 2 tick | 120 s；4 s 内位移 < 0.03 m 判卡死 |
| `ALIGN_SETTLE` | 零速停稳 | 0.30 s | — |
| `ALIGN_YAW` | 转到航点 `yaw`（`align_final_yaw:=false` 可跳过） | 连续 3 tick 误差 ≤ 0.06 rad | 15 s |
| `DONE_SETTLE` | 零速，打印实际位置/航向误差 | 0.30 s | — |
| `DWELL` | 可选停留（默认关闭） | 达到 `dwell` | — |
| `BLOCKED` | 障碍/扫描超时/TF 丢失/AMCL 异常：立即零速 | 条件消失（障碍需滞回） | 10 s 后判本点失败 |

失败策略：默认 `continue_on_failure=false`，任一点失败则中止整条路线；可用 `start_at`、`names` 选择子集。

---

## 5. 控制律

所有角度先 `wrap_angle()` 归一化到 `(-π, π]`。

### 5.1 原地转向（ROTATE / ALIGN_YAW）

```
e = wrap(target_yaw - yaw)
|e| <= turn_tol(0.06) 连续 3 tick → 停止，settle 0.3 s
w = sign(e) * clamp(k_turn*|e|, turn_w_min, turn_w_max)
                  = clamp(1.2*|e|, 0.10, 0.35) rad/s
```

符号取**剩余误差**，过冲后自动反向修正；±π 附近保持原方向。

### 5.2 直行（DRIVE）

```
d      = hypot(tx - x, ty - y)
bearing = atan2(ty - y, tx - x)
e      = wrap(bearing - yaw)

|e| > e_reorient(0.60):  v = 0，按 5.1 原地重新对准
否则:
  v = min(v_max, k_d * max(0, d - stop_distance))     # 0.8, 0.05 m, 0.15 m/s
  if d > 0.20: v = max(v, v_min)                      # 0.05 m/s
  v *= max(0.35, 1 - |e|/e_reorient)                  # 航向差越大越慢
  w = clamp(k_theta * e, -w_max, w_max)               # 1.0, ±0.45 rad/s
```

- `linear.y = 0`（差速/skid-steer，不使用横移）。
- 到达判据用 AMCL 位姿，位置容差 0.08 m、终点 yaw 容差 0.06 rad。

### 5.3 输出斜坡

```
dt = 1/control_rate
v_cmd = clamp(v_target, v_cmd_prev - linear_accel*dt,  v_cmd_prev + linear_accel*dt)
w_cmd = clamp(w_target, w_cmd_prev - angular_accel*dt, w_cmd_prev + angular_accel*dt)
```

`linear_accel=0.5 m/s²`、`angular_accel=1.0 rad/s²`。**急停（障碍、超时、Ctrl-C）不走斜坡，直接零速**；恢复前进时从零重新爬升。

---

## 6. 安全层

| 保护 | 规则 |
|---|---|
| 激光前向急停 | `/scan` 前向 ±25° 最小有效距离（`range_min < r <= min(range_max, 6)`）< 0.25 m → 立即零速；恢复到 > 0.35 m 且持续 0.8 s 才继续；阻塞超过 10 s 判本点失败 |
| TF 看门狗 | 每 tick 查 `lookupTransform(map, base_footprint, Time(0))`；连续丢失 0.5 s 零速，2 s 内恢复则从 `ROTATE` 重来，否则失败 |
| 扫描超时 | `/scan` 超过 0.5 s 未更新 → 零速 |
| AMCL 看门狗 | 相邻 `/amcl_pose` 平移 > 0.30 m 或 yaw > 0.35 rad → 停车 2 s；`cov[0]/cov[7] > 0.30` 或 `cov[35] > 0.10` 持续 0.5 s → 停车。位姿超过 3 s 未更新时，**仅当 `/odom` 显示实际运动**才判定异常并锁存，直到新位姿到达（AMCL 静止时不发布，不能按“命令速度”判断，否则会抖动）。控制位姿始终用 TF，AMCL 只做看门狗 |
| 命令独占 | 启动时与每 1 s 用 `rosgraph` 查询 `/cmd_vel` 发布者；除本节点（及 `allowed_cmd_publishers`）外存在任何发布者 → 零速退出 |
| 后端互斥 | `require_backend:=real` 要求存在 `/chassis` 且不存在 `/gazebo`；`sim` 反之；`any` 跳过 |
| 直线段预检 | 每个航点出发前，在 `/map` 上沿直线每 0.05 m 采样，计算到“占据/未知格矩形边缘”的真实距离；小于 `segment_half_width(0.14 m)` 即失败，提示补中间航点 |
| 退出停车 | SIGINT/异常：20 Hz 连发 0.8 s 零速；`disable_chassis_on_exit:=true` 时再调 `/chassis/enable false` |

注意：激光安装高度约 16 cm，低于扫描平面的障碍不可见；车头 0.2 m 内是 `range_min` 盲区。急停只是兜底，不能替代人工值守。

---

## 7. 参数（`direct_controller.yaml`）

### 航点与运行方式

| 参数 | 默认 | 说明 |
|---|---|---|
| `waypoints_file` | 由 launch 指定 | 航点 YAML（`name/x/y/yaw/dwell`，可选 `frame_correction`） |
| `start_at` | `""` | 从该名称开始，按文件顺序继续 |
| `names` | `""` | 逗号分隔名称，按给出顺序执行；优先于 `start_at` |
| `loop` / `max_loops` | `false` / `0` | 循环与次数限制（0 = 无限） |
| `honor_dwell` | `false` | 是否执行航点 `dwell` |
| `align_final_yaw` | `true` | 到点后是否转到航点 yaw |
| `continue_on_failure` | `false` | 单点失败后是否继续 |
| `dry_run` / `dry_run_seconds` | `false` / `10` | 只发零速，打印意图速度 |
| `check_only` | `false` | 只做航点/直线段地图预检后退出 |

### 控制

| 参数 | 默认 | 参数 | 默认 |
|---|---:|---|---:|
| `control_rate` | 20 Hz | `status_rate` | 2 Hz |
| `v_max` | 0.15 m/s | `v_min` | 0.05 m/s |
| `w_max` | 0.45 rad/s | `turn_w_max` / `turn_w_min` | 0.35 / 0.10 rad/s |
| `k_d` | 0.8 /s | `k_theta` / `k_turn` | 1.0 / 1.2 /s |
| `stop_distance` | 0.05 m | `position_tolerance` | 0.08 m |
| `e_reorient` | 0.60 rad | `turn_tol` / `yaw_tol` | 0.06 rad |
| `settle_time` | 0.30 s | `linear_accel` / `angular_accel` | 0.5 m/s² / 1.0 rad/s² |
| `rotate_skip_distance` | 0.25 m | | |

### 安全与超时

| 参数 | 默认 | 参数 | 默认 |
|---|---:|---|---:|
| `obstacle_half_angle_deg` | 25° | `obstacle_stop_distance` | 0.25 m |
| `obstacle_resume_distance` | 0.35 m | `obstacle_clear_hold` | 0.8 s |
| `obstacle_wait` | 10 s | `scan_timeout` | 0.5 s |
| `tf_fail_timeout` | 0.5 s | `tf_recovery_wait` | 2 s |
| `use_amcl_watchdog` | true | `amcl_pose_timeout` | 3 s |
| `odom_topic` | `/odom` | `odom_timeout` | 0.5 s |
| `odom_moving_linear` / `_angular` | 0.02 m/s / 0.05 rad/s | `amcl_jump_xy` / `amcl_jump_yaw` | 0.30 m / 0.35 rad |
| `amcl_cov_xy_max` / `_yaw_max` | 0.30 / 0.10 | `amcl_recovery_wait` | 2 s |
| `exclusive_cmd` | true | `cmd_check_period` | 1 s |
| `segment_check` | true | `segment_half_width` | 0.14 m |
| `map_occupied_threshold` | 65 | `map_unknown_blocked` | true |
| `zero_burst_sec` / `zero_burst_rate` | 0.8 s / 20 Hz | `disable_chassis_on_exit` | false |
| `ready_timeout` | 60 s | `startup_settle` | 2 s |
| `turn_timeout` / `drive_timeout` / `align_timeout` | 20 / 120 / 15 s | `stall_window` / `stall_distance` | 4 s / 0.03 m |

---

## 8. 启动方式

```bash
# 0) 只做预检：检查航点直线是否穿墙，不驱动
roslaunch aic26_bringup direct.launch backend:=real check_only:=true

# 1) 不动的联调：检查 TF/scan/map/参数/命令独占
roslaunch aic26_bringup direct.launch backend:=real dry_run:=true

# 2) 单点低速实测
roslaunch aic26_bringup direct.launch backend:=real use_rviz:=true \
  names:="traffic_light_1" v_max:=0.10

# 3) 连续多段（默认用 direct_test_waypoints.yaml）
roslaunch aic26_bringup direct.launch backend:=real \
  waypoints_file:=$(rospack find aic26_decision)/config/direct_test_waypoints.yaml

# 4) 指定任务表中的若干点（按给定顺序）
roslaunch aic26_bringup direct.launch backend:=real \
  waypoints_file:=$(rospack find aic26_decision)/config/real_waypoints.yaml \
  names:="traffic_light_1,station_1"
```

运行前提：

- **move_base、teleop、mission_controller 必须未运行**（节点会自检并退出）。
- AMCL 初始位姿与实际摆放一致；`direct.launch` 默认 `(1.6, 1.7, π)`，车端 odom 初值与此保持一致。
- 实车使用地图 `world.yaml`（speech 工作区默认）；如换成 `world_real.yaml`，用 `map_file:=...` 覆盖。

---

## 9. 验证记录

全部为**离线验证**：独立 ROS master（端口 11312–11314）+ map_server + 假激光 + 假底盘（积分 `/cmd_vel` 广播 TF），**未启动 chassis 节点、未接串口、未产生真实运动**。

| 测试 | 结果 |
|---|---|
| `check_only`（起点 → traffic_light_1 → station_1 → corner_2） | 3 段全部 PASS |
| `dry_run` | `heading_error=0.10 rad → v=0.13 m/s, w=0.10 rad/s`，无非零 `/cmd_vel` |
| 前方 0.23 m 障碍 | 进入 `BLOCKED`，`v=w=0` |
| 闭环三航点（`traffic_light_1, station_1, corner_2`） | 全部到达：位置误差 0.079 / 0.080 / 0.079 m；yaw 误差 0.056 / 0.018 / 0.030 rad |

测试脚本位于车端 `/tmp/direct_test/`（`fake_scan.py`、`fake_pose.py`、`run_offline_test.sh`、`run_closed_loop_test.sh`），未放入仓库。

---

## 10. 已知限制与实车前清单

1. **现有 `real_waypoints.yaml` 是给 TEB 用的**：相邻点的直线可能穿墙。必须先用 `check_only` 验证；被挡时补中间航点，不要加宽 `segment_half_width`。
2. 首跑用 `v_max:=0.10`，人跟车，急停在手；确认 Ctrl-C 后 0.8 s 内停住。
3. 激光看不到低于约 16 cm 的障碍；车头 0.2 m 内是盲区；只有前向扇区保护，无侧向保护。
4. speech 工作区的 AMCL 参数较旧（300/1200 粒子、0.10/0.10 更新），位姿会有阶梯修正；如表现差再考虑从 main 借用参数。
5. 直行段不做绕障：遇到障碍只会停车，超过 10 s 判失败。
6. 终点 yaw 若朝向墙，原地旋转可能扫掠到障碍；航点朝向需逐点检查。
7. 建议指标：位置误差 ≤ 0.10 m、yaw ≤ 0.10 rad、无非计划停车、3 次连续全程 0 失败。

---

## 11. 与 TEB 的取舍

| 维度 | 直接控制器 | TEB + move_base |
|---|---|---|
| 可预测性 | 高：固定转向/直行，无轨迹优化 | 低：可能 `not feasible`、振荡、恢复旋转 |
| 窄道行为 | 按直线走，路线由航点保证 | 依赖 costmap/膨胀，容易被判不可行 |
| 障碍绕行 | 无，只急停 | 有（全局 + 局部规划） |
| 路线要求 | 相邻航点必须直线可达或补中间点 | 可绕行，容错更高 |
| CPU | 低（无 costmap/TEB） | 高（两层 costmap + TEB 10 Hz） |
| 定位依赖 | 强：AMCL 跳变会直接反映到运动 | 同样依赖，但有代价地图约束 |

---

## 12. 开发过程中的坑

- **ROS1 `tf` 的 `lookupTransform` 不接受 timeout 参数**（那是 tf2 的 API）；调用四个参数会抛 `TypeError`。本实现只用 `Time(0)`。
- 直线段检查最初用“格数半径”，在 0.05 m 栅格 + 浮点边界下会多算一格，导致贴墙航点误报。改为“采样点到占据格矩形边缘的真实欧氏距离”后稳定；走廊半宽取 0.14 m（机器人半宽 0.136 m）。
- `rosrun ... _check_only:=true` 的私有参数会留在参数服务器上，下一次运行若未显式覆盖会沿用；`direct.launch` 对每个参数都显式赋值，不受影响。
- 该工作区 `build/CMakeCache.txt` 里有遗留 `CATKIN_WHITELIST_PACKAGES=aic26_navigation`，`catkin_make` 不会重编 `aic26_decision`；新脚本仍可被 `rosrun`/`roslib` 从源码目录解析到。需要安装到 devel 时先清掉该变量。
