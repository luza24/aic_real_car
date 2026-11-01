# aic26_ws_speech 导航 / 运动控制「到位反复修正 / 偏移」调参记录

- 日期：2026-10-28
- 工作区：`/home/mowen/aic26_ws_speech`（git worktree，分支 `feature/speech-announcement`）
- 车辆：`mowen@192.168.1.15`（主机名 `mowen-JNE10`）
- 运行栈：ROS Noetic / move_base / amcl / TEB local planner 0.9.1
- 核对方式：分析时导航栈正在运行，所有“当前值”都用 `rosparam get` 与 YAML 双向核对
- 约束：**尽量不改航点**；每次只改一小步；每步都要有原因和验证标准

---

## 1. 结论摘要

1. **到位判定在 TEB，不在 move_base**
   - `TebLocalPlannerROS/xy_goal_tolerance = 0.10 m`
   - `TebLocalPlannerROS/yaw_goal_tolerance = 0.12 rad ≈ 6.9°`
   - `free_goal_vel = false`（终点规划零速度，正常）
   - move_base 层没有设置 `/move_base/xy_goal_tolerance`
   - 任务层原地转头容差 `turn_tolerance = 0.10 rad ≈ 5.7°`

2. **“反复修正 / 固定偏移”的主因很可能不是容差，而是航点落在 TEB 的不可行区**
   - TEB 的可行性检查用的是 local costmap 的 footprint + inscribed inflation，不是 `min_obstacle_dist`。
   - footprint 内切半径 0.136 m；障碍物周围 0.136 m 内是 cost 253 栅格，footprint 不能碰 253。
   - 对侧向墙，机器人中心必须离墙 ≥ `0.136 + 0.136 = 0.272 m`（前后墙 0.278 m）。
   - 这比 `min_obstacle_dist = 0.12` 推出来的 0.256 m 更严格。
   - 按 `/map` 0.05 m 栅格中心实测，多个航点到最近障碍只有 0.207–0.226 m，**在 0.272 m 虚拟核心内**；其余航点也只有 4 mm 余量。

3. **日志证据**
   - 最近一次任务（15:08:40–15:11:11，15 个 task）出现 4 次：
     `TebLocalPlannerROS: trajectory is not feasible. Resetting planner...`
   - 时间：15:09:57、15:10:28、15:10:29、15:11:04。
   - `car_1` goal 于 15:10:22.5 发出，15:10:28/29 连续两次 reset，15:10:33.8 才进入下一个 task；这段时间的“到位后反复修正”与日志吻合。

4. **任务层没有“到位后二次位置精修”的代码**
   - `mission_common.go_to()` 只发 move_base goal 并等待结果，成功后直接返回。
   - 之后唯一的动作是 `turn_in_place()` / `turn_back()`：inspect 任务会故意转 90°，检测完再转回航点朝向。
   - 所以：到位后的**平移/倒车**修正来自 move_base/TEB；**转头再转回**是任务设计本身，不是 bug。

---

## 2. 当前参数清单（已核对运行值）

### 2.1 到位 / 终点相关

| 参数 | 当前值 | 文件:行 | 说明 |
|---|---:|---|---|
| `TebLocalPlannerROS/xy_goal_tolerance` | 0.10 m | `aic26_navigation/config/local_planner_params.yaml:13` | 位置容差，直接决定最大停车偏移 |
| `TebLocalPlannerROS/yaw_goal_tolerance` | 0.12 rad | 同上 `:14` | ≈6.9° |
| `TebLocalPlannerROS/free_goal_vel` | `false` | `:15` | 终点零速度；不要改 true |
| `TebLocalPlannerROS/penalty_epsilon` | 0.05 | TEB 默认（dynparam 可见） | 约束松弛，二阶影响 |
| `GlobalPlanner/default_tolerance` | 0.1 m | `aic26_navigation/config/global_planner_params.yaml:3` | 目标在障碍里时允许投影 10 cm；当前可达目标实测 `end_dx=0` |
| costmap resolution | 0.05 m | global/local costmap | 栅格量化 5 cm |
| AMCL `update_min_d` / `update_min_a` | 0.10 m / 0.10 rad | `aic26_navigation/config/amcl_params.yaml:16-17` | 位姿更新粒度 ≈10 cm / 5.7° |
| AMCL `transform_tolerance` | 0.5 s | `:19` | TF 容忍 |
| AMCL `min_particles` / `max_particles` | 300 / 1200 | `:9-10` | N100 资源优化 |
| AMCL `initial_cov_*` | 0.03 | `:42-44` | ≈17 cm / 9.9° 初始标准差 |
| `turn_tolerance` | 0.10 rad | `aic26_decision/launch/mission.launch:60` | 原地转头容差 ≈5.7° |
| `turn_speed` | 0.8 rad/s | `:59` | 转头速度 |
| `turn_min_speed` | 0.15 rad/s | `:61` | 末端最小转速 |
| `turn_settle` | 0.5 s | `:58` | 停转后稳定时间 |
| `turn_rate` | 20 Hz | `mission_common.py:196` 默认 | 转头控制频率 |
| `turn_timeout` | 20 s | `mission.launch:62` | 单次转头超时 |
| `nav_timeout` | 120 s | `mission.launch:50` | 单 goal 上限 |

> TEB 0.9.1 没有 `trans_stopped_vel` / `theta_stopped_vel`，不要照网上教程加这两个参数。

### 2.2 反复修正 / 恢复相关

| 参数 | 当前值 | 文件:行 | 影响 |
|---|---:|---|---|
| `min_obstacle_dist` | 0.12 m | `local_planner_params.yaml:30` | TEB 优化目标；与 footprint 相加 0.256 m，但 costmap 253 区更严（0.272 m） |
| `inflation_dist` | 0.15 m | `:31` | TEB 膨胀代价带；当前 > `min_obstacle_dist`，合法 |
| `weight_obstacle` | 6.0 | `:32` | 相对 `weight_viapoint=30` 偏低 |
| `weight_inflation` | 0.6 | `:33` | 膨胀代价权重 |
| `include_costmap_obstacles` | true | `:34` | 使用 costmap 障碍点 |
| `footprint` | 0.284 × 0.272 m | `local_planner_params.yaml:11`、`costmap_common_params.yaml:4` | 内切半径 0.136 m |
| `footprint_padding` | 0.0 | `costmap_common_params.yaml:5` | 可负值缩小 costmap footprint |
| `allow_init_with_backwards_motion` | true | `local_planner_params.yaml:44` | 终点附近可能倒车修正 |
| `weight_kinematics_forward_drive` | 4.0 | `:18` | 前进偏好偏弱 |
| `global_plan_viapoint_sep` | 0.3 m | `:25` | viapoint 间距 |
| `weight_viapoint` | 30.0 | `:27` | 强拉回全局路径，终点附近可能左右修 |
| `via_points_ordered` | false | `:26` | 路径自交时可能错配 viapoint |
| `max_number_classes` | 5 | TEB 默认 | 5 个同伦类候选，可能来回切换 |
| `switching_blocking_period` | 0.0 | TEB 默认 | 允许立即切换同伦类 |
| `feasibility_check_no_poses` | 1 | `local_planner_params.yaml:46` | 只检查第一个 pose |
| `global_plan_overwrite_orientation` | **false** | `:43` | 正确：保留目标朝向 |
| `oscillation_recovery` | true | TEB 默认 | TEB 自身振荡恢复 |
| `shrink_horizon_backup` | true | TEB 默认 | 优化失败时缩短时域 |
| move_base `oscillation_timeout` | 6.0 s | `aic26_navigation/launch/navigation.launch:56` | 6 s 内移动不到 `oscillation_distance` 判振荡 |
| move_base `oscillation_distance` | 0.1 m | `:57` | 与 `xy_goal_tolerance` 相同，末端蠕行极易误判 |
| `recovery_behavior_enabled` | true | `:59` | 触发恢复行为 |
| `clearing_rotation_allowed` | true | `:58` | 恢复时原地旋转，看起来像突然修正 |
| `controller_patience` | 15.0 s | `:55` | 控制器失败等待 |
| `planner_patience` | 5.0 s | `:54` | 规划失败等待 |
| `controller_frequency` | 10.0 Hz | `:52` | 与底盘 `cmd_rate` 相同 |
| `planner_frequency` | 1.0 Hz | `:53` | 全局重规划频率 |

### 2.3 底盘执行层

| 参数 | 当前值 | 文件:行 | 说明 |
|---|---:|---|---|
| `publish_rate` | 30.0 Hz | `aic26_cobot/config/chassis.yaml:28` | odom/TF |
| `cmd_rate` | 10.0 Hz | `:29` | 与 move_base 10 Hz 相同，末端低速可能顿挫 |
| `feedback_rate` | 20.0 Hz | `:30` | STM32 反馈 |
| `cmd_timeout` | 0.5 s | `:40` | move_base 停止发布后最多保持 0.5 s 旧命令 |
| `linear_deadband` / `angular_deadband` | 0.002 | `:34-35` | 静止噪声门限 |
| `linear_cmd_scale` / `angular_cmd_scale` | 1.0 / 1.0 | `:54-55` | 命令标定 |
| `linear_odom_scale` / `angular_odom_scale` | 1.0 / 1.0 | `:56-57` | 里程标定 |
| `max_linear` / `max_angular` | 0.5 / 1.5 | `:58-59` | 限幅 |
| `use_imu_yaw` | true | `:65` | 陀螺 yaw 融合 |
| `imu_gyro_gain` | 0.95 | `:68` | 陀螺权重 |

---

## 3. 实测数据

### 3.1 航点到最近静态障碍的距离（`/map`，0.05 m 栅格中心）

> 测量误差约 ±2 cm；结论是“全部航点都在临界线附近或里面”，不要求把每个数值当成精确值。

| 航点 | 最近障碍 | 与 0.272 m 虚拟核心的差 | 判定 |
|---|---:|---:|---|
| `traffic_light_1` | 0.226 | **−0.046** | 不可行 |
| `station_1` | **0.207** | **−0.065** | 最严重 |
| `car_1` | 0.226 | **−0.046** | 不可行 |
| `car_2` | 0.226 | **−0.046** | 不可行 |
| `car_3` | 0.226 | **−0.046** | 不可行 |
| `corner_4` | 0.226 | **−0.046** | 不可行 |
| `station_2` | 0.276 | +0.004 | 临界（4 mm） |
| `station_3` | 0.276 | +0.004 | 临界（4 mm） |
| `traffic_light_2` | 0.276 | +0.004 | 临界（4 mm） |
| `start` | 0.276 | +0.004 | 临界（4 mm） |
| `corner_5` | 0.306 | +0.034 | 可行 |
| `corner_2` / `corner_3` | 0.326 | +0.054 | 可行 |
| `corner_1` | 0.389 | +0.117 | 可行 |

走廊宽度实测（沿墙法线方向自由距离）：

| 区域 | 走廊宽度 | 0.272 m 虚拟核心下的横向可行宽度 |
|---|---:|---:|
| 顶部走廊（light1 / station1） | ≈0.61 m | ≈6.6 cm |
| `car_2` 走廊 | ≈0.61 m | ≈6.6 cm |
| `car_3` 走廊 | ≈0.56 m | **≈1.6 cm** |
| `station_3` / `traffic_light_2` 走廊 | ≈0.56 m | **≈1.6 cm** |

0.56 m 走廊里，0.272 m 虚拟核心只给 ±0.8 cm 横向余量，任何定位/地图误差都会让目标不可达。

### 3.2 现场动态障碍（激光，分析时）

| 航点 | 激光最近点 | 说明 |
|---|---:|---|
| `station_2` | 0.154 m | 现场人/道具很近 |
| `traffic_light_2` | 0.182 m | 同上 |
| `car_1` / `corner_4` | 0.241 m | 同上 |
| `corner_5` | 0.236 m | 同上 |

现场有人/道具时，可行域比静态地图更小。调参测试要在场地清空或至少目标附近无障碍时进行，否则测不出参数效果。

### 3.3 日志与规划检查

- 4 次 `trajectory is not feasible. Resetting planner...`：15:09:57、15:10:28、15:10:29、15:11:04。
- 当前现场调用 `/move_base/make_plan`：
  - `traffic_light_1` 从多个起点返回空 plan；
  - `traffic_light_2` 从右侧起点返回空 plan，从上下走廊返回正常；
  - 其他航点正常，可达目标 `end_dx/dy/dyaw = 0`。
  - 空 plan 可能与现场动态障碍有关；场地清空后复测。

---

## 4. 先判断症状类型（决定改哪一组参数）

在动参数之前，先观察一次失败：

| 现象 | 最可能原因 | 对应阶段 |
|---|---|---|
| 每次都停在离航点约 3–6 cm、方向一致地偏向走廊中心/远离最近墙 | 虚拟核心 0.272 m，目标不可行 | 阶段 1–6 |
| 停在 8–10 cm 内就算成功，方向随机 | `xy_goal_tolerance=0.10` + 定位噪声 | 阶段 7 |
| 到位后突然原地转一圈/清除旋转 | 振荡误判 + recovery | 阶段 8（先完成 1–6） |
| 终点附近倒车再前进、左右摆 | 倒车初始化 / viapoint 强拉 | 阶段 9 |
| 转头后朝向差，平移不大 | `turn_tolerance` 等任务层参数 | 阶段 10 |
| 所有航点都朝同一个方向偏，且偏得一样多 | AMCL/地图/航点标定，不是 TEB | 先别调 TEB，复核定位 |

> 如果偏移方向总是“远离最近的墙”，基本可以确认是虚拟核心问题。

---

## 5. 逐步调整方案（不改航点，小步走）

原则：

- 每次只改 **1 个**参数（或同一文件里紧耦合的 1 组），测试后再下一步。
- 每次改动后记录：reset 次数、最终 dx/dy/dyaw、是否触发 recovery、完成时间。
- 症状消失就停，不要追求零误差。
- 任何一步让 reset 次数或完成时间变差，立即回退。
- 航点保持不动；如果最后只剩 `station_1` 差 1–2 cm，再单独讨论是否接受或微调 3 cm。

### 阶段 0：基线测量（不改参数）

**目的**：拿到可重复的基线，确认偏移方向是否“远离最近墙”（虚拟核心假设），并作为后续每一步的对照。

#### 0.1 测试前检查（逐项确认）

在车上终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/mowen/aic26_ws_speech/devel/setup.bash

# 1) 导航栈节点齐全
rosnode list | grep -E '^/(amcl|move_base|map_server|chassis)$'

# 2) 没有 mission_controller 在跑（避免它同时发 goal）
rosnode list | grep mission_controller || echo "mission_controller not running"

# 3) /cmd_vel 只有 /move_base 发布者；如果有 teleop 节点，先停掉
rostopic info /cmd_vel

# 4) TF 正常
timeout 3 rosrun tf tf_echo map base_footprint

# 5) 基线 TEB 值（应为 0.10 / 0.12 / 0.12）
rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/yaw_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/min_obstacle_dist
```

同时确认：

- 场地目标附近清空（人、道具、椅子）；
- 车电量正常，急停在手；
- 车上有人值守，行进路线上没有障碍；
- 记录机器人当前位姿：`rosrun tf tf_echo map base_footprint`，后面每次测试要尽量回到同一位置。

#### 0.2 选一个代表航点

建议顺序：

1. `car_1`：上次日志里 15:10:28/29 连续 reset 的点，最容易复现；
2. `car_3`：0.56 m 窄道，几何最紧；
3. `station_1`：间隙最小（0.207 m），但路径最长。

一次只测一个点。从 `real_waypoints.yaml` 自动提取，避免手抄坐标错：

```bash
python3 - <<'PY'
import yaml
src = '/home/mowen/aic26_ws_speech/src/aic26_decision/config/real_waypoints.yaml'
name = 'car_1'          # 改成 car_3 / station_1
wps = yaml.safe_load(open(src, encoding='utf-8'))['waypoints']
wp = [w for w in wps if w['name'] == name][0]
yaml.safe_dump({'waypoints': [wp]}, open('/tmp/one_wp.yaml', 'w'),
               allow_unicode=True)
print(wp)
PY
cat /tmp/one_wp.yaml
```

#### 0.3 记录日志基线行号

`rosout.log` 是跨多次测试累积的，先记下当前行数：

```bash
wc -l < ~/.ros/log/latest/rosout.log > /tmp/baseline_line
cat /tmp/baseline_line
```

测试结束后用这个行号切片，只统计本次新增的日志。

#### 0.4 启动最终误差监控（终端 A）

把下面的脚本存到 `/tmp/goal_error_monitor.py`（完整内容见本文附录 12），然后：

```bash
source /opt/ros/noetic/setup.bash
source /home/mowen/aic26_ws_speech/devel/setup.bash
python3 /tmp/goal_error_monitor.py
```

> 必须在发送 goal **之前**启动；它依赖 `/move_base/current_goal`，启动晚了就拿不到目标坐标。

#### 0.5 启动 reset/恢复日志监听（终端 B）

```bash
tail -n 0 -F ~/.ros/log/latest/rosout.log \
  | grep --line-buffered -E \
    "trajectory is not feasible|Rotate recovery|Clearing costmap|oscillat"
```

如果 `tail` 的 `-F` 不支持，用 `tail -f`。

#### 0.6 发单点 goal（终端 C）

```bash
source /opt/ros/noetic/setup.bash
source /home/mowen/aic26_ws_speech/devel/setup.bash
rosrun aic26_decision waypoint_follower.py _waypoints_file:=/tmp/one_wp.yaml
```

预期日志：

```
waypoint follower started, 1 waypoints
going to car_1
reached car_1
```

- 车会实际移动，保持急停；
- 如果出现异常，优先按急停；键盘 `Ctrl-C` 只是让 waypoint_follower 发零速度，move_base 仍可能继续执行 goal；
- 需要取消所有 goal：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID \
  "{stamp: {secs: 0, nsecs: 0}, id: ''}"
```

#### 0.7 收集本次数据

测试结束后，在终端执行：

```bash
BASE=$(cat /tmp/baseline_line)
echo "=== not feasible 次数 ==="
tail -n +$((BASE+1)) ~/.ros/log/latest/rosout.log \
  | grep -c "trajectory is not feasible"
echo "=== 恢复/振荡事件 ==="
tail -n +$((BASE+1)) ~/.ros/log/latest/rosout.log \
  | grep -E "Rotate recovery|Clearing costmap|oscillat"
echo "=== 时间线 ==="
tail -n +$((BASE+1)) ~/.ros/log/latest/rosout.log \
  | grep -E "going to|reached|failed to reach|trajectory is not feasible|Rotate recovery|Clearing costmap"
```

终端 A 会打印：

```
FINAL ERROR dx=... dy=... dyaw=...
```

记录到测试表：

- `not feasible` 次数；
- `FINAL ERROR` 的 dx / dy / dyaw；
- 是否出现 `Rotate recovery` / `Clearing costmap`；
- `going to` → `reached` 的耗时（用日志时间戳相减，或看秒表）；
- 机器人最终位姿与最近墙的关系（见 0.9）。

#### 0.8 重复 3 次，保证可重复性

同一点至少跑 3 次。每次结束后：

- 用遥控/单独一个 `start` 航点把车开回 `(1.6, 1.7, π)` 附近；
- 用 `rosrun tf tf_echo map base_footprint` 记录本次起始位姿；
- 起始位姿和上次差超过 5 cm / 3°，本次数据单独标注，不直接和上一次比较。

不想跑 3 次也可以，但至少 1 次基线 + 1 次改动后对比；TEB 的行为有一定随机性，3 次更稳。

#### 0.9 怎么判断基线是否支持“虚拟核心”假设

| 测试点 | 最近墙 | 预期偏移方向 | 说明 |
|---|---|---|---|
| `car_1` | 右侧（+x，约 x=1.41） | **dx 为负**（向 −x，远离墙） | 约 −4~−5 cm |
| `car_3` | 右侧（+x） | **dx 为负** | 约 −4~−5 cm |
| `station_1` | 下方（−y，约 y=1.45） | **dy 为正**（向 +y，远离墙） | 约 −5~−6 cm 的短差 → dy 正 |

如果实测方向和上表一致，基本可以确认是 0.272 m 虚拟核心；进入阶段 1。

如果：

- 偏移方向随机、每次不同 → 先查 AMCL/定位；
- 所有航点往同一个方向偏、幅度差不多 → 查地图/航点标定，不是 TEB；
- 没有 reset、最终误差 <2 cm → 该点基线已经很好，换下一个问题点，或直接进入阶段 7 看容差。

**通过标准**：拿到至少 1 次（最好 3 次）可重复的基线：reset 次数、dx/dy/dyaw、恢复事件、耗时，并判断偏移方向。

---

### 阶段 1：local costmap footprint 缩小 2 cm（只改 local）

**改动**：

文件：`aic26_navigation/config/local_costmap_params.yaml`

在 `local_costmap:` 下增加：

```yaml
local_costmap:
  footprint_padding: -0.02
```

（`costmap_common_params.yaml` 里仍是 `footprint_padding: 0.0`，global costmap 不受影响。）

**原因**：

- 这是验证“虚拟核心假设”的最小一步。
- local footprint 半宽从 0.136 缩到约 0.116，虚拟核心从 0.272 → 0.232 m。
- 0.226 m 的航点仍差 0.6 cm，`station_1` 仍差 2.5 cm，**预期只能部分改善**；如果完全没变化，说明 reset 主要来自现场动态障碍，不是静态几何，需要先排查现场。

**验证**：

- `rosparam get /move_base/local_costmap/footprint_padding` 应为 -0.02。
- `/move_base/local_costmap/footprint` 在 RViz 中应略小。
- 同一航点：reset 次数、最终误差、完成时间。
- 如果 `footprint_padding` 动态改没生效，重启导航栈后再测。

**回退**：删除该行或改回 0.0。

---

### 阶段 2：local footprint 再缩 1 cm（-0.02 → -0.03）

**改动**：

```yaml
local_costmap:
  footprint_padding: -0.03
```

**原因**：

- 虚拟核心 0.232 → 0.212 m。
- 0.226 m 的航点变为可行（余量 1.4 cm）；`station_1` 0.207 m 只差 0.5 cm，可能仍会偶发 reset。
- 只改 1 cm，尽量保留 costmap 安全余量。

**验证**：同一航点。若 `station_1` 仍 reset，进入阶段 3；否则直接跳到阶段 4。

**回退**：-0.02。

---

### 阶段 3（按需）：local footprint 再缩 1 cm（-0.03 → -0.04）

**改动**：

```yaml
local_costmap:
  footprint_padding: -0.04
```

**原因**：

- 虚拟核心 0.192 m，按 0.05 m 栅格测量，现有全部航点（0.207–0.226）都进入可行区。
- 只在这一步仍出现 `station_1`/`car_*` 的 `not feasible` 时才做。
- 真实车体边缘到墙仍有约 4–9 cm；`station_1` 最紧约 4.4 cm（按射线测量 0.18 m 时）。
- 如果 -0.04 后 `station_1` 仍偶发 reset，优先考虑把 `station_1.y` 微调 +0.03 m（这是唯一建议的航点微调，且只 3 cm）；不要继续把 padding 降到 -0.05 以下而不评估物理间隙。

**验证**：同一航点；`station_1` 单独测一次。

**回退**：-0.03。

---

### 阶段 4：`min_obstacle_dist` 0.12 → 0.10（TEB）

**改动**：

文件：`aic26_navigation/config/local_planner_params.yaml`

```yaml
min_obstacle_dist: 0.10
```

**原因**：

- 阶段 1–3 解决的是 costmap 的 `253` 碰撞检查；TEB 优化器仍以 `0.136 + min_obstacle_dist` 为目标惩罚终点。
- 0.10 对应 0.236 m：0.226 m 的航点刚好可行（1 cm 余量）；`station_1` 0.207 仍差 2.9 cm，可能继续提前停。
- 先降 0.02，不直接降到 0.05，保留 TEB 的 10 cm 安全目标。
- `inflation_dist` 保持 0.15（必须 > `min_obstacle_dist`，当前合法），不要一起改。

**验证**：`traffic_light_1` / `car_1` / `car_2` / `car_3` 的最终 dx/dy 应比以前小；`station_1` 可能仍偏。

**回退**：0.12。

---

### 阶段 5：`min_obstacle_dist` 0.10 → 0.08（TEB）

**改动**：

```yaml
min_obstacle_dist: 0.08
```

**原因**：

- 目标 0.216 m：0.226 航点有 1 cm 余量；`station_1` 0.207 仍差 9 mm。
- 一次只降 0.02，观察是否出现贴墙/窄道通过性下降。

**验证**：窄道 `car_3`、`station_3` 通过是否变慢或触发 recovery；最终误差。

**回退**：0.10。

---

### 阶段 6：`min_obstacle_dist` 0.08 → 0.06（TEB）

**改动**：

```yaml
min_obstacle_dist: 0.06
```

**原因**：

- 目标 0.196 m：按静态栅格测量，所有航点都可行。
- 如果按射线测量 `station_1` 只有 0.18 m，0.06 仍会差 1.6 cm；此时不要继续降到 0.05 以下硬追，接受 `station_1` 的 1–2 cm 小偏移，或单独把 `station_1.y` +0.03（可选，尊重“尽量不改航点”）。
- 0.06 m 是建议的下限附近；再低会明显削弱 TEB 的物理安全余量。

**验证**：`station_1` 单独测；如果仍有 reset/偏移，评估“接受小偏移”还是“3 cm 航点微调”。

**回退**：0.08。

> **停止点**：如果此时 reset 消失、最终平移误差 ≤2 cm，就停在阶段 6，进入阶段 7；不要为了 0.5 cm 继续压参数。

---

### 阶段 7：收紧到位容差（一次一个）

**前提**：阶段 1–6 已完成，且 reset 明显减少或消失。

#### 7a. `xy_goal_tolerance` 0.10 → 0.08

```yaml
xy_goal_tolerance: 0.08
```

**原因**：目标现在可达，把“成功圆”从 10 cm 收到 8 cm，减少允许的提前停车偏移。这是 2 cm 的小步，先看是否引入新的 reset。

**验证**：同一航点；如果 reset 增加，说明目标仍不够可行，回到阶段 6，不要继续收紧。

#### 7b. `xy_goal_tolerance` 0.08 → 0.06

**原因**：进一步减少允许偏移。0.06 与 5 cm 栅格 + AMCL 10 cm 更新粒度匹配；**不建议低于 0.05**。

**验证**：连续 3 次同一航点，最终误差是否稳定；是否有新的 `not feasible`。

#### 7c. `yaw_goal_tolerance` 0.12 → 0.10 → 0.08

**原因**：如果问题是朝向差（不是平移），单独收紧 yaw。每次 0.02，测一次。0.08 rad ≈ 4.6°。

**注意**：xy 和 yaw 不要在同一轮一起改，否则无法归因。

**回退**：每一步回上一档。

---

### 阶段 8：抑制末端“振荡误判 / 原地转圈”（一次一个）

**前提**：阶段 1–6 已完成；如果现在没有原地转圈，跳过本阶段。

#### 8a. `oscillation_timeout` 6.0 → 8.0

文件：`aic26_navigation/launch/navigation.launch`

```xml
<param name="oscillation_timeout" value="8.0" />
```

**原因**：末端 1 cm/s 的蠕行在 6 s 内只走 6 cm，低于 `oscillation_distance=0.1`，会被判振荡 → 清除旋转。先加 2 s，避免一次加太多掩盖真正卡住的情况。

**验证**：`/move_base/recovery_status` 是否还出现 clearing rotation；完成时间是否变长。

#### 8b. `oscillation_distance` 0.1 → 0.12

```xml
<param name="oscillation_distance" value="0.12" />
```

**原因**：让小幅修正不被当成“没动”。一次加 2 cm。

#### 8c. 最后手段：`clearing_rotation_allowed: false`

**原因**：如果 8a/8b 后仍在目标附近原地转，才考虑关闭清除旋转。窄道里会降低恢复能力，必须先确保阶段 1–6 已让目标可行。

**回退**：改回 true。

> `oscillation_timeout/distance` 是 move_base 启动参数，改完必须重启 move_base/导航栈。

---

### 阶段 9：终点倒车 / 左右摆（一次一个）

**前提**：阶段 1–6 已完成；只有确实观察到倒车或左右摆才做。

#### 9a. `allow_init_with_backwards_motion: false`

```yaml
allow_init_with_backwards_motion: false
```

**原因**：禁止终点附近从倒车轨迹开始优化，减少“退一点再进”的修正。窄道里如果出现无法脱困，回退 true。

#### 9b. `weight_kinematics_forward_drive` 4.0 → 6.0

```yaml
weight_kinematics_forward_drive: 6.0
```

**原因**：加强前进偏好，抑制倒车。一次加 2，观察窄道通过性；不够再到 10，不要一次到 20。

#### 9c. 左右摆仍存在：二选一

- `via_points_ordered: true`（路径不自交时更正确；自交路线慎用）
- 或 `weight_viapoint: 30.0 → 20.0`（减弱对旧全局路径的强拉）

**原因**：左右摆通常来自 viapoint 强拉或同伦类切换；两个一起改无法归因。

#### 9d. 最后手段：同伦类

- `max_number_classes: 5 → 1`，或
- `switching_blocking_period: 0.0 → 0.5`

**原因**：减少候选路径来回切换。先改一个，测一次；窄道里如果规划变差就回退。

---

### 阶段 10：转头后的朝向差（只调任务层）

**前提**：平移没问题，问题出在 inspect 任务 ±90° 转头后的朝向。

#### 10a. `turn_tolerance` 0.10 → 0.08

文件：`aic26_decision/launch/mission.launch`

```xml
<param name="turn_tolerance" value="0.08" />
```

**原因**：收紧转头成功角。一次 0.02，避免直接跳到 0.03 导致放过冲/来回修。

#### 10b. `turn_min_speed` 0.15 → 0.12

**原因**：末端转速更低，减少 10 Hz 底盘指令带来的过冲。

#### 10c. `turn_settle` 0.5 → 0.65

**原因**：给车体更多稳定时间再触发检测，避免“停稳后又被认为没到位”。

#### 10d. 不建议低于

- `turn_tolerance` 不低于 0.05 rad；
- `turn_min_speed` 不低于 0.10 rad/s。

**回退**：每步回上一档。

---

### 阶段 11：底盘（最后，风险最高）

**前提**：导航参数已稳定；只有末端蠕行明显顿挫/溜车才做。

#### 11a. `cmd_rate` 10 → 15

文件：`aic26_cobot/config/chassis.yaml`

```yaml
cmd_rate: 15.0
```

**原因**：减少 10 Hz 指令与 10 Hz 控制器同频带来的抖动/丢帧。先 15，不要直接 20。

#### 11b. `cmd_timeout` 0.5 → 0.4

**原因**：减少 move_base 停止发布后底盘的滑行时间。一次 0.1 s。

#### 11c. 若 15 Hz 正常，再评估 20 Hz / 0.3 s

**注意**：STM32 串口时序敏感；先架空空转测试，再落地。

**回退**：10 Hz / 0.5 s。

---

## 6. 测试记录表

每跑一次填一行：

| 阶段 | 改动（文件:参数=值） | 航点 | reset 次数 | dx (m) | dy (m) | dyaw (rad) | recovery | 耗时 (s) | 备注 |
|---|---|---|---:|---:|---:|---:|---|---:|---|
| 0 基线 | 无 | `car_3` | | | | | | | |
| 1 | local padding -0.02 | `car_3` | | | | | | | |
| 2 | local padding -0.03 | `car_3` / `station_1` | | | | | | | |
| 3 | local padding -0.04 | `station_1` | | | | | | | |
| 4 | min_obstacle_dist 0.10 | `car_1` | | | | | | | |
| 5 | min_obstacle_dist 0.08 | `car_3` | | | | | | | |
| 6 | min_obstacle_dist 0.06 | `station_1` | | | | | | | |
| 7a | xy_goal_tolerance 0.08 | 代表点 | | | | | | | |
| 7b | xy_goal_tolerance 0.06 | 代表点 | | | | | | | |
| 7c | yaw_goal_tolerance 0.10/0.08 | 代表点 | | | | | | | |
| 8a | oscillation_timeout 8.0 | 代表点 | | | | | | | |
| 8b | oscillation_distance 0.12 | 代表点 | | | | | | | |
| 9a | allow_init_backwards false | 代表点 | | | | | | | |
| 9b | forward_drive 6.0 | 代表点 | | | | | | | |
| 10 | turn_tolerance 0.08 | station_* | | | | | | | |
| 11 | cmd_rate 15.0 | 代表点 | | | | | | | |

**判定规则**：

- reset 次数下降 + 最终误差下降 + 完成时间不明显变长 → 保留。
- reset 次数增加或完成时间明显变长 → 回退。
- 误差方向从“远离最近墙”变成随机 → 虚拟核心问题已解决，剩下的是定位/容差。

---

## 7. 调参命令与生效方式

### 7.1 TEB 参数（可动态生效）

```bash
source /opt/ros/noetic/setup.bash

# 示例：逐条执行，不要一次改一堆
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance 0.08
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS yaw_goal_tolerance 0.10
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.10
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS allow_init_with_backwards_motion false
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS weight_kinematics_forward_drive 6.0
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS via_points_ordered true
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS weight_viapoint 20.0
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_number_classes 1
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS switching_blocking_period 0.5
```

- `dynparam` 只改运行值，重启丢失；确认有效后写回 YAML。
- 每次改完用 `dynparam get` 或 `rosparam get` 确认。
- 不要同时改多个 TEB 参数。

### 7.2 costmap `footprint_padding`

- 推荐直接改 `local_costmap_params.yaml` 并重启导航栈。
- 想先试运行值：

```bash
rosrun dynamic_reconfigure dynparam set /move_base/local_costmap footprint_padding -0.02
```

- 注意：inflation 层的 inscribed radius 可能需要在 costmap 重建后更新；如果 RViz 里 footprint/膨胀没变化，重启导航栈。
- 改完确认：

```bash
rosparam get /move_base/local_costmap/footprint_padding
rosparam get /move_base/global_costmap/footprint_padding   # 应仍为 0.0
```

### 7.3 move_base 振荡/恢复参数

必须改 `aic26_navigation/launch/navigation.launch` 后重启 move_base/导航栈：

```xml
<param name="oscillation_timeout"  value="8.0" />
<param name="oscillation_distance" value="0.12" />
<!-- 最后手段才动 -->
<!-- <param name="clearing_rotation_allowed" value="false" /> -->
```

### 7.4 任务层转头参数

改 `aic26_decision/launch/mission.launch` 后重启 `mission_controller`（或整个 mission.launch）：

```xml
<param name="turn_tolerance" value="0.08" />
<param name="turn_min_speed" value="0.12" />
<param name="turn_settle"   value="0.65" />
```

### 7.5 底盘参数

改 `aic26_cobot/config/chassis.yaml` 后重启 `chassis` 节点；这是硬件节点，先架空测试：

```yaml
cmd_rate: 15.0
cmd_timeout: 0.4
```

---

## 8. 不要做的事

1. **不要先收紧 `xy_goal_tolerance` / `yaw_goal_tolerance`**：航点仍在 0.272 m 虚拟核心内时，收紧只会增加 `not feasible` reset。
2. **不要只调 `min_obstacle_dist`**：它不改变 costmap 的 `253` 碰撞检查。
3. **不要一次改多个参数**：至少保持“一次一个”，否则无法归因。
4. **不要同时改 padding、min_obstacle_dist、xy 容差**：按阶段来。
5. **不要把 `free_goal_vel` 改成 true**：允许终点非零速度，会冲过点。
6. **不要先关 recovery**：先解决可行性。
7. **不要为了 0.5 cm 把 padding 降到 -0.06 以下或 min_obstacle_dist 降到 0.04 以下**：物理安全余量会不够。
8. **不要改动 `inflation_radius: 0.20` / `cost_scaling_factor: 10`**：先保持，避免同时改变窄道居中行为。

---

## 9. 预期结果与停止条件

按阶段 1–6 的顺序，预期：

- `traffic_light_1`、`car_1`、`car_2`、`car_3`、`corner_4`：reset 消失，最终平移误差从约 4–5 cm 降到 1–2 cm。
- `station_2`、`station_3`、`traffic_light_2`：本来只是 4 mm 临界，应该更稳定。
- `station_1`：可能仍有 1–2 cm 偏移；这是最紧的点。如果必须零偏移，唯一干净的办法是把 `station_1.y` +0.03 m（可选，尊重“尽量不改航点”）。
- 如果所有航点仍有**同方向、同大小**的偏移：停止调 TEB，检查 AMCL/地图/航点标定。

**停止条件**：

- 症状消失（不再反复修正、偏移可接受）；
- 或改动开始让完成时间/窄道通过性变差；
- 或已经到阶段 6/7 的合理下限。

---

## 10. 相关文件清单

| 文件 | 作用 |
|---|---|
| `aic26_navigation/config/local_planner_params.yaml` | TEB 参数（容差、min_obstacle_dist、倒车、viapoint、同伦类） |
| `aic26_navigation/config/local_costmap_params.yaml` | local costmap；建议在此加 `footprint_padding` |
| `aic26_navigation/config/costmap_common_params.yaml` | footprint / padding / obstacle layer（global+local 共用） |
| `aic26_navigation/config/global_costmap_params.yaml` | global costmap，inflation 0.20 |
| `aic26_navigation/config/amcl_params.yaml` | AMCL 粒子、更新阈值、初始协方差 |
| `aic26_navigation/launch/navigation.launch` | move_base 振荡/恢复/频率参数 |
| `aic26_decision/launch/mission.launch` | 任务节点 timeout、turn 参数 |
| `aic26_decision/scripts/mission_common.py` | `go_to` / `rotate_to` / `turn_in_place` 实现 |
| `aic26_cobot/config/chassis.yaml` | 底盘速率、超时、标定、IMU 融合 |

---

## 11. 备注

- 本文所有“当前值”是 2026-10-28 分析时从运行中的 ROS master 读取的；与 YAML 一致。
- 当前工作区是 git worktree，且已有本地修改（`real_waypoints.yaml`、`mission_controller.py` 等）；改 YAML 前建议先备份或 commit，便于回退。
- 调参测试是实车运动，必须确保场地清空、急停可用，并有人值守。

---

## 12. 附录：goal_error_monitor.py

保存到 `/tmp/goal_error_monitor.py` 或工作区的 `tools/` 下：

```python
#!/usr/bin/env python3
# goal_error_monitor.py
# 必须在发送 goal 之前启动，否则会漏掉 /move_base/current_goal。
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

goal = {'p': None}
printed = {'done': False}
listener = None

def on_goal(msg):
    goal['p'] = msg

def on_result(msg):
    if msg.status.status != 3 or goal['p'] is None:   # 3 = SUCCEEDED
        return
    if printed['done']:
        return
    try:
        listener.waitForTransform('map', 'base_footprint',
                                  rospy.Time(0), rospy.Duration(2.0))
        trans, rot = listener.lookupTransform(
            'map', 'base_footprint', rospy.Time(0))
    except Exception as err:
        rospy.logwarn('lookup failed: %s', err)
        return
    gyaw = tf.transformations.euler_from_quaternion([
        goal['p'].pose.orientation.x, goal['p'].pose.orientation.y,
        goal['p'].pose.orientation.z, goal['p'].pose.orientation.w])[2]
    ryaw = tf.transformations.euler_from_quaternion(rot)[2]
    dx = trans[0] - goal['p'].pose.position.x
    dy = trans[1] - goal['p'].pose.position.y
    dyaw = math.atan2(math.sin(ryaw - gyaw), math.cos(ryaw - gyaw))
    printed['done'] = True
    rospy.loginfo('FINAL ERROR dx=%.3f dy=%.3f dyaw=%.3f', dx, dy, dyaw)

rospy.init_node('goal_error_monitor')
listener = tf.TransformListener()
rospy.Subscriber('/move_base/current_goal', PoseStamped, on_goal)
rospy.Subscriber('/move_base/result', MoveBaseActionResult, on_result)
rospy.spin()
```

运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/mowen/aic26_ws_speech/devel/setup.bash
python3 /tmp/goal_error_monitor.py
```

输出示例：

```
[INFO] [1793...]: FINAL ERROR dx=-0.043 dy=0.004 dyaw=0.031
```

`dx/dy = 机器人 - 目标`：

- `car_1`/`car_3` 如果 `dx` 为负（向 −x），说明停在墙（+x）的远离侧，是虚拟核心短差；
- `station_1` 如果 `dy` 为正（向 +y），说明停在下方墙的远离侧，同上。

---

## 13. 变更记录

| 日期时间 | 阶段 | 文件 | 改动 | 生效方式 | 验证结果 |
|---|---|---|---|---|---|
| 2026-10-28 | 阶段 1 | `aic26_navigation/config/local_costmap_params.yaml` | 新增 `footprint_padding: -0.02`（只影响 local costmap） | YAML 已写入；同时用 `dynparam set /move_base/local_costmap footprint_padding -0.02` 对当前运行栈生效 | `dynparam get` = -0.02；`rosparam get /move_base/local_costmap/footprint_padding` = -0.02；global 仍为 0.0；local footprint 从约 0.272×0.284 缩到约 0.232×0.244；虚拟核心 0.272 → 0.232 m |

备份：

- `/home/mowen/aic26_ws_speech/docs/backups/local_costmap_params.yaml.20261028_stage1`

回退（当前运行栈，不改文件）：

```bash
rosrun dynamic_reconfigure dynparam set /move_base/local_costmap footprint_padding 0.0
```

回退（文件）：

```bash
cp /home/mowen/aic26_ws_speech/docs/backups/local_costmap_params.yaml.20261028_stage1 \
   /home/mowen/aic26_ws_speech/src/aic26_navigation/config/local_costmap_params.yaml
```

### 阶段 1 的预期

- 0.226 m 的航点（`car_1`/`car_2`/`car_3`/`corner_4`/`traffic_light_1`）仍比 0.232 m 虚拟核心少 0.6 cm，**可能仍偶发 `not feasible`**；
- `station_1` 0.207 m 仍差 2.5 cm，**大概率仍会提前停/修正**；
- 这一步的目的是确认“缩小 footprint → reset/偏移减少”的方向是否成立。如果完全没有改善，先检查现场动态障碍，再考虑阶段 2（-0.03）。

### 阶段 1 的测试

按第 0 节的单点流程，优先测：

1. `car_1`（0.226 m，上次日志有 reset）；
2. `station_1`（0.207 m，预期仍不理想）；
3. `car_3`（0.56 m 窄道）。

记录：

- `not feasible` 次数（与上次任务日志的 4 次对比，或用单点基线对比）；
- `FINAL ERROR dx/dy/dyaw`；
- 是否触发 `Rotate recovery` / `Clearing costmap`；
- 耗时。

判定：

- reset 次数下降、偏移变小 → 保留 -0.02，进入阶段 2（-0.03）；
- 完全无变化 → 先排查现场障碍/AMCL，不要继续盲目降 padding；
- 变差 → 立即回退 0.0。
