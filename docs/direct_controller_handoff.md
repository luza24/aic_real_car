# 直接航点控制器方案：改动清单 + Astra 可行性评审说明

- 日期：2026-11-01
- 工作区：`/home/mowen/aic26_ws_speech`（git worktree，分支 `feature/speech-announcement`，HEAD `8cfc49d`）
- 用途：把 `src/` 发给 Astra 时，配合本文件与 `direct_controller_changes.patch`，说明**哪些是本次新增/修改、哪些是原本就有的未提交改动**，以及需要评审的关键假设。

---

## 0. 先看这里（给 Astra）

本次方案的核心是：**不使用 move_base / TEB**，新增一个独立 ROS 节点，读取 map 帧航点，用 AMCL 的 `map→base_footprint` TF 做闭环，直接向底盘发布 `/cmd_vel`。

请重点评审：

1. 控制律在**低速、位姿更新粒度 10 cm / 0.1 rad、20 Hz 控制**下是否稳定；
2. 状态机的失败/恢复路径是否会卡死或漏停车；
3. 安全层（激光前向急停、TF/AMCL 看门狗、命令独占、退出零速）是否足够；
4. 直线段静态地图预检的假设是否成立（航点直线可达、地图未知区处理、走廊半宽）；
5. 实车 footprint 与检查走廊不一致的风险；
6. 哪些必须改完才能实车试跑。

**注意**：`src/` 里还有大量与本次无关的未提交改动（第 3 节列出），请勿把它们算作本方案的一部分。

---

## 1. 本次新增的文件

| 文件（相对工作区根） | 行数 | 作用 |
|---|---:|---|
| `src/aic26_decision/scripts/direct_waypoint_controller.py` | 945 | 独立 rospy 节点。订阅 TF `map→base_footprint`、`/scan`、`/amcl_pose`、`/map`；发布 `/cmd_vel`、`/direct_waypoint/status`。不 import `mission_common`，不创建 move_base action client |
| `src/aic26_decision/config/direct_controller.yaml` | 85 | 全部参数默认值（航点、控制、安全、超时） |
| `src/aic26_decision/config/direct_test_waypoints.yaml` | 28 | 首轮验证用安全子集航点（顶部长廊 3 点） |
| `src/aic26_bringup/launch/direct.launch` | 106 | 一键启动：backend（硬件/仿真）+ map_server + AMCL（**不含 move_base**）+ 控制器 |
| `docs/direct_waypoint_controller.md` | 286 | 完整设计与实现说明（**在 `src/` 之外**，发 src 时不会带上，需要单独附） |

节点内部结构（方法级）：

```
DirectWaypointController
├─ _on_scan / _on_amcl / _on_map          回调，只存数据，不阻塞
├─ _lookup_pose() -> (x, y, yaw)          TF map→base_footprint, Time(0)
├─ _front_min()                           前向 ±25° 最小有效激光距离
├─ _safety_state(now) -> ok/blocked/fatal 激光、扫描超时、AMCL 看门狗、命令冲突
├─ _check_cmd_owners() / _verify_cmd_exclusive()
├─ _segment_clear(x0,y0,x1,y1)            静态地图直线走廊预检
├─ _rotate_to / _drive_command            控制律
├─ _ramp / _stop / _send
├─ _wait_ready / _run_check_only / _run_dry_run
├─ _drive_waypoint(waypoint)              单点状态机
└─ run()                                  航点循环、loop、失败策略
```

---

## 2. 本次修改的文件

| 文件 | 改动量 | 内容 |
|---|---|---|
| `src/aic26_navigation/launch/navigation.launch` | +33 / −27（含缩进） | 新增 `<arg name="use_move_base" default="true"/>`；把整个 move_base 节点包进 `<group if="$(arg use_move_base)">`。**默认行为与原文件完全一致**，只有 `direct.launch` 显式传 `use_move_base:=false` 时才跳过 move_base |
| `src/aic26_decision/CMakeLists.txt` | +1 | `catkin_install_python(PROGRAMS ...)` 列表增加 `scripts/direct_waypoint_controller.py` |

精确 diff 见 `direct_controller_changes.patch`（含新增文件全文 + 这两处修改）。

---

## 3. 原本就存在、**不是本次改动**的文件

评审时请忽略（这些是之前语音/调参/相机工作留下的未提交改动）：

```
 M src/aic26_cobot/config/status.yaml
 M src/aic26_cobot/scripts/camera_node.py
 M src/aic26_cobot/scripts/hardware_status_node.py
 M src/aic26_decision/config/real_tasks.yaml
 M src/aic26_decision/config/real_waypoints.yaml
 M src/aic26_decision/scripts/mission_controller.py
 M src/aic26_navigation/config/local_costmap_params.yaml
 M src/aic26_navigation/config/local_planner_params.yaml
 M src/aic26_perception/results/summary.json
 M src/aic26_perception/scripts/car_detector.py
 M src/aic26_perception/scripts/people_detector.py
 M src/aic26_perception/scripts/traffic_light_detector.py
?? camera_preview.sh
?? capture_photos.py
?? capture_photos.sh
?? src/aic26_decision/config/real_tasks.yaml.bak-20261028
?? src/aic26_decision/config/real_waypoints.yaml.bak-20261028
?? src/aic26_decision/scripts/mission_controller.py.bak-gui-freeze
```

`docs/` 目录原本就存在（含 10-28 的导航调参笔记），本次只在其中新增了 `direct_waypoint_controller.md`。

---

## 4. 方案逻辑摘要

### 4.1 状态机（单航点）

```
WAIT_READY → [直线段预检] → ROTATE → DRIVE_SETTLE → DRIVE
           → ALIGN_SETTLE → ALIGN_YAW → DONE_SETTLE → DWELL → 下一点
                          ↑
                 BLOCKED（可恢复；恢复后从 ROTATE 重来）
```

- 转向 20 s、直行 120 s、终点对准 15 s；直行段 4 s 内位移 < 0.03 m 判卡死。
- 到位：距离 ≤ 0.08 m 连续 2 tick；终点 yaw ≤ 0.06 rad 连续 3 tick。
- 默认失败即中止整条路线（`continue_on_failure=false`）。

### 4.2 控制律

```
# 原地转向
e = wrap(target_yaw - yaw)
|e| <= 0.06 连续 3 tick → 停
w = sign(e) * clamp(1.2*|e|, 0.10, 0.35)

# 直行
d = hypot(tx-x, ty-y); bearing = atan2(ty-y, tx-x); e = wrap(bearing-yaw)
|e| > 0.60 → v=0，原地重对准
否则:
  v = min(0.15, 0.8*max(0, d-0.05)); if d>0.20: v=max(v,0.05)
  v *= max(0.35, 1-|e|/0.60)
  w = clamp(1.0*e, -0.45, 0.45)

# 输出斜坡（急停绕过）
v_cmd = clamp(v_target, v_prev ± 0.5*dt)
w_cmd = clamp(w_target, w_prev ± 1.0*dt)
```

### 4.3 安全层

- 激光前向 ±25° < 0.25 m → 零速；>0.35 m 持续 0.8 s 才恢复；阻塞 10 s 判失败。
- TF 丢失 0.5 s → 零速；2 s 内恢复则从 ROTATE 重来，否则失败。
- `/scan` 超时 0.5 s → 零速。
- AMCL 看门狗：相邻位姿跳变 >0.30 m / 0.35 rad、协方差超阈值持续 0.5 s → 零速；位姿 >3 s 未更新时**仅当 `/odom` 显示实际运动**才判定异常并锁存到新位姿到达（AMCL 静止时不发布，按命令速度判断会抖动）。
- `/cmd_vel` 独占：启动时 + 每 1 s 检查发布者，除本节点外出现任何发布者即退出。
- 后端互斥：`real` 要求 `/chassis` 存在且 `/gazebo` 不存在。
- 直线段预检：沿直线每 0.05 m 采样，计算采样点到“占据/未知栅格矩形边缘”的真实距离；< `segment_half_width=0.14 m` 判失败。
- 退出：20 Hz × 0.8 s 零速 burst。

### 4.4 本轮未做的

- 不做绕障；不做全局规划；不做路径平滑/纯跟踪。
- 不接入 mission_controller（检测、语音、红绿灯等待都不在直控节点里）。
- 不用轮式 odom 做闭环，只依赖 AMCL TF（没有 fallback）。

---

## 5. 可行性评审需要关注的假设与风险

1. **AMCL 位姿更新粒度**：speech 工作区 AMCL 是 300/1200 粒子、`update_min_d=0.10 m`、`update_min_a=0.10 rad`。`map→base_footprint` 不是每帧更新，低速时会呈阶梯状。20 Hz 控制 + 10 cm 位姿步进，P 控制是否会产生锯齿/超调，请评估；如有必要建议改为 50 Hz 插值/使用 `/amcl_pose`+odom 短时推算。
2. **地图与航点**：speech 用的地图是 `world.yaml`，航点 `real_waypoints.yaml` 是给 TEB 的，部分点位/坐标未在实车重测。直控走**相邻航点直线**，可能穿墙或切内角；静态预检只能发现地图上的静态障碍，不能替代现场核验。
3. **Footprint 不一致**：speech 的导航配置硬编码仿真方形 footprint `0.284×0.272（半宽 0.136）`；main 分支的真车八边形是 `0.3425×0.285（半宽 0.1425）`。直控预检走廊半宽取 0.14，**比真车半宽 0.1425 还小 2.5 mm**；若按真车几何，应提高到 ≥0.145，或统一使用真车 footprint。请评审这一处。
4. **激光覆盖**：激光安装高度约 16 cm，`range_min=0.2`，车头前方 0.2 m 内是盲区；只保护前向 ±25°，**没有侧向/后方保护**，窄道侧碰无法靠本控制器避免。0.25 m 停止阈值对应的车头物理净空约 0.17 m（激光前置约 8 cm），测量误差与底盘停止延迟会吃掉余量。
5. **底盘执行特性**：串口 10 Hz、无加速度限制、`cmd_timeout=0.5 s`；控制器的斜坡只在指令侧生效，底盘跟踪滞后与停车过冲（尤其 0.8 rad/s 原地转）未建模。请评估停止距离 0.05 m、到位容差 0.08 m 是否现实。
6. **终止行为**：节点进程崩溃时主机侧无法停车，依赖底盘 0.5 s 超时和固件兜底；退出零速 burst 0.8 s 是否足够，请评审。
7. **原地旋转扫掠**：终点 `ALIGN_YAW` 会在航点位置原地旋转；若航点贴墙（历史分析中 station_3 等），扫掠半径约 0.197 m 可能越界。直控没有 costmap 约束。
8. **单线程时序**：所有回调与循环在一个 Python 进程；TF lookup 用 `Time(0)`（非阻塞），但 AMCL/scan 数据量大时 GIL 竞争是否会导致 20 Hz 抖动，需要实车确认。
9. **验证强度**：目前只有离线验证（隔离 ROS master + 假激光 + 假底盘积分器），没有真实底盘、真实 AMCL、真实延迟与打滑。请不要把离线闭环误差（0.08 m）当作实车精度承诺。

---

## 6. 已做的离线验证

环境：车端独立 master（端口 11312–11314），map_server + 假 `/scan` + 假底盘（积分 `/cmd_vel` 广播 TF），**未启动 chassis 节点、未接串口、未产生真实运动**。

| 测试 | 结果 |
|---|---|
| `check_only`：起点→traffic_light_1→station_1→corner_2 | 3 段全部 PASS（world.yaml，走廊半宽 0.14） |
| `dry_run` | 目标距离 1.00 m、航向误差 0.10 rad → 意图 `v=0.13 m/s, w=0.10 rad/s`；无非零 `/cmd_vel` |
| 前方 0.23 m 障碍 | 进入 BLOCKED，速度保持 0 |
| 闭环三航点（理想单轮模型） | 全部到达；位置误差 0.079 / 0.080 / 0.079 m；yaw 误差 0.056 / 0.018 / 0.030 rad |

---

## 7. 建议 Astra 回答的问题

1. 在上述位姿更新粒度与执行延迟下，这套 P 控制 + 斜坡是否会稳定？需要改增益/加前馈/加位姿插值吗？
2. 状态机是否存在卡死、重复动作、恢复后从错误阶段继续等缺陷？
3. 安全层还缺什么最小必要项（例如侧向激光、里程计超时、看门狗优先级）？
4. 直线段预检的算法（到栅格矩形的距离、未知区视为阻挡）是否合理？走廊半宽 0.14 与真车 0.1425 的矛盾如何定？
5. 原地转向扫掠、贴墙航点、窄道侧碰，哪些必须通过改航点/改策略解决，而不是改控制器？
6. 如果要合入 main 的定位改进（omni/500-2000 粒子/3 cm 更新/真实足迹），哪些参数/代码必须同步？
7. 实车首测的最小安全程序（速度、场地、人员、急停、bag 记录）是什么？
8. 哪些设计应该推翻重做，而不是调参？

---

## 8. 发送材料建议

- `src/` 整个目录（新增与修改文件都在其中）；
- 本文件 `direct_controller_handoff.md`；
- `direct_controller_changes.patch`（仅本次 7 个文件的精确补丁，可用于核对）；
- 如需完整逻辑说明：`docs/direct_waypoint_controller.md`（在 `src/` 之外，需单独附）。

补丁统计：

```
7 files changed, 1483 insertions(+), 27 deletions(-)
```
