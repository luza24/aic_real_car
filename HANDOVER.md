# 接续提示词：AIC26 实车（ROS1 Noetic）导航/定位调试

你在接续一个 catkin 工作区 `/home/mowen/aic26_ws`。机器人在场、可跑，但**任何自动驾驶都必须先满足下面第 6 节的三道闸**。先读完本文，再动任何 launch。

## 0. 一句话现状

仿真正常；实车定位的根因已基本查清：雷达解包、**地图倾斜（成因已定：雷达本体偏角 2.29° + 摆放误差）**、AMCL 参数。2026-10-29 已换成新图（`world_real.pgm`，916 格，内部特征 +47%），雷达偏角已补偿，`realign_map.py` 不再需要。

**当前状态（2026-11-01 夜）**：**一整圈能稳定跑完**（4 次 13/13，157/168/182/187 s，单圈最多 1 次重试），
四路看门狗（σ / 扫描-地图 / 卡死 / 新增的 `map->odom` 跳变）都在实车触发过。
`map->odom` 跳变看门狗已在 22:39 / 22:50 两轮实车生效（0.32 m/10.0°、0.34 m/9.3°，见 13.8.1）。
**剩下的问题是位姿在一圈里缓慢漂移 0.2–0.5 m**（原地旋转与侧移段，航向攒 5–9°），
第二圈以前基本必挂；2026-11-02 21:45（no-strafe）已连跑两整圈、看门狗零触发（13.8.4），
但 yaw 仍以 ~5°/圈累积，第三圈起未验证，且曾因此撞上 `traffic_light_2` 的灯杆。参数与结论见第 13 节。

### 两个已定量的量（2026-10-29）

```
雷达本体偏角   静止 8 帧四墙拟合 −2.29°（离散 0.08°），补偿后 −0.14°
                → lidar.yaml angle_offset_deg = 2.29
摆放误差       同一套 AMCL 实测 map->odom：好摆放 0.02–0.11°，差摆放 1.8–6.9°
```

⇒ 历史记录里"每轮 2.6–5.2° 的旋转残差" = 固定 2.3°（雷达）+ 每轮不同的摆放误差。**这两个量现在能分开测**，不要再当成一个待解的谜。

### 仍然存在的几何退化（改图救不了）

`station_1→station_2` 左走廊、底部开阔区，地图内部特征数仍是 **0 / 接近 0**：那里雷达只能看见两道平行墙，画线对雷达不可见。AMCL 必然沿墙滑移（实测 0.41 m）。只能行为兜底：低速通过 + 扫描一致性看门狗。

### round5 实测（历史基线，0.25 m/s）

```
t=0.5–71s   σ 0.012–0.020, map->odom 0.006→0.151 m, dyaw 0.8→−4.3°   优秀
t=77–90s    0.164→0.278 m, σ 仍 0.02                                  平稳
t=106.5s    σ 0.157 → 看门狗在 0.25 处触发，取消目标停车
t=112.8s    σ 0.597（场地底部开阔区 (−0.06,−1.37)，长直墙导致沿墙退化）
```

对 `map->odom` 做完整 2D 仿射拟合（`dx = a + b·x + c·y`、`dy = d + e·x + f·y`），三轮结果：

```
旋转残差      : +5.2° / +2.6° / +4.2°        ← 摆放误差 + 固定 2.3° 雷达偏角
同轴"尺度"    : x −2% 级、y +2~4% 级，三轮符号/大小不稳 ⇒ 拟合噪声，不是真实形变
交叉项         : ax[2] 与 −ay[1] 不等 ⇒ 不是纯旋转，但主体仍是旋转
```

**重要纠正**：此前一版写过"地图沿 x 偏小 5.9%、y 偏大 5.5%"，那是只回归同轴项的错误结论，**已作废**。刚体变换保距，不可能引入尺度误差；地图没有形变。**AMCL 自己吸收刚体残差，不需要为它改地图或加尺度。**

## 1. 环境与设备

- ROS1 Noetic，catkin，4 核 N100。`source devel/setup.bash` 后才可 roslaunch。
- 设备（udev 规则按 `ID_PATH`，即 **USB 物理端口**；设备换口或重插会改变指向。曾因把 IMU 换到另一个口导致 `carserial`/`imu` 互换，整栈看起来"什么都正常但没有 odom"）：
  - `/dev/carserial` → STM32 底盘，115200，帧 `AA BB 0A 12 02`（指令）/ `AA BB 0A 0A`（反馈，12 B）
  - `/dev/imu` → WIT IMU，**9600 / 10 Hz**（模块寄存器被复位，2026-10-29 实测；`imu.launch` 与 `config/imu.yaml` 已改 9600，见第 7 节第 6 条）
  - `/dev/lslidar` → 乐神 N10_P，460800，108 B 帧 `A5 5A`；**本体相对车头偏 −2.29°**，已用 `angle_offset_deg: 2.29` 补偿（见第 0、3 节）
  - `/dev/video0` → Orbbec DaBai，UVC（会同时出现 `video0`/`video1`；拔插后先 `v4l2-ctl --list-devices` 确认，**不要因为一次没查到就判定相机不在**）
- 关键包：`aic26_cobot`（驱动）、`aic26_navigation`（AMCL/move_base/TEB）、`aic26_decision`（任务节点）、`aic26_slam`（地图与工具）、`aic26_bringup`（唯一入口）。
- N100 的 CPU 调频器默认是 `powersave`（全核锁 800 MHz），跑建图/导航会明显卡顿，且 `load average` 会飙到 3 而 ROS 其实只占两成：

  ```bash
  sudo sh -c 'for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$f"; done'
  ```

  切到 `performance` 后同样负载下 load 从 2.96 掉到 0.73。

## 2. 绝对不要做的三件事

1. **不要用文本编辑器/文本通道碰 `.pgm`**。上一版 `world_real.pgm` 就是这样被 UTF-8 转码毁掉的（198647 个 `EF BF BD`）。工具一律用 `python3 .../realign_map.py`、`clean_map.py`。
2. **不要在车不在“起点”时重启导航栈**。重启会把 odom 与 AMCL 同时初始化到 `(1.6, 1.7, π)`，两者一起错，任何闸门都检测不到 → 车会照错误位姿开出去。
3. **不要把 AMCL 似然调锐**（【存疑，单次对照】）。一次对照：`sigma_hit 0.08 / z_hit 0.8 / 90 束` 时 σ 一度掉到 0.005（过度自信），第 34 s 起反复丢失、第 101 s 跳到 4.06 m 外并撞墙；原始值那一轮则稳定跟踪到 106 s。两组各只跑过一次，不能算证明，但**默认值更安全，别调锐**。

## 3. 已完成的关键修复（按影响排序）

| # | 问题 | 证据 | 修法（文件） |
| --- | --- | --- | --- |
| 1 | **雷达解包错误**：N10_P 是 6 字节/点，代码按 2 字节读，每 4 点错 3 点 | 静止两帧均方残差 0.212 m、720 束 77% 无回波、最长连续弧 1° | `aic26_cobot/scripts/hardware_common.py` 的 `parse_n10p_packet`（16 点 × 6 B，大端 mm + 强度 + 第二回波）；`lidar.yaml` 的 `bins: 540` |
| 2 | 修好后：静止残差 **0.014 m**、有效束 **75%**、最长弧 8°，旋向已验证（转 +62.7° → 图样移 −62.7°） | 同上 |
| 3 | **real 的 AMCL 地图只有地板线**（雷达看不见），无特征 → 位姿跟轮子漂 | world.pgm 608 格中 564 格在墙线 5cm 内；场地内仅 70% 已知 | 改用现场 gmapping 建的 `world_real.yaml`（`backend:=real` 的 `map_name`） |
| 4 | 新地图整体歪 **−3.38°** + 中心偏 (−0.126, +0.094) | Hough 共线 48 格 @ −3.38°（world_sim 为 0.00°）；最小外接矩形独立给出 −3.65°，两者一致 | `realign_map.py --yaw 3.38 --dx 0.126 --dy -0.094`，再 `clean_map.py --close 2 --min-neighbours 1`。结果：墙 0.00°、连通块 25→8、`corner_3` 余量 0.20→0.40 m。**【存疑】旋转的*成因*未定**（可能是建图起步时车头摆放偏了，也可能是雷达安装有偏角，或两者叠加）；**【存疑】平移量是假设**"物理房间与画线场地同心"，残余最大 ±0.1 m |
| 5 | **IMU 全程失效**：`imu.launch` 的 `baud` 覆盖了 yaml 里的值（当时两处不一致） | `/imu/data` 0 Hz | `imu.launch` 与 `config/imu.yaml` 统一成 **9600**（2026-10-29，见第 13 条；曾写“两处都改 115200”，**已作废**）；节点加 3 s 无帧告警 |
| 6 | 旧融合用**磁航向**当绝对真值（室内不可用），且 2 Hz + 0.02 rad 限幅追不上 | 代码与限幅值可从 git 复核；**【推断】"追不上"是按 0.04 rad/s 最大修正速率算出来的，未做实测对照** | 改为**只融陀螺角速率**（静止学零偏）；`use_imu_yaw` 默认 true；`imu_gyro_gain 0.8` |
| 7 | `angular_scale` 一个系数同时乘在指令和反馈上 | 偏航系统性低估 | 拆成 `*_cmd_scale` / `*_odom_scale` |
| 8 | 实车与 sim 的初始位姿三处不一致（1.6/1.7/1.7） | 起步偏 10–20 cm | 全部统一 `(1.6, 1.7, π)` |
| 9 | ~~横向量纲~~ **已作废**：那是“只回归同轴项”的拟合假象 | 完整 2D 仿射拟合后，同轴系数三轮不稳定（−2%/+2% 级），主体是每轮不同的旋转（2.6/4.2/5.2°） | 所有 `*_scale` 已回退 1.0；不要再做这类全局拟合来定尺度（x、y 沿线共线，无法区分轴尺度与航向偏差） |
| 10 | **雷达本体相对车头偏 −2.29°**：每张新图都自带这个倾角，与摆放无关 | 静止 8 帧四墙联合拟合：无补偿 −2.29°（离散 0.08°），写入补偿后 −0.14° | `lidar.yaml` 的 `angle_offset_deg: 2.29`。补偿后重扫，四墙 +0.81/−0.25/−0.03/−0.03，**`realign_map.py` 不再需要** |
| 11 | **地图内部特征太少**，AMCL 在左走廊沿墙滑移（实测偏 0.41 m） | 新旧图在 `station_1→station_2` 走廊的内部占据格都是 **0**，底部开阔区 27/28 | 重扫并换用 v2 图（916 格，内部 151 vs 103，+47%，房间尺寸与旧图逐厘米一致 4.450×4.400）。**但走廊内部仍是 0——这是几何退化，改图救不了** |
| 12 | **看门狗只看 σ，抓不到"自信地错"** | `station_2` 处 σ 0.0476（健康），而扫描↔地图中位残差 0.41 m；旧版继续顶着灯杆/墙磨 25 s | 新增三道看门狗（见第 6 节）：σ / 扫描-地图一致性 / 卡死。实测 2 s 内停车 |
| 13 | IMU 模块被复位回 9600 / 10 Hz（`/imu/data` 0 Hz，115200 只读到全零） | 115200 读到 2048 B 全 `00`；9600 读到 `55 51/52/53` 各 10.0 Hz，acc z 1.006 g | `imu.launch` 的 `baud` 与 `config/imu.yaml` 两处都改 9600（哪个都别单独改） |
| 14 | **横移时陀螺输出虚假偏航**：每 0.37 m 纯横移，odom 的 yaw 漂 +8.2 / −12.1°，而车根本没转 | 同一段横移三轮对照：IMU 开 +8.2/−12.1°，IMU 关 −0.1/+0.7°，加门限后 −0.3/−0.2° | `chassis_node` 的 `imu_strafe_gate: 0.02`：`|v_y|` 超过门限时不用陀螺、改用轮速算 yaw。这解释了 car 行（`car_1/2/3` 全在 x=1.10，纯横移）为什么会攒出十几度偏差、局部图歪、墙角假障碍 |
| 15 | **有指令但车不动、无打滑、日志无异常**：`move_base` ACTIVE 且发 0.18 m/s，odom 3 s 只动 0.001 m | 手工 `rosservice call /chassis/enable "data: true"` 后发 0.05 m/s，odom 立即动了 8.4 cm | 底盘 `_enabled` 是软状态：为 False 时节点只发零速、不报错。`stop_robot.py` 会把它置 False。节点启动时 `_enabled` 默认 **True**，启动日志只有 `chassis: /dev/carserial @ 115200, odom odom -> base_footprint`；调用 `/chassis/enable` 时才打印 `chassis: motors enabled/disabled`（**没有** `motors enabled at start-up` 这一行）。**每次上车前先确认启动时没有残留进程把电机置 False（重启栈即复位）**；`/vel_raw` 非零而车不动=打滑，`/vel_raw` 为零=没执行 |

## 4. 当前有效配置（关键数值）

> **以第 13.1 节为准**；本节保留历年调参的理由，数值如与 13.1 冲突，一律信 13.1。

- 速度（`navigation.launch` 与 4 个 mode launch 都是 arg，可命令行覆盖）：
  现在 `max_vel_x 0.24 / backwards 0.15 / max_vel_y 0.14 / max_vel_theta 0.60 / acc_lim 0.6,0.6,0.8`；`turn_speed 0.5`（decision）。
  实测（**每档只跑过 1 次，属单次对照**）：0.12 → dyaw max 3.5°；0.18 → 5.1°；0.20 → 一圈 168 s；
  0.24 → 一圈 157–187 s、零重试。0.25 曾在旧图/旧雷达参数下于底部开阔区退化（不可直接比较）。
- AMCL：`odom_model_type omni`、`beams 90`、`sigma_hit 0.18`、`z_hit 0.55`、`update_min_d 0.03`、`update_min_a 0.15`、粒子 500–2000、`initial_cov 0.0004(real)/0.01(sim)`、`recovery_alpha_slow/fast = 0`。
  `alpha1 0.40 / alpha2 0.40 / alpha3 0.20 / alpha4 0.30 / alpha5 0.30`（2026-10-29 从默认 0.2 上调旋转项：整轮 demo 里 `map->odom` yaw 会从 −0.3° 攒到 +7.6°，终点前崩掉）。**这是单变量实验，还没验证**。
- 底盘：`feedback_rate 20`、`publish_rate 30`、`use_imu_yaw true`、`imu_gyro_gain 0.8`、**`imu_strafe_gate 0.02`**（横移时改用轮速算 yaw，见第 3 节第 14 条）、**所有 `*_scale` 均为 1.0**（`linear/lateral/angular × cmd/odom`，见第 9 节：不要凭全局拟合去改）。`disable_strafe:=true` 时 `chassis.launch` 会把 `lateral_cmd_scale` 覆盖成 0（只是运行参数，yaml 仍是 1.0）。
- 雷达：`bins 540`、`max_range 6.0`、`fill_gaps 2`、`min_coverage_deg 200`、`angle_sign -1`、**`angle_offset_deg 2.29`**（补偿本体偏角，见第 3 节第 10 条）。
- 地图：`aic26_slam/maps/world_real.yaml` —— 2026-10-29 的 v2（916 格、内部 151、四墙 +0.81/−0.25/−0.03/−0.03°、房间 4.450×4.400 m，与旧图逐厘米同帧）。旧图备份在 `world_real.pgm.bak20261029`，更早的 `world_real.pgm.orig` 未动；同目录的 `world_real_v2.pgm/yaml` 与正在使用的 `world_real.pgm` **内容相同**（md5 一致），只是备份名，别改错文件。`world.pgm` 只作线几何参考；地板线由 `virtual_walls.py` 以 LETHAL 注入两个 costmap。
- footprint（real）：八边形，`x −0.185…+0.1575`（尾部电控箱到 −0.185，原版漏了这 2.75 cm）、`y ±0.1425`（轮子外廓）。5 个 launch（demo/mission/capture/waypoints/localise）里是同一串 `$(eval ...)`（real 八边形 / sim 矩形），改要一起改；直接 `roslaunch aic26_navigation navigation.launch` 时默认仍是 sim 的矩形，需要显式传 `footprint`。
- TEB / 膨胀：现在 `min_obstacle_dist 0.08`、`inflation_dist 0.16`、`weight_inflation 3.5`、`weight_optimaltime 2.0`、`xy_goal_tolerance 0.08`、`lookahead 0.8`；局部 `inflation_radius 0.18 / cost_scaling 5.0`、全局 `0.26 / 3.5`。**TEB 的可行性检查只看 footprint 是否压 LETHAL，与 `min_obstacle_dist` 无关**（2026-11-01 查源码确认），所以"压线死锁"只能靠膨胀与权重压住，不能靠压裕度。
  历史：`min_obstacle_dist` 压到 0.04/0.07 实测会压线；`inflation_radius` 必须 ≥ 内切半径 0.1425 才会在"车体会碰到"的格子打 INSCRIBED(253)，否则 TEB 看不见车宽。`weight_optimaltime` 从 1.2 提到 2.0 是为了减少收尾慢爬——底盘在 <0.15 rad/s / ~0.05 m/s 基本不动。
- teleop：`aic26_cobot/teleop.launch` 默认 `speed 0.12 / turn 0.40`（可 `speed:=` 覆盖）；建图那圈用的是 `aic26_slam/gmapping.launch` **自带的** teleop（`launch-prefix="xterm -e"`，speed 0.1 / turn 0.5），不是 `aic26_cobot` 那个。
- 起点对位：`start gate`（map->odom ≤0.30 m/11.5°，σ ≤0.10）+ `start align`（开到 start ±0.08 m/0.10 rad，已在附近则跳过）。

## 5. 可用工具（`src/aic26_slam/scripts/`、`src/aic26_decision/scripts/`，以及注明的 `aic26_utils`）

- `frame_calibrate.py`：车在起点、AMCL 收敛后测“场地→地图”刚体修正（打印 `frame_correction x y yaw`，`--write` 可回写 waypoint 文件）。
- `measure_placement.py`（`aic26_decision`）：**建图前必做**。AMCL 收敛后打印本次摆放的真实位姿，直接给出 `initial_pose_x/y/a` 三个可粘贴的参数，并打印 `map->odom`（= 本次摆放误差）。注入建图后，gmapping 的 map 系与旧图重合且不倾斜。用法：`rosrun aic26_decision measure_placement.py --settle 12`。
- `localisation_check.py`（`aic26_navigation`）：1 Hz 算“有效束到最近占据格的中位距离”，发布 `/localisation/scan_residual` 与 `/localisation/scan_ok`；服务 `~/align` 在 ±0.15 m / ±0.08 rad 窗口内做带边界搜索并回灌 `/initialpose`。
  - **只用于修摆放误差，不要拿它救漂移**：在长直墙走廊里沿墙方向不可观测，任何位置都能把残差压到 0（实测 0.55 m 的"修正"是滑移假象）。
  - `auto_align`（**`localise.launch` 默认 true；demo/mission/capture/waypoints 现在默认 false**）：全程只在第一次 `residual > align_threshold(0.10 m)` 时自动做一次有界搜索并回灌 `/initialpose`。22:39 那轮它在 station_1 修正了 0.16 m / −2.3°，0.14 s 后出现跳变（见 13.8.1）；任务模式已把它关掉，`localise` 仍保留用于摆正起点。
- `realign_map.py --map X.yaml --yaw Y --dx .. --dy ..`：把 `p' = R(yaw)·p` 烘焙进地图（同网格）。
  - **符号**：`--yaw` 是"施加"的旋转。实测地图歪 +1.6° 时要用 `--yaw 1.60`；写 `-1.60` 会把倾角放大到 3.5°。
  - 补了 `angle_offset_deg` 之后新图本身就是正的，**正常情况下不需要这个工具**。
- `clean_map.py --close 2 --min-neighbours 1 --backup`：定向闭合补墙缝 + 只删零邻居噪点（**禁止 3×3 开运算**：实测吃掉 90% 墙）。
- `audit_map.py`：三图对比 + 13 个位点离墙线/离障碍距离 + real/sim 差异 + 报警。
- `cleanup_leftovers.py`（`aic26_utils`，**已入库并 install**）：`--dry-run` / `--quiet` / `--kill-cpu N`；`backend.launch` 的 `cleanup:=true` 会自动跑。它**不在** `~/aic26_logs/tools/`。
- 诊断：`rosrun tf tf_echo map odom`；`rostopic echo /amcl_pose` 看 covariance[0]；四墙倾角拟合见 `/tmp/wallfit2.py` 的思路（多帧、联合拟合，离散 <0.1° 才可信；单帧短弧会把噪声当倾角）。

## 6. 安全操作规程

停：`rosrun aic26_utils stop_robot.py`（禁用电机并停 demo_cycle/move_base）或
`python3 src/aic26_cobot/scripts/emergency_stop.py --port /dev/carserial --seconds 2`；恢复 `rosservice call /chassis/enable "data: true"`。

起（车必须在起点，车头朝 −x）：一条命令，`demo.launch` 会自动跑完 13 个停靠点并循环：

```bash
roslaunch aic26_bringup demo.launch backend:=real show_view:=false use_rviz:=true
# 覆盖速度：max_vel_x:=0.12 max_vel_theta:=0.40 turn_speed:=0.40
# 不侧移（试验模式：TEB 的 vy/acc_y=0、底盘 lateral_cmd_scale=0，并改选 *_nostrafe 配置）
roslaunch aic26_bringup demo.launch backend:=real disable_strafe:=true \
  show_view:=false use_rviz:=true
```

日志必须依次出现 `start gate: map -> odom = ...` 和 `start align: ...`，两者数值正常才代表可以跑。
⚠️ `demo_cycle` 在 `task_delay`（20 s）后**立即开走**，没有确认环节——清场、人站急停旁。红绿灯点没有感知时 `wait_green` 会在 `no_data_wait`（10 s）后放行。

只起定位（车不动）：`roslaunch aic26_bringup bringup.launch mode:=localise backend:=real use_rviz:=true`
（2026-10-29 新增，替代原来手工拼两条 launch）。
⚠️ **`initial_cov` 别用 0.05 去跑任务**：AMCL 静止时不更新（`update_min_d/a`），σ 会卡在 0.197 > `max_start_sigma 0.10`，`check_start_pose` 判 blocked 而 `align_to_start` 又被同一标志挡住 ⇒ 死锁。0.05 只用于校准/检查。

三道闸（`aic26_decision/scripts/mission_common.py`，0.2 s 定时）：

1. `check_start_pose()`：map->odom 或 AMCL σ 超限 → 不发目标；
2. `align_to_start()`：先开到 start 再开始任务；
3. 运行中三路**独立**看门狗（任一触发即 cancel 目标 + 停车，之后 `_send_goal`/`back_off`/`rotate_to` 全部拒绝，路线节点退出）：
   - σ：`amcl_sigma > max_runtime_sigma(0.25)` 持续 1 s → `localisation_lost`；
   - 扫描一致性：`/localisation/scan_ok == false`（有效束到最近占据格的中位距离 > `max_scan_residual(0.18)`）持续 `scan_bad_timeout(2.0)` s → `localisation_lost`；
     ⚠️ 只用 **>0.8 m 的远束**（`min_beam_range`）：贴墙时 5° 偏航只产生 2.6 cm 位移，近束会把"偏了 5°"看成"完全匹配"（实测 yaw 已错 −5° 时残差仍是 0.00–0.05，等开出 0.25 m 才报警）。
     ⚠️ 残差按 0.05 m 网格量化，正常行驶中位 0.05、常有样本恰好 0.10；阈值必须 ≥0.15，否则正常行驶就误触发（实测在 `car_1` 误停过一次）。`localisation_check` 里另有 1e-3 容差防浮点边界。
   - 卡死：`|cmd_vel| > stuck_cmd(0.08)` 但 odom 位移 < `stuck_distance(0.005)` 且转角 < `stuck_yaw(0.02)` 持续 `stuck_timeout(3.0)` s → `motion_blocked`。
   - `map->odom` 跳变：车体位姿单步位移 > `max_odom_jump(0.10)` m 或航向 > `max_odom_yaw_jump(0.06)` rad。原地转里的纯航向修正直接接受；位置跳先回滚（13.8.2），其余 → `localisation_lost`。

   ⚠️ 卡死阈值**不能再收紧**：`back_off` 只发 0.05 m/s，底盘有 ~0.7 s 死区 + 1.3 s 斜坡，1 s 窗口会把正常倒车判成卡死（实测误报过一次）。

## 7. 未解决 / 下一步

> 这一节的 1、2 两条已于 2026-11-01 完成（一圈能稳定跑通），现在的头号问题是**位姿漂移**，
> 见第 13.7 节。下面保留原文作为历史。

1. ~~还没有一轮完整的 demo 跑通~~ **2026-11-01 已完成**：13/13、157/168/182 s、单圈最多 1 次重试。
   最后那次失败是 `station_2` 处扫描残差 0.41 m（旧图/旧参数时代）。
2. 当时要盯的三个数，结论：
   - `map->odom` 的 dyaw：现在一圈会攒到 **5–9°**（不是当初期望的 1° 以内），是当前主要问题；
   - `station_2` 那段的 `scan_residual`：已不再爬到 0.4，窄道里通常 0.00–0.05；
   - TEB 新参数：`min_obstacle_dist 0.08` + 膨胀 0.18/0.26 能让窄道可行（一圈都过）。
3. 左走廊 / 底部开阔区的内部特征仍是 0（见第 0 节），**不要再改图**。若仍频繁退化，只能改路线（把检查点挪到有角点的位置）或加物理标志物。
4. `real_waypoints.yaml` 里 `station_1` 的 x 与 sim 差 **0.60 m**（real −1.00 / sim −0.40），始终未确认是有意还是笔误（第 9 节第 9 条）；`car_1/2/3` 的 real y 已上移 **+0.10 / +0.20 / +0.20 m**（相对 sim，real −1.60/−0.90/−0.30），现离障碍 0.50 / 0.85 / 0.60 m（2026-11-02 用 `audit_map.py` 复核）。
5. 底盘的 ~0.7 s 死区 + ~1.3 s 斜坡（第 9 节第 7 条）：TEB 与卡死阈值都已按此设过，**不要再收紧**。
6. IMU 现在是 9600 / 10 Hz（模块寄存器被复位过）。若再把配置改回 115200，`/imu/data` 会读不到帧；要恢复 20 Hz 必须重发模块配置命令，**当前配置下不要动**。
7. 诊断脚本（`scan_check` / `wallfit2` 那种）还在 `/tmp`，没有入库。需要时按第 5 节的描述重写，或复用 `localisation_check.py` 的 `measure()`。

## 8. 常用命令速查

```bash
# 只起定位（不动车）
roslaunch aic26_bringup bringup.launch mode:=localise backend:=real use_rviz:=true

# 量摆放误差（建图前必做；直接打印 initial_pose_x/y/a）
rosrun aic26_decision measure_placement.py --settle 12

# 建图（把上一步的三个数填进去；teleop 会自己在 xterm 里开）
roslaunch aic26_bringup mapping.launch backend:=real use_rviz:=true \
  start_x:=1.599 start_y:=1.698 start_yaw:=-3.141194

# 存图 / 清理 / 审计
rosrun map_server map_saver -f /tmp/world_new map:=/map
python3 src/aic26_slam/scripts/clean_map.py --map <yaml> --close 2 --min-neighbours 1
python3 src/aic26_slam/scripts/audit_map.py

# 运行中定位诊断
rostopic echo /localisation/scan_residual     # 中位残差；起点 0.02–0.05 属正常
rosservice call /localisation_check/align     # 只在起点用，只修摆放误差

# 单测
cd src/aic26_cobot && python3 -m unittest discover -s test    # 24 项
```

## 9. 存疑清单（不要当成结论，别据此改代码）

1. **雷达点密度/包率**：一次原始抓包测得 133 pkt/s，但 `/scan` 每帧有 406–437 个有效 bin（10 Hz 下这要求 ≥4000 点/s ⇒ ≥250 pkt/s），两者矛盾 —— 那次抓包很可能自己丢了数据。**可靠的只有**：540 bins 下 75–81% 有效、静止帧间残差 0.014 m、旋向 −62.7° 精确对应。任何"≈530 点/圈""33 包/圈"之类的推导**都属于存疑**。
2. ~~−3.38° 的成因未定~~ **已结案（2026-10-29）**：雷达本体偏角 −2.29°（静止 8 帧四墙拟合，离散 0.08°）+ 当时摆放约 1.7°。两者已能分开测，见第 0、3 节。残留存疑：这 2.29° 究竟是"雷达相对车体的安装偏角"还是"雷达内部零位偏"，外观上无法区分，但用 `angle_offset_deg` 补偿的效果等价。
3. **realign 的平移量是假设**：按"物理房间外框与画线场地同心"定，实测残余最大 ±0.1 m。若以后想更准，只能用 `frame_calibrate` 在起点实测。
4. **AMCL 调锐 vs 默认**：各只跑过一次，属于单次对照，不是统计结论。
5. **旋转残差的拆解**：2.6/4.2/5.2° = 固定 2.3°（雷达）+ 每轮摆放误差。拆解只用了一轮（补偿前后各一次四墙拟合），**还不是统计结论**；但"存在一个与摆放无关的固定倾角"这点是确定的。
6. **陀螺"静止输出精确 0"的成因未定**：只测到现象（0x52 全零），可能是模块内置死区，也可能是输出被门限；不影响"只融角速率"的做法，但**不要据此假设低速时陀螺一定可靠**。
7. **底盘低速死区与延迟各只测过一次**：0.05/0.10 rad/s 各 3 s 无动作；指令到运动约 0.7 s、斜坡约 1.3 s。属单次试验，随电池电压/地面状态可能变化。
8. **"x/y 共线导致尺度拟合失真"是对拟合假象的解释**，逻辑成立但未做数值证明（可用去相关的实验证伪）。
9. **`station_1` 与 sim 差 0.60 m**（real −1.00 / sim −0.40）始终未确认是有意还是笔误 —— 需人工到现场确认，不要自动"修正"。
10. **看门狗只验证了"能阻止后续动作"**：未验证 cancel 失败、AMCL 完全不发消息等极端路径。
11. **`world_real.pgm` 的可信度**：2026-10-29 已换成补偿后的 v2（四墙 +0.81/−0.25/−0.03/−0.03°，房间 4.450×4.400 m，与旧图逐厘米同帧），仍未与任何独立测量交叉验证。`world.pgm` 只含画线、`world_sim.pgm` 是仿真产物，都不能当真值。旧图在 `world_real.pgm.bak20261029`。
12. **TEB 裕度现在是 0.08**（见 13.1）：0.04/0.07 两档实测都会压线，压线后 footprint 落进 LETHAL 的 virtual_wall 格、TEB 不再发指令（不是发 0，是静默）。0.08 这一档已经在完整一圈里跑过（157/168/182/187 s，2026-11-01）；若 0.58 m 窄道或 0.45 m 车道仍无解，优先挪航点，不要再压裕度。
13. **v2 的北墙 +0.81° 是离群值**：其余三面 ≤0.25°，北墙拟合 n=127。可能该段扫糊了，也可能拟合窗口混进了别的东西。不足以回退，但下次建图值得复核。
14. **扫描残差指标会被"滑移"欺骗**：`scan_residual` 是"到最近占据格的中位距离"，在长直墙走廊里沿墙滑动同样能压到 0.000（实测 `align` 给出过 0.55 m 的假修正）。它适合做**停车判据**（0.41 明显异常），**不适合做位姿仲裁**。

## 10. 相机链路与 CPU（2026-10-31）

### 相机改成按需

`camera_node.py` 不再无条件开设备，按订阅者决定三档：

| 档 | 触发 | 设备 | 发布 |
| --- | --- | --- | --- |
| `idle` | 无订阅者 | **关闭**（`/dev/video0` 释放） | 无 |
| `view` | 只有 `/camera/image_raw/compressed` 订阅者（`camera_view`） | `view_width×view_height @ view_fps` | JPEG，`view_quality` |
| `full` | 有 `/camera/image_raw` 或 `/camera/camera_info` 订阅者（perception / capture） | `width×height @ fps` | bgr8 + CameraInfo |

新增 `/camera/active`（`std_msgs/Bool`，1 Hz，latch）：`hardware_status` 改看它，
**不再订阅图像**——原来那一路订阅让相机永远全速全分辨率跑。
`camera_view` 默认 `transport=compressed`（`~transport raw` 可回旧行为）。
`capture_cycle` 只在拍照那一小段订阅原图（`~camera_timeout` 5 s，拍完退订），
所以相机只在拍摄窗口内以 1280×720 打开。

实测（N100，同一套栈）：

```
idle   相机进程 <0.5%  CPU，/dev/video0 空闲
view   5.6%     640×360@10 JPEG q30，每帧 ~28 KB
full   27.8%    1280×720@30 bgr8（每帧 2.76 MB）
```

### 其它 CPU

- `cleanup_leftovers.py`（`aic26_utils`，`backend.launch` 里 `cleanup:=true` 自动跑）：
  杀两类残留——(a) 属于**别的 master**（端口不同/本地别名互换也算同一个）的 aic26 进程；
  (b) **同一个 master 但启动时间早于本次 roslaunch** 的同类进程。另加 `rosnode` 里
  `xxx_12345_67890123` 这类重复节点。`--kill-cpu N` 才按 CPU 比例杀，默认只打印 top10。
  实测：外部 master 的假进程被杀、当前栈节点全部存活；`hardware_delay:=2.0` 保证清理先于串口打开。
- costmap 发布频率：local `publish_frequency 20 → 2`、`always_send_full_costmap false`；
  global `10 → 1`。只有 rviz 订阅这两个话题，TEB 直接读 costmap 对象，不受影响。
- rviz `Frame Rate 30 → 15`。

### 机器自身的两个大项（需要 sudo，未动）

```
gdm3 在 activating(start-pre)/gdm-wait-for-dr 之间反复重启，累积 60 个残留 session
→ gdm3 ~11% + dbus-daemon ~14% + logind/journald/polkit/accounts-daemon ~20%（合计约半核）
→ 且没有可用的 X：/tmp/.X11-unix 为空，rviz 报
  "could not connect to display mowen-JNE10:10.0"，用 use_rviz:=true 也起不来
```

处理办法（二选一，都要 root）：

```bash
sudo systemctl stop gdm          # 无显示器，纯 SSH/远程开发
sudo systemctl disable gdm
# 或者修好显示：journalctl -u gdm -b、检查显卡驱动/接显示器
```

### 本轮实车跑了两段的结论（未改代码）

run1/run2 用的是旧相机链路，定位部分是目前最好的一次：

```
start gate   map->odom 0.002 m / 0.0°，σ 0.021
σ            0.014–0.049（全程）
scan         residual 中位 0.000 / p91 0.050 / max 0.100，>0.10 记 0 次；scan_ok 全程 true
map->odom    峰值 0.222 m / +3.4°，末了 −0.1°（旧基线 2.6–5.2°）
进度         7/13 站（corner_1、light_1、station_1/2/3、station_3 二次）
```

两处失败都不是定位问题，是 **TEB 把 footprint 蹭进 LETHAL 格后彻底解不出轨迹**：

```
run1  (−0.385, 0.140)  在 y=0.16 / x=−0.42 两条线交点旁，footprint 压线
      → "trajectory is not feasible" ×4 → "Failed to get a plan" ×3
      → "Rotate recovery can't rotate in place ... Cost: -1.00"
run2  (1.256, −1.227)  右侧 x=1.35 起就是 LETHAL 带，footprint 右缘 1.414 压进去
      → 同样的 not feasible 刷屏，卡死 40 s 直到人工停车

## 11. 提速到 0.20 m/s 与状态更新频率（2026-10-31）

参数：`max_vel_x 0.15 → 0.20`、`max_vel_x_backwards 0.12`、`turn_speed 0.45`，其余不动。
（这台机器 `gdm` inactive、`/tmp/.X11-unix` 为空，没有 X server，rviz 起不来，这轮 `use_rviz:=false`。）

### 状态更新频率（1 s 窗口计数）

```
speed bin    amcl_hz  tf_chg/s  cmd_hz
0.02–0.08      0.89     0.89    10.26
0.08–0.12      1.53     1.53     8.13
0.12–0.16      3.00     3.00     5.50
0.16–0.20      3.25     3.25     7.79
>0.20          3.69     3.69     8.19
```

- `/odom` 恒定 30 Hz、`/scan` 10 Hz、`/localisation/scan_residual` 1 Hz，与速度无关。
- 静止时 `/amcl_pose` 是 **0 Hz**（当时 `update_min_d 0.05` / `update_min_a 0.07` 不触发，这是设计如此，不是丢定位；现值为 0.03 / 0.15）。
- `map->odom` 的 TF 修正次数与 `/amcl_pose` 完全同步 ⇒ **每 5 cm 一次修正**，0.20 m/s 时约 3.3–3.7 Hz（每 0.27–0.30 s）。
- TEB 在 0.20 m/s 时实际 7.8–8.2 Hz（标称 10 Hz）。整轮 536 次 `Control loop missed its desired rate` + 578 次 `Map update loop missed`，约 13% 的周期超时——这是 0.20 m/s 下最该盯的数。

### 定位质量

```
σ          0.013–0.056（中位 0.052；0.15 m/s 那轮中位 0.031）
residual   中位 0.05 / p91 0.10 / max 0.10，>0.10 记 0 次
map->odom  ≤0.348 m，dyaw −4.8…+4.1°，末态 +4.0°
```

0.20 m/s 下定位没有崩，只是 σ 整体比 0.15 m/s 高（粒子云更散），残差指标不变。

### 效果与新的卡死点

11/13 站、160 s；同段 `station_1→station_2` 21.7 s（0.15 m/s 那轮 24.8 s，−12%）。

新卡死点：`car_3 (1.10, −0.30) → corner_1 (1.10, 1.70)`，即**车队列那条窄道**（第一次真正跑这段）：

```
global costmap 横截面（y=0 附近）
x:  0.50…0.90 LETHAL ｜ 0.95–1.00 + ｜ 1.05–1.20 自由 ｜ 1.25–1.30 + ｜ 1.35…1.55 LETHAL
⇒ 两条 LETHAL 带之间只有 0.45 m，车宽 0.285 ⇒ 单侧 0.0825 m
⇒ 小于 min_obstacle_dist 0.10 ⇒ TEB 采样轨迹"全部不可行"，静默不发 /cmd_vel
```

表现与 run2 完全同型：TEB `trajectory is not feasible` → 120 s 超时 → `back_off 0.12 m` → 重试仍失败（`corner_1`、`start` 各失败一次），cycle 2 又冲同一段。
卡死时 `/cmd_vel` 掉到 1–2 Hz，**三路看门狗都不触发**（卡死判据要求 cmd 新鲜 <0.5 s）。
末态 AMCL 报 `(0.79, −0.40)`——已经落在 x≤0.90 的 LETHAL 带里，要么 AMCL 在那条窄道里横向滑了 0.3 m，要么车头真的探进了车缝；**需要人到现场看一眼**。

结论：0.20 m/s 本身没问题（定位、残差、超时都在可接受范围），卡死是**几何**问题。
可选：把 `car_1/2/3` 或整段路线挪出这条 0.45 m 车道（首选，按第 9 节第 12 条），
或把 `min_obstacle_dist` 压到 0.08 专门过这条道（会牺牲别的裕度，需实车验证）。

```

两次都是 `go_to()` 的 120 s `nav_timeout` → `back_off 0.12 m` 重试才可能脱困（run1 里还没走到就人停了）。
**三路看门狗都抓不到这个**：TEB 是"静默不发 /cmd_vel"，而卡死判据要求 `cmd_vel` 是新鲜的。
要不要给"有目标、无 cmd、无位移"补一条，见下一轮再定；先别改阈值。


## 12. 2026-11-01：一通完整跑通 + 之后的两次失败 + 当前参数

### 跑通过一次（run_20261101_170533）

```
13/13 站，168 s（17:05:57 → 17:08:45），三路看门狗一次没响
唯一一次重试：light_2（17:07:36 back_off 0.12 m 后成功）
start gate 0.000 m / 0.0°，σ 0.020
σ        0.014–0.048，全程 σ>0.25 的样本 0 个
residual 逐段 0.00–0.10，车队列 car_1/2/3 段 0.00–0.05
map->odom ≤0.12 m、yaw ≤1.2°
窄道   行驶覆盖 36/38 s；前进 −13%、侧移 −33%；/cmd_vel 最长断流 0.30 s，
       TEB 静默 >0.8 s 的样本 0（上一轮是 36.6 s）
延迟   控制周期中位 0.182 / p90 0.268 / 最坏 0.577 s（目标 0.100）
```

那一轮生效的关键改动：`min_obstacle_dist 0.10 → 0.07`、`inflation_dist 0.20 → 0.12`、
`update_min_d/a 0.05/0.07 → 0.03/0.05`、`max_runtime_sigma 0.25 → 0.65`。

### 同一段（car_3 北侧约 0.4 m）的两轮失败与跑通

```
run 162914  车推不动：cmd_vy 0.14，odom_vy 只有 0.03，14 s 只走 0.36 m；
            residual 0.05 → 0.20，看门狗 10 s 停车。（当时地面还有凸起）
run 165439  残差修好了（0.00–0.05），但 σ 呼吸：0.02 → 0.55 → 0.02，
            σ 看门狗（0.25）在 car_3 前拦下；事后 σ 0.02→1.54 并跳到 3.6 m 外。
run 170533  上面两条都改掉后跑通。
```

结论：这一段出问题时会连续踩三个坑——(1) 轮子被地面顶住推不动；(2) 更新变快后
σ 呼吸；(3) 被看门狗按住十几秒后 AMCL 会全局重定位，位姿直接废掉，必须回起点重起。

### 当前参数（2026-11-01 末，下一次跑的基线）

> ⚠️ 这一段是 17:54 前后的参数，**不是最终值**；最终值见 13.1（`weight_inflation 3.5`、
> `weight_viapoint 50`、global `inflation_radius 0.26 / cost_scaling 3.5`）。

```
速度        max_vel_x 0.24 / backwards 0.15 / vy 0.14 / theta 0.60
            acc 0.6 / 0.6 / 0.8        turn_speed 0.50
TEB         min_obstacle_dist 0.08   inflation_dist 0.16   weight_obstacle 25
            weight_inflation 4.0    weight_viapoint 80    viapoint_sep 0.3
            lookahead 0.8  dt_ref 0.3  samples ≤200  inner/outer 6/4
            xy_goal 0.08  yaw_goal 0.12
costmap     local  inflation_radius 0.18  cost_scaling 5.0   (update 10 / publish 2)
            global inflation_radius 0.28  cost_scaling 5.0   (update 5  / publish 1)
AMCL        update_min_d 0.03  update_min_a 0.05  beams 90  particles 500–2000
            sigma_hit 0.20  z_hit 0.50  alpha1/2 0.40  alpha4 0.30
看门狗      σ 0.65（1 s）  残差 0.18（2 s）  卡死 0.08 / 5 mm / 0.02 rad（3 s）
底盘        feedback 20 Hz  publish 30 Hz  use_imu_yaw  strafe_gate 0.02
相机        按需：idle 关设备 / view 640×360@10 JPEG q30 / full 1280×720@30
```

### 为什么把 inflation 抬到 0.18 / 0.28

车体内切半径 0.1425。`inflation_radius` 小于它时，costmap 不会在"footprint 会碰到"
的格子上打 INSCRIBED(253)，TEB 就看不见车体有多宽——表现为反复压线，压进去之后
footprint 落进 LETHAL，TEB 直接判"不可行"并静默。0.18（≥0.1425+半格）才补上这一环；
`cost_scaling 8 → 5` 让代价衰减更慢，窄道里会把车往中间推。

### 还没验证

新参数（inflation 0.18/0.28、min_obstacle 0.08、AMCL 90 束/500–2000 粒子）**一次都没跑过**。
风险点：0.45 m 窄道里车中心可用窗口只有 ±0.0825，而 `min_obstacle_dist 0.08` 几乎贴着
这个上限，可能又回到"不可行"的静默状态。跑之前要先看 TEB 是否还发 /cmd_vel。

### 2026-11-01 17:28 那一轮：station_3 → traffic_light_2 切内角后 TEB 死锁

现象：车从 station_3 往 light_2 走时走了一条"短弧线"，贴着 y=0.16 / x=−0.42 两条线的
交点过去，然后 TEB 报 `trajectory is not feasible` + `Rotate recovery ... potential
collision, cost: -1.00`，move_base ABORTED，之后 car_1 也失败，最后 AMCL 崩（σ 1.09）。

数据（segment.csv）：

```
t=103  (−0.738, 0.370)  yaw −28°  cmd(0.240, 0.140)
t=104  (−0.573, 0.297)  yaw −49.5°  cmd 0     ← 卡死点
交点    (−0.415, 0.160)
中心到交点 0.209 m，footprint 边缘到交点只剩 0.049 m（没有真压上，只差 5 cm）
```

原因不是"墙线没起作用"——LETHAL 格一直挡着规划穿线。真正的原因是 **TEB 把内角切得太狠**：
`weight_inflation` 只有 1.0（默认），0.18 m 的膨胀代价在优化目标里几乎没分量，于是它只
守 `min_obstacle_dist 0.08`，把车带到离 LETHAL 只剩 5 cm 的位置；footprint 一碰到 LETHAL，
TEB 所有轨迹（包括原地转）都判碰撞，永久死锁。

对应改动：`weight_inflation 1.0 → 4.0`、`weight_viapoint 50 → 80`、
`max_global_plan_lookahead_dist 1.2 → 0.8`（不看弯道另一侧就不会切角）。

### 2026-11-01 17:54 那一轮：weight_inflation 2.5 的对照

```
2.5 + lookahead 0.8 + AMCL recovery 关闭：
  corner_1 → light_1 → station_1 → station_2（无重试，西走廊干净）→ station_3 ×2 → light_2
  → car_1 失败：车停在 (0.025, −1.069)，离 x=0.22 那条线的 LETHAL 只剩 2.7 cm，
    /cmd_vel 33 s 无输出（TEB 静默），最后 back_off 才脱身
```

对照 4.0 那轮（17:39）：西走廊 78 次"不可行"+1 次重试，但**整圈跑完**、没有任何贴墙死锁。
结论：`weight_inflation` **用 4.0**，西走廊的抖动属于可恢复代价；2.5 会贴墙并在窄处死锁。

顺带把 `nav_timeout 120 → 60 s`（demo/capture/waypoints；`mission.launch` 设 0.0 = 不超时）：TEB 静默时 60 s 就放弃目标、
`back_off 0.12 m` 重试，不用干等两分钟。AMCL 的 `recovery_alpha_slow/fast` 已置 0，
被按住的十几秒不会再"撒粒子跳 3.6 m"（还没在实车上验证过效果）。

### 2026-11-01 21:19 那一轮：最好的一圈 + 第 2 圈航向误差

```
weight_inflation 3.5 / global inflation 0.26 / nav_timeout 60 s / sigma_hit 0.20
第 1 圈 21:20:37 → 21:23:14 = 157 s，13/13，零重试   ← 目前最好
        σ max 0.050，residual max 0.100，超过看门狗阈值的样本 0
第 2 圈 21:23:19 起，21:23:43 在 light_1 → station_1（北侧走廊）挂了：
        车 (−0.738, 1.678)，σ 只有 0.031，但 residual 0.25 → 0.62
```

关键观测：`map->odom` 的 yaw 修正量

```
t=20~60   0.1~0.2°      （起点，正常）
t=70     −9.2°          ← station_1 那次原地转之后突然出现一个 ~9° 的修正
t=80~130 −5.7 → −4.8°   （之后基本不变）
第 2 圈  −7.8°，但 residual 已经 0.25~0.62
```

即：odom 的航向在"原地转 + 侧移"阶段攒了 5~9° 的误差，AMCL 在第 1 圈还能把它拽回来
（残差 ≤0.10），第 2 圈拽不动了，于是残差一路涨到 0.6 并被看门狗拦下。这不是粒子云发散
（σ 只有 0.03），是**系统性航向误差 + 似然太平**（`sigma_hit 0.2 / z_hit 0.5` 时 10° 的
偏差在 2 m 处只让似然降 20%，滤波器懒得纠正）。

下一步（保守）：`sigma_hit 0.20 → 0.18`、`z_hit 0.50 → 0.55`，让 5–10° 的航向误差在似然上
看得出来，同时离 HANDOVER 警告过的 0.08/0.8 还差得远。

### 现场坑

- `/dev/video0` 在 2026-11-01 傍晚从 USB 总线掉了（相机节点会自动重连，导航不受影响）。
- 地面油布摩擦小，侧移打滑 −33% 是物理问题，参数层面只能靠"别让它一直压着墙磨"。
- rviz 需要 X：`gdm` 现在是 inactive，但 `mowen-JNE10:10.0` 仍然可用
  （`~/.Xauthority` 里有 :10 的 cookie），`xdpyinfo` 能通；rviz 起不来先查这个。


## 13. 2026-11-01 收尾记录（重要：这一节是下次上车的唯一依据）

> **一句话现状**：参数调好后**一圈能稳定跑完**（4 次 13/13，157/168/182/187 s），但**位姿会在一圈里
> 漂 0.2–0.5 m**（主要发生在原地旋转和侧移段），第二圈以前基本必挂（2026-11-02 21:45 no-strafe
> 连跑两整圈，见 13.8.4）；2026-11-01 晚还因此撞过 traffic_light_2 的灯杆。已加两道加固
> （旋转期减少 AMCL 更新、map->odom 跳变检测），**两项都已在 22:39/22:50 的实车上触发（见 13.8.1），
> 但只能停车，不能恢复**。

### 13.1 最终参数（全部都在实车上跑过；★ 两项的实车记录见 13.8.1）

> ★ `update_min_a 0.15` 与 ★ `map->odom` 跳变看门狗是 2026-11-01 夜新增；22:39 / 22:50 两轮已在实车触发，`params_used.txt` 确认两项参数都已生效（见 13.8.1）。

```
速度      max_vel_x 0.24   backwards 0.15   vy 0.14   theta 0.60
          acc 0.6 / 0.6 / 0.8       turn_speed 0.50（原地转）
TEB       min_obstacle_dist 0.08   inflation_dist 0.16
          weight_obstacle 25   weight_inflation 3.5   weight_viapoint 50
          viapoint_sep 0.3   max_global_plan_lookahead_dist 0.8
          dt_ref 0.3  max_samples 200  inner/outer 6/4
          xy_goal_tol 0.08  yaw_goal_tol 0.12  feasibility_check_no_poses 0
costmap   local  inflation_radius 0.18  cost_scaling 5.0  (update 10 / publish 2)
          global inflation_radius 0.26  cost_scaling 3.5  (update 5  / publish 1)
AMCL      update_min_d 0.03  update_min_a 0.15   beams 90   particles 500–2000
          sigma_hit 0.18  z_hit 0.55  alpha1/2 0.40  alpha4 0.30
          recovery_alpha_slow/fast = 0（关掉"撒随机粒子"式重定位）
          初始 (1.6, 1.7, π)，initial_cov 0.0004
看门狗    σ > 0.65（1 s）   residual > 0.18（2 s）   nav_timeout 60 s（mission 0 = 不限）
          map->odom 单步 > 0.10 m 或 > 0.06 rad（5 Hz 采样，>0.6 s 间隔不判）
          卡死 cmd>0.08 & 位移<5 mm & 转角<0.02（3 s）
底盘      feedback 20 Hz  publish 30 Hz  use_imu_yaw  imu_gyro_gain 0.8
          imu_strafe_gate 0.02  linear/angular_deadband 0.002
相机      idle 关设备 / view 640×360@10 JPEG q30 / full 1280×720@30（按订阅者切换）
```

### 13.2 跑过的每一轮（同一套路线，13 站）

| 时间 | 关键变量 | 结果 |
|---|---|---|
| 17:05 | 0.24/0.14，min_obst 0.07，σ门 0.25，infl 0.12 | **整圈 168 s**，无重试 |
| 17:27 | infl 0.18/0.28 + weight_infl 1.0 | station_3→light_2 切内角到 4.9 cm，TEB 死锁 |
| 17:39 | 同 + weight_infl 4.0 + lookahead 0.8 | **整圈 187 s**（西走廊 78 次"不可行"+1 重试） |
| 17:54 | weight_infl 2.5 | 西走廊干净，但 car_1 前贴 x=0.22 线剩 2.7 cm → 死锁 |
| 21:19 | weight_infl 3.5，global infl 0.26，nav_timeout 60 | **整圈 157 s，零重试**（最好）；第 2 圈北侧走廊 residual 0.62 |
| 21:35 | 再 + global cost_scaling 3.5 + sigma_hit 0.18/z_hit 0.55 | **整圈 182 s**（car_2 一次重试）；第 2 圈刚开始 residual 0.20 → 立即 lost |

结论：**现在的参数能稳定跑完一圈（4 次 13/13：157/168/182/187 s）**；跑第二圈基本必挂
（17:39、21:19、21:35 的第 2 圈都在开始后 1 分钟内被残差看门狗拦下）。

### 13.3 六类失败，各自的"指纹"（照着认就行）

```
1) TEB 静默死锁（最常见）
   "trajectory is not feasible" 连续刷屏 → "Rotate recovery ... cost: -1.00"
   → "Aborting because a valid control could not be found"
   原因：footprint 蹭进 LETHAL（贴墙线 3–5 cm），此后所有轨迹从非法位姿出发
   处置：等 nav_timeout（60 s）→ back_off 0.12 m → 重试；或人工退 20 cm

2) 残差看门狗（定位系统性偏）
   "scan to map residual 0.2x m for 2.0 s" → 取消目标、停车、退出路线
   指纹：σ 只有 0.02–0.05（粒子云没散），map->odom 的 yaw 常年 5–9°
   根因：原地转向 + 侧移让 odom 航向攒误差，似然太平导致 AMCL 不纠正

3) σ 看门狗（粒子云真的散了）
   "amcl sigma 0.29 m for more than 1 s"（阈值现已放宽到 0.65）
   指纹：窄道里 σ 呼吸到 0.3–0.6；超过 0.8 基本就是要炸

4) 被按住之后的 AMCL 崩溃（派生问题）
   看门狗停车后若十几秒不动，σ 会从 0.02 冲到 1.5，位姿跳 3–4 m
   ⇒ 已把 recovery_alpha 置 0 抑制；但一旦发生只能回起点重启

5) 全局规划不可达
   "Failed to get a plan" 或目标很快 ABORT；常见于车贴着墙/在窄道端头

6) 位姿整体滑移（最阴险，2026-11-01 撞灯杆就是它）
   原地旋转期间 AMCL 把整个位姿平移 0.1–0.2 m（位置在该姿态下不可观测）
   指纹：σ 只涨到 0.05、residual 只到 0.10 —— 三路旧看门狗全不响
   后果：下一段路径照着偏移的目标走 → 撞上地图里没有的东西（灯杆）
   ⇒ 已加 _check_odom_jump（车体位姿单步 >0.10 m 或航向 >3.4°；原地转里的位置跳先回滚，见 13.8.2）
   ⇒ 22:39 / 22:50 两轮实车触发（map->odom 0.32 m/10.0°、0.34 m/9.3°；车体实际 0.117/0.191 m，见 13.8.1）
   ⇒ 现场表现：车"以为"自己在 start，实际在 corner_1（差 0.5 m）
```

### 13.4 关键实测数字（用于判断"现在正常吗"）

```
状态更新率   /odom 30 Hz  /scan 10 Hz  residual 1 Hz
             /amcl_pose = 速度/0.03 m ≈ 8 Hz 巡航；静止时 0 Hz（正常，不是丢定位）
             map->odom 的 TF 与 /amcl_pose 同频
控制延迟     TEB 周期 中位 0.18 s / p90 0.27–0.35 / 最坏 0.6–1.1 s（标称 0.1）
             整圈约 400 次 "Control loop missed"、300 次 "Map update"
定位质量     正常：σ 0.012–0.05，residual 0.00–0.10，map->odom ≤0.12 m
             警惕：residual 0.15+ 或 map->odom yaw 超过 ±5°
侧移         cmd 0.123 → 实际 0.082（−33%，地面油布摩擦小，物理限制）
前进         cmd 0.171 → 实际 0.149（−13%）
CPU          整套栈空转 31%（单核，其中 lidar 8.4 / chassis 7.8 / move_base 3.1）
             相机 idle 时 0%、camera_view 5.6%、全分辨率 27.8%
```

### 13.5 操作规矩（别再踩）

1. **每次启动栈之前必须确认车在起点**（1.6, 1.7, 朝 −x）。不在起点启动 = odom 和 AMCL
   一起错，`start gate` 看不出来（它只看 map->odom，而两者一起错时这个值≈0）。
   上电前可用"残差"核对：起点扫描 residual 应 ≈0.05、σ≈0.02、map->odom≈0。
2. 看门狗一响就把车推回起点重启，不要原地重发目标。
3. rviz 需要 X：`xdpyinfo -display mowen-JNE10:10.0` 能通才能开；`gdm` 现在 inactive，
   `:10` 是外部远程桌面提供的，会掉，掉了 rviz 会 core dump（不影响跑车）。
4. 雷达"电机转但距离全 0"是硬件状态（本机遇到过两次）：断电重上电即可。
   验证：`python3 ~/aic26_logs/tools/lidar_probe.py`，3 s 应有 10000+ 点，0 点就是坏的。
5. `/dev/video0` 会从 USB 总线掉（不影响导航），拍照前 `v4l2-ctl --list-devices` 确认。
6. 雷达/底盘/IMU 的 udev 按 USB 物理口绑定，换口会让 `carserial`/`imu` 互换。

### 13.6 工具（`~/aic26_logs/tools/`，不在仓库里；`cleanup_leftovers.py` 例外，在 `aic26_utils/scripts/`）

```
run_cycle.sh        一键：停旧栈 → 起 demo（有 X 就自动开 rviz）→ 起 10 Hz 记录 → dump 参数
segment_monitor.py  10 Hz 记录 cmd / cmd_age / odom / amcl / sigma / map->odom / residual / 目标
analyse_lane.py     窄道逐秒：命令 vs 实际速度、打滑比、cmd 断流
analyse_segment.py  按目标分段汇总
sigma_stats.py      按目标统计 σ / residual 分布
loop_report.sh      掉帧与延迟统计
cpu_probe.py        进程级 CPU
lidar_probe.py      雷达原始抓包 + 解析自检（判断"转但不测距"）
pose_check.sh       只起定位、不驱动，核对起点（残差/σ/map->odom）
```

### 13.7 还开着的口子（按优先级）

1. **航向/位姿漂移**（第一优先级）：一圈攒 5–9° 航向 + 最多 0.5 m 位置，是"压线、撞杆、
   残差报警"的共同源头。旋转时的整体滑移已由 `_check_odom_jump` 兜底（只止血不治本）；
   治本要动底盘航向源或路线，一次只改一个：
   - `imu_strafe_gate 0.02 → 0`（横移时也用陀螺航向，而不是轮速）；
   - `car_1/2/3` 的朝向 0° → +90°（窄道里直行代替侧移，顺带解决 −33% 打滑和侧移航向漂移）。
   - **（已落地为试验模式，未实车跑）** `disable_strafe:=true`：`navigation.launch` 把 TEB 的
     `max_vel_y` / `acc_lim_y` 置 0、`weight_kinematics_nh` 置 1000，`chassis.launch` 把
     `lateral_cmd_scale` 强制 0（连 teleop 的横移也丢掉），decision 四个 launch 改选
     `*_nostrafe_*` 的 waypoint/task/route。`real_nostrafe_waypoints.yaml` 只把
     **`car_2` / `car_3` 的 yaw 改成 +90°**（`car_1` 仍是 0°），并在对应的两个拍照点加了
     `turn: "right"`。`run_cycle.sh` 还没有带这个参数，要手动加。
2. **`traffic_light_1/2` 航点几乎压在灯杆上**（杆子不在图里、位置还会 ±0.3 m 漂）：
   建议把这两个航点平移 0.4 m 左右、再用 `turn: left/right` 转向看灯，这样即使位姿偏 0.2 m
   也不会撞杆。**需要现场确认杆子的实际位置后再改 yaml。**
3. **第二圈必挂**：demo 是无限循环，跑完一圈停反而稳；要连跑多圈先解决 1。
4. **TEB 单次迭代 0.18 s**（标称 10 Hz 实际 ~5.5 Hz）：`no_inner/outer_iterations` 6/4 → 4/3
   是上游默认值，能省约 1/3，但没验证过路径质量。
5. `station_1` 的 x 与仿真差 0.60 m（第 9 节第 9 条），始终没人现场确认。

### 13.8 2026-11-01 晚：撞灯杆复盘 + 后两项加固

现场看到：去 `traffic_light_2` 时车左偏、撞上灯杆左腿，之后整条路径都歪，最后停在
`corner_1` 却"以为"自己在 `start`。**`/amcl_pose`、`residual`、`σ` 全都正常**——它们是
"机器人自己以为的"，不是真值，这一课要记牢。

数据（run_20261101_213559）：

```
t=92–96   原地转（station_3，w=0.50），(−0.97, 0.53)，residual 0.00
t=97.2    位姿跳 (−0.98,0.52) → (−0.78,0.52) = 0.20 m，纯旋转期间
          map->odom (0.10,0.10,1.6°) → (0.27,0.04,−1.7°)
          σ 只 0.015→0.048，residual 只 0.05→0.10 —— 三路看门狗全不响
t=99–105  从错误位姿出发往 southeast 走 → 撞灯杆（灯杆不在图里，规划器不知道）
t=128+    顶住杆子、轮子空转 → odom 漂 → AMCL 跟着漂 → 末态偏 ~0.5 m
```

根因：**原地旋转时位置不可观测，AMCL 的粒子云可以整体滑动**；滑动后的位姿被用来规划
下一段 → 车照着偏移 0.2 m 的目标开 → 撞上地图里不存在的灯杆 → 撞了没人知道。

已做的两项加固（22:39 / 22:50 已在实车触发，记录见 13.8.1）：

1. `update_min_a 0.05 → 0.15`（5 个 launch）——一次 90° 原地转的 AMCL 更新次数从 ~30 降到 ~10，
   滑位机会随之变少。
2. `mission_common.RobotDriver._check_odom_jump()`（新增，5 Hz）：
   `map->odom` 单步变化 > `max_odom_jump`(0.10 m) 或 > `max_odom_yaw_jump`(0.06 rad ≈ 3.4°)
   就 `logerr` + `_hold()`（取消目标、停车、退出路线）；`odom_jump_max_gap`(0.6 s) 内的
   采样才比较，避免 TF 中间断过造成误判；`odom_jump_hold:=false` 可只记录不停车。
   逻辑用假 listener 跑过：首帧不判、2 cm 不动、20 cm 触发、8° 触发、间隔 2 s 跳过。

灯杆本身仍不在地图里：用户确认"红绿灯位置可能有 ±0.3 m 的平移，杆太细，pgm 里有记录但对
lidar 几乎是空的"。**所以不能靠地图标杆子解决**，只能靠第 2 条（跳变检测）在偏的瞬间就刹住，
再配合"撞了之后必须回起点重启"。

### 13.8.1 22:39 / 22:48 / 22:50 三轮：一次 start gate 拦截 + 两次跳变停车

三轮的 `params_used.txt` 都是 13.1 那套（`update_min_a 0.15`、`sigma_hit 0.18`、`z_hit 0.55`、
`min_obstacle_dist 0.08`、`max_runtime_sigma 0.65`），所以这是**新参数的第一次实车检验**。

```
run_20261101_223933（22:39）
  start gate 0.004 m / 0.1°，σ 0.020；corner_1 → light_1 → station_1
  station_1 TEB "trajectory is not feasible" 刷了约 18 s → ABORTED → back_off 0.12 m
  22:40:45.51  localisation_check "residual 0.100 m, refining the pose"（auto_align）
  22:40:45.65  自动修正 "offset 0.16 m / -2.3 deg, residual 0.100 -> 0.000"
  22:40:45.79  map->odom 跳 0.32 m / 10.0° → _check_odom_jump 停车
  22:40:48.21  "localisation lost, aborting the route"

run_20261101_224840（22:48）
  start gate 拒绝启动：map->odom = 4.50 m / 148.8°，σ 0.020，一直 waiting → 人工退出
  （车不在起点就起栈的现场样本；闸门 1 按设计挡住了，没有发出任何目标）

run_20261101_225031（22:50）
  start gate 0.005 m / 0.1°；corner_1 → light_1 → station_1
  station_1 原地转（w=0.50）途中，22:51:22.67 map->odom 跳 0.34 m / 9.3°
  → _check_odom_jump 停车 → "localisation not trusted, not rotating"
  → "could not restore heading at station_1, holding" → "localisation lost, aborting the route"
  这一轮没有 auto_align 修正日志，是纯旋转期间的位姿滑移
```

边界要写清楚：

- `_check_odom_jump` 已在实车触发两次，都及时停车并退出路线；实测 0.32–0.34 m / 9.3–10.0°
  离阈值 0.10 m / 0.06 rad 很远，不是边缘误报。
- `update_min_a 0.15` 生效了，但**没有阻止滑移**：滑移发生在原地转的过程中，不是在两次
  AMCL 更新之间。
- 22:39 那轮的滑移与 `localisation_check` 的 `/initialpose` 修正只隔 0.14 s（修正量
  0.16 m / −2.3°，观测到的跳变 0.32 m / 10.0°）。**时间紧邻，但不能据此断定因果**。
  这是个默认开启、全程只触发一次（第一次 `residual > 0.10 m`）的自动修正；如果怀疑它，
  用 `auto_align:=false` 做对照，不要直接改算法。
- 两次行驶都在 station_1 附近被拦下（22:48 那轮根本没起步）：西走廊入口 / 原地转这一带是滑移高发点，下一轮实验优先放这里。
- 停车之后没有恢复路径：当时只有"推回起点、重启栈"这一条（见 13.9）；13.8.2 已给原地转工况加了回滚。

### 13.8.2 2026-11-02 修复：跳变看门狗改看车体位姿，原地转只回滚位置

原判据直接比 `map->odom`。它的平移 = “odom 原点到车”的力臂 × 航向修正：station_1 离 odom 原点约
2 m，9.3° 的航向修正本身就会让 `map->odom` 平移变 0.34 m，于是正常的航向修正被当成滑移。
用 `segment.csv` 把两轮拆开，车体（`map->base`）实际只跳了 **0.117 / 0.191 m**，航向 9.2 / 10.0°。

改动（`aic26_decision/scripts/mission_common.py`）：

- 位置用车体 `map->base`（`map_pose(mod, odom)`），> `0.10 m` 才算位置滑移；航向仍用 `map->odom`
  的 yaw 变化（相对 odom 的修正量，正常转向时它≈0）。
- 位置没动、只是航向修正：≤ `max_odom_yaw_accept`（0.35 rad ≈ 20°）一律接受并记一条 warn，
  不管当时在不在转（纯航向不动位置，TEB 会按新航向重规划）；> 20°（翻转/重定位）才按故障处理。
- 位置真的跳了（> 0.10 m）、正在原地转、且航向修正 ≤ 20°：平移在物理上不可能，保留跳变前的位置、
  接受新航向，发 `/initialpose` 重新播种 AMCL；`odom_jump_settle`（0.4 s）后核对位置，没被接受才
  `_hold()`。每轮最多回滚 `odom_jump_recover_limit`（3）次。
- 不在原地转、扫描残差不 ok、σ 超限、或超过回滚次数 → 仍是原来的“停车 + 退路线”。
- `odom_jump_recover:=false` 退回旧行为；`odom_jump_settle` / `odom_jump_recover_limit` /
  `odom_jump_turn_grace` / `max_odom_yaw_accept` 可调。
- `navigation.launch` 的 `auto_align` 默认改成 **false**（任务模式不再在途中自动改位姿）；
  `localise.launch` 仍是 true。

验证：`py_compile` + 假 listener 覆盖 11 种情况（2 m 力臂的 9° 修正不误判、9° 纯航向修正行驶中接受、
180° 翻转 hold、正常转向不触发、0.19 m 真滑移触发、非旋转时仍 hold、回滚被接受 / 被拒绝、次数上限、
转停后 0.4 s 的更新回滚、转完开始前进后仍 hold）全部通过。

### 13.8.3 2026-11-02 21:35 第一次实车：过了 station_1，但被航向判据误停

栈：`demo.launch backend:=real show_view:=false use_rviz:=true disable_strafe:=true`（rviz 通，
相机 idle，无侧移）。结果：

- `corner_1 → traffic_light_1 → station_1` 全部正常；**station_1 的原地转完整走完，没有停车**，
  随后开始去 `station_2`。这一轮 `map->odom` 的 yaw 全程 <1.5°，所以 station_1 那次 9–10° 修正
  没有复现，13.8.2 的回滚路径**没有被触发**。
- 去 `station_2` 的路上（t≈37 s）被误停：`robot pose moved 0.02 m / 3.9 deg`。原因是第一版把
  航向也取成了 `map->base` 的 yaw，TEB 行驶中转向 3.9°/0.2 s 就被当成跳变；最终判据是
  **位置用车体、航向用 `map->odom` yaw，且 ≤20° 的纯航向修正一律接受**（见本节上面的规则）。
- 之后人工撤车时（t≈140 s 起）AMCL 跳到 79°/29°、σ 0.21、residual 0.25 —— 与撤车时间吻合，
  是“车在栈运行时被移动”，**不要拿这段当定位结论**。规矩仍是：栈运行期间不要搬车。

下一轮要验证的是：13.8.2 的回滚在真正出现 0.1–0.2 m 位置跳时能不能接住并继续。

### 13.8.4 2026-11-02 21:45 复跑：两整圈，看门狗零触发

同一套栈（real / no view / rviz / no strafe），车在起点。结果：

- 第 1 圈、第 2 圈都完整跑完（13 站/圈）；第 3 圈过 `station_1`、去 `station_2` 途中人工停车
  （`stop_robot.py`，不无限连圈）。
- `robot pose moved` 0 次、`map -> odom jump` 0 次、`not feasible` 0 次。
  唯一一条 `station_2 did not succeed` 是 `stop_robot.py` 取消目标造成的，不是失败重试。
- σ 最大 0.050，residual 最大 0.100，`scan_ok` 全程 true；无重试、无停车。
- `map->odom` yaw 从 +1.9° 漂到 −10.8°（约 5°/圈），位置仍跟得住。这是原来“每圈攒 5–9°”的
  老问题，本次改动**没有治它**，只是不再误杀。
- 这轮 `station_1` 的 9–10° 单步修正没有复现，所以 13.8.2 的回滚分支**仍未在实车触发**；
  但三类误杀（力臂放大、正常转向、行驶中 ≤20° 航向修正）都没有再出现。

日志：`/tmp/aic26_run_20261102_214553/`（`segment.csv` + `roslaunch.log`）。

### 13.9 下次上车的动作清单

```
1. 车放回起点 (1.6, 1.7, 朝 −x)；用 pose_check.sh 核对：residual≈0.05、σ≈0.02、map->odom≈0
2. rosrun aic26_utils cleanup_leftovers.py --dry-run   # 确认没有别的 master 的残留
3. ~/aic26_logs/tools/run_cycle.sh                     # 自动起栈(+rviz)、起记录、dump 参数
4. 重点盯四条日志：
   "map -> odom jump"        ← 停车（回滚失败 / 不在原地转 / 超过 3 次）
   "robot pose moved"        ← 原地转里的位置跳已回滚，继续观察后面有没有再跳
   "scan to map residual"    ← 定位系统性偏
   "trajectory is not feasible" 连续刷屏 ← 贴墙死锁
5. 任何一条出现 → 停车、车推回起点、重启栈；不要在原地重发目标
```

`update_min_a 0.15` 与跳变看门狗已在 22:39 / 22:50 实车触发（见 13.8.1）。13.8.2 之后，原地转里的
位置跳会先回滚（日志 `robot pose moved ... re-seeding`），回滚失败或不在转才停车。第一轮重点确认：
回滚后 `rotate_to` 能走完，且下一段 `go_to` 没有沿着错误位姿偏 0.1 m 以上。

## 14. 2026-11-02 文档校对记录（只核对，没有改代码）

核对方式：逐项读 `src/` 下的 launch / yaml / scripts；跑 `python3 src/aic26_slam/scripts/audit_map.py`；
读 `~/aic26_logs/run_20261101_223933`、`_224840`、`_225031` 的 `roslaunch.log` / `segment.csv` /
`params_used.txt`；四个整圈日志（170533 / 173949 / 211944 / 213559）用于核对 13.2。

已确认与文档一致：

- 13.1 的速度 / TEB / costmap / AMCL / 看门狗 / 底盘 / 相机数值，逐项等于 launch 与 yaml 的默认值。
- `world_real` 916 格、`world` 608 格、`world` 在场地内已知 70%（`audit_map.py` 输出）；
  `world_real_v2.pgm` 与 `world_real.pgm` 的 md5 相同。
- 13 个航点；real/sim 差：`station_1` dx −0.60，`car_1/2/3` dy +0.10/+0.20/+0.20；离障碍 0.50/0.85/0.60 m。
- `_check_odom_jump`、`nav_timeout`、`max_runtime_sigma`、`auto_align`、`disable_strafe` 的参数名与默认值（`auto_align` 已按 13.8.2 在任务模式下改成 false）。
- 13.8 的 0.20 m 跳变数据（run_20261101_213559, t=97.1）能在 `segment.csv` 里逐行对上。
- 22:39 / 22:50 的跳变看门狗记录，以及 22:48 的 start gate 拦截（13.8.1）。

已改正的原文（原文 → 实际）：

- IMU 第 5 条"两处都改 115200" → **9600**（与第 13 条一致）；
- `nav_timeout` 45 → **60 s**，并注明 `mission.launch` 是 **0**（不限时）；
- `car_1/2/3` "上移 0.10" → **+0.10 / +0.20 / +0.20**；
- `chassis: motors enabled at start-up` 这行日志**不存在**（只有 `/chassis/enable` 被调用时才打印）；
- `cleanup_leftovers.py` **已入库**（`aic26_utils/scripts/`，已 install），不再属于 `~/aic26_logs/tools/`；
- "3 次 13/13，157/168/182" → **4 次，157/168/182/187**；
- "TEB 裕度 0.10 未跑过整圈" → 现为 **0.08**，且已跑过整圈；
- "两处加固尚未实车验证" → 已在 **22:39 / 22:50** 触发（新增 13.8.1）；
- 新增 `disable_strafe` / `*_nostrafe` 的现状说明（13.7、第 8 节命令）。

仍未核实 / 需要现场确认：

- `station_1` 的 0.60 m 差是有意还是笔误（第 9 节第 9 条）。
- `traffic_light_1/2` 灯杆的实际位置（第 13.7 节第 2 条）。
- `world_real.pgm` 仍没有独立测量做交叉验证。
- “内部特征 151 vs 103”和四墙倾角没有现成脚本可复算（`audit_map.py` 统计场地内 occupied 是 155），保留原值。
- `disable_strafe` + `car_2/car_3` 朝 +90° 这套配置**一次都没跑过**。
- 13.8.2 的回滚还没有在**真实的 0.1–0.2 m 位置跳**上验证；21:45 复跑两整圈、看门狗零触发（13.8.4），
  但 `station_1` 的 9–10° 单步修正没有复现，回滚分支仍未实车触发。逻辑测试 11 项通过。
- `README.md` 说 nostrafe 把 `car_1..car_3` 都转成 +90°，实际只转了 `car_2/car_3`（README 待改）。

