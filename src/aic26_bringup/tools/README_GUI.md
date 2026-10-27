# aic26_ws_speech GUI 启动说明

## 0. 先看结论

- `roslaunch aic26_bringup bringup.launch mode:=mission backend:=real`
  **不会启动 Gazebo**，这是设计如此。Gazebo 只在 `backend:=sim` 下启动。
- 仿真默认只启动 **Gazebo GUI + RViz**（`use_rviz:=true`）；相机窗口
  （`show_view`）和检测结果面板（`gui`）在 `backend:=sim` 下默认关闭。
  实车默认仍是 RViz + 相机窗口 + 结果面板。小车当前没有显示器、`DISPLAY` 为空，
  不配 X 显示时这些窗口会启动失败/静默退出。
- 要看到界面，需要给 RViz 一个 X 显示：Windows 用 VcXsrv + `ssh -Y`，
  或者用车上已在运行的 NoMachine 虚拟桌面。

## 0.1 裸命令直接弹窗（2026-10-24 新增）

`aic26_bringup/launch/bringup.launch` 现在会自动设置 `DISPLAY`：

- 已有 `DISPLAY`（NoMachine / `ssh -Y`）→ 直接使用；
- 否则使用 `SSH_CLIENT` 里的客户端 IP + `:0.0`
  （也就是 Windows 上运行的 VcXsrv，TCP 6000）。

所以在 **VS Code Remote-SSH 终端**里直接运行：

```bash
roslaunch aic26_bringup bringup.launch mode:=mission backend:=sim
```

只要 Windows 上 VcXsrv 在运行，Gazebo / Gazebo GUI / RViz 就会直接弹出，
不需要额外 source 或改 `.bashrc`。需要相机窗口或结果面板时显式加
`show_view:=true gui:=true`。

前提：

1. **VcXsrv 必须在运行**。重连/重启 Windows 后先双击桌面
   `start_vcxsrv.cmd`（或手动运行 VcXsrv）。
2. **默认端口 11311 / 11345 必须空闲**。启动前先停掉旧的
   `roslaunch aic26_bringup ...` / headless demo：
   ```bash
   pgrep -af 'roslaunch aic26_bringup'
   # 在它原来的终端 Ctrl+C，或：
   kill -INT <PID>
   ```
3. 可以用 `gui_display:=192.168.1.189:0.0` 覆盖自动值，
   `gui_display:=` 关闭自动设置。
4. 直连 X11 使用 VcXsrv 的 `-ac`（无认证），仅建议在可信局域网使用。

> 旧 demo 不是本会话启动的进程，工具不会自动替你结束它；需要你手动停。

## 1. 现有 GUI 组件

| 参数 | 默认 | 界面 |
|---|---|---|
| `use_rviz` | true | RViz（地图、机器人、代价地图、路径） |
| `gui` | sim: false / real: true | Tk 检测结果面板（相机画面 + people/car/traffic_light） |
| `show_view` | sim: false / real: true | OpenCV 相机窗口（`camera_view`） |
| `day_night_gui` | false | Gazebo 昼夜滑块（仅 sim） |
| `backend:=sim` | sim | Gazebo + Gazebo GUI + RViz（默认两个窗口） |
| `backend:=real` | — | 实车硬件 + RViz + 相机 + 结果面板，**没有 Gazebo** |

可以按需开关：`use_rviz:=false`、`gui:=true`、`show_view:=true`。

仿真相机默认 **640×360@15**（`sim_camera_width` / `sim_camera_height` /
`sim_camera_rate`，在 `backend.launch` 里透传）。原因：这台 N100 没有可用 GPU
（无 `/dev/dri`，Gazebo 走 llvmpipe 软件渲染），1280×720@30 相机要吃掉近 2 个核，
实时率会掉到 ~0.56。`mode:=capture` 会强制回到 **1280×720@30** 采数据集，实车
相机参数不受影响。

## 2. 快速启动

### 2.1 Windows + VcXsrv（推荐，实车）

双击桌面上的：

- `aic26_gui_real.cmd` — 实车任务 + RViz + 相机 + 结果面板
- `aic26_gui_sim.cmd` — Gazebo 仿真 + RViz（默认只有两个窗口）

脚本会自动：

1. 如果 VcXsrv 没运行，用 `:0 -multiwindow -clipboard -wgl -ac -listen tcp` 启动；
2. 设置 `DISPLAY=localhost:0.0`、`XAUTHORITY`、VcXsrv 的 `xauth.exe` 到 PATH；
3. 用 `ssh -Y vehicle` 登录小车；
4. 在小车 `~/aic26_ws_speech` 里 source ROS/devel 并启动 bringup。

等几秒后，RViz（实车另外还有相机窗口和结果面板）会作为 Windows 窗口出现。

> 实车脚本不会启动 Gazebo；需要 Gazebo 请用 `aic26_gui_sim.cmd`。
>
> 两个脚本都会先检查车上是否已有
> `roslaunch aic26_(bringup|navigation|decision|gazebo)` 进程；如果有，
> 会打印 PID 并拒绝启动，避免第二个 bringup 共用 `/cmd_vel`、TF 和 master。
> 仿真脚本还会检查 Gazebo 的 11345 端口。先停掉旧任务再启动。

### 2.2 NoMachine 虚拟桌面

车上 NoMachine 服务已经在运行（`nxserver.service`），Windows 也装了
NoMachine 客户端：

1. 打开 NoMachine，新建连接：主机 `192.168.1.15`，用户 `mowen`；
2. 进入虚拟桌面后打开终端；
3. 运行：
   ```bash
   bash ~/aic26_ws_speech/src/aic26_bringup/tools/run_gui_real.sh
   # 或仿真：
   bash ~/aic26_ws_speech/src/aic26_bringup/tools/run_gui_sim.sh
   ```
4. 界面会出现在 NoMachine 桌面里。

NoMachine 虚拟桌面不依赖物理显示器，适合当前“小车没接屏幕”的情况。

### 2.3 Git Bash 手动方式

```bash
# 先启动 VcXsrv（如果还没运行）
"/c/Program Files/VcXsrv/vcxsrv.exe" :0 -multiwindow -clipboard -wgl -ac -listen tcp &

export PATH="/c/Program Files/VcXsrv:$PATH"
export XAUTHORITY="/c/Users/luza/.Xauthority"
export DISPLAY=localhost:0.0

ssh -Y vehicle "bash -lc 'source /opt/ros/noetic/setup.bash && cd ~/aic26_ws_speech && source devel/setup.bash && roslaunch aic26_bringup bringup.launch mode:=mission backend:=real use_rviz:=true gui:=true show_view:=true'"
```

## 3. 为什么 backend:=real 没有 Gazebo

`aic26_bringup/launch/backend.launch` 的选择逻辑：

- `backend:=sim` → include `aic26_gazebo/launch/sim.launch`（Gazebo + 仿真车）
- `backend:=real` → include `aic26_cobot/launch/hardware.launch`（底盘/IMU/激光/相机）

`backend:=real` 的节点列表里只有 `/chassis`、`/imu`、`/lidar`、`/camera`、
`/rviz`、`/camera_view`、`/mission_controller` 等，没有 `/gazebo`。

如果在实车模式下再手动启动 Gazebo，会出现两个机器人同时发布 TF/odom、
同时消费 `/cmd_vel`，导航会表现异常。decision 里的 `check_environment()`
也会检测到 `/chassis` + `/gazebo` 同时存在并拒绝行驶。

所以：

- **看实车 + RViz**：`backend:=real`，按本文配置显示。
- **看 Gazebo**：`backend:=sim`（仿真车会自己跑任务）。
- 不要在同一 ROS master 上同时跑 real 和 sim。

## 4. 故障排查

| 现象 | 原因/处理 |
|---|---|
| `another aic26 stack is already running` | 已有 bringup/nav/decision/gazebo 在跑。先 `pgrep -af 'roslaunch .*aic26_|rosmaster'` 找到 PID，Ctrl+C 或 `kill -INT <pid>` 停掉旧任务 |
| `Gazebo is already listening on port 11345` | 旧仿真占用了 Gazebo。停掉旧的 `gzserver`/bringup，或改用独立 `GAZEBO_MASTER_URI` |
| 命令后没有任何窗口 | 小车 `DISPLAY` 为空。用 VcXsrv + `ssh -Y` 或 NoMachine，见第 2 节 |
| `unable to open display` | X11 转发没建立；确认 VcXsrv 在运行、`DISPLAY=localhost:0.0`、`ssh -Y` |
| `No xauth data; using fake authentication data` | 通常只是警告。若随后 `wrong authentication`，确认 VcXsrv 带 `-ac`，并把 `C:\Program Files\VcXsrv` 加进 PATH 让 SSH 找到 `xauth.exe` |
| VcXsrv 已经在运行但没有窗口 | 关闭已有 VcXsrv，重新用脚本启动（确保带 `-ac -listen tcp`） |
| Gazebo GUI 很卡/黑屏 | VcXsrv 的 OpenGL 支持有限；仿真 Gazebo 建议用 NoMachine 虚拟桌面 |
| RViz 很卡 | X11 转发带宽/渲染限制；用 NoMachine，或降低 RViz 刷新/关闭部分 display |
| 界面太小/太大 | 设置 `AIC26_GUI_SCALE=1.5`（Tk 面板/相机窗口） |
| 只想看相机 | `use_rviz:=false gui:=false show_view:=true` |
| 只想看 RViz | `use_rviz:=true gui:=false show_view:=false` |
| 实车模式想停车 | 终端 Ctrl+C 结束 roslaunch；必要时 `rosrun aic26_cobot emergency_stop.py` |

## 5. 文件说明

- `tools/run_gui_real.sh`：车端实车 GUI 启动脚本（需有效 DISPLAY）
- `tools/run_gui_sim.sh`：车端仿真 GUI 启动脚本（需有效 DISPLAY）
- `tools/windows/aic26_gui_real.cmd`：Windows 一键实车 GUI（VcXsrv + ssh -Y）
- `tools/windows/aic26_gui_sim.cmd`：Windows 一键仿真 GUI（VcXsrv + ssh -Y）

## 6. 已验证情况（2026-10-24）

- Windows `cmd` + VcXsrv + `ssh -Y`：`xdpyinfo` 能连到 VcXsrv（vendor `HC-Consult`）。
- RViz 通过 `ssh -Y` 能稳定运行 15 s 以上，窗口可见；关闭时 RViz 可能打印
  `段错误`，属于退出阶段的问题，不影响使用。
- 无 `DISPLAY` 时，车端脚本会打印明确错误并退出，不会静默失败。
- 用独立端口（ROS 11419 / Gazebo 11346）完整验证了 `backend:=sim`：
  即使启动 shell 的 `DISPLAY` 为空，`bringup.launch` 自动注入后
  `gzserver`、`gzclient`、`rviz`、`camera_view`、`day_night_gui` 全部正常启动，
  启动日志无错误。
- 默认端口 11311/11345 仍被一个 headless demo 占用；停掉它之后，
  裸 `roslaunch ... backend:=sim` 即可直接在 Windows 上弹窗。
- 2026-10-25：仿真相机默认降为 640×360@15，GUI 精简为 Gazebo + RViz，
  `navigation.rviz` 帧率从默认 30 降到 10 FPS。
  实测原因：N100 的 i915 未绑定（无 `/dev/dri`），1280×720@30 相机由 llvmpipe
  在 CPU 上渲染，gzserver 占 ~230%（4 个 llvmpipe 线程），RTF ~0.56，
  `/camera/image_raw` 只有 ~2 Hz。改后 gzserver 约 75–85%，窗口只剩两个，
  但远程 X11 流量仍有 ~6 MB/s（gzclient 2.2 + RViz 4.2），2.4GHz WiFi + 省电
  仍是交互延迟的主要来源。
- Gazebo GUI 对 OpenGL 要求较高；如果 VcXsrv 下卡顿/黑屏，优先用
  NoMachine 虚拟桌面。

## 7. 安全提示

- `aic26_gui_real.cmd` 会启动实车底盘、激光、相机和任务节点；运行前确认车周围安全，
  并准备好 `emergency_stop.py` 或断电。
- 实车任务一旦启动，`mission_controller` 会按 `real_tasks.yaml` 发 goal 并行驶。
- 只验证界面时，可以先只启动导航（不启动任务）：
  ```bash
  roslaunch aic26_navigation navigation.launch use_rviz:=true
  ```
  或把 bringup 的 `mode:=waypoints` 配合 `task_delay` 调大，先只显示 RViz。
