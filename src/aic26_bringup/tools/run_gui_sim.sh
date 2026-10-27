#!/usr/bin/env bash
# Launch the Gazebo simulation together with RViz only. The camera_view
# window and the Tk result panel stay off so the car only transfers two X11
# clients (Gazebo GUI + RViz); the N100 has no usable GPU.
#
# Run this from a session with a valid DISPLAY:
#   - inside a NoMachine virtual desktop, or
#   - over SSH with X11 forwarding (ssh -Y), or
#   - on a monitor attached to the robot.

set -eo pipefail

if [ -z "${DISPLAY:-}" ]; then
    cat >&2 <<'EOF'
[ERROR] DISPLAY is empty, so the Gazebo GUI / RViz cannot open.

Choose one of:
  1) Windows: double-click windows/aic26_gui_sim.cmd (VcXsrv + ssh -Y).
  2) NoMachine: open a terminal inside the NoMachine desktop, then run this.
  3) Git Bash: export DISPLAY=localhost:0.0; ssh -Y vehicle ...

See tools/README_GUI.md for details.
EOF
    exit 1
fi

# Refuse to start a second full stack or a second Gazebo. /cmd_vel, TF and
# the Gazebo master (port 11345) would otherwise be shared with the old run.
if pgrep -f 'roslaunch .*aic26_(bringup|navigation|decision|gazebo)' >/dev/null; then
    echo "[ERROR] another aic26 stack is already running:" >&2
    pgrep -af 'roslaunch .*aic26_(bringup|navigation|decision|gazebo)' >&2 || true
    echo "Stop it first (Ctrl+C in its terminal, or: kill -INT <pid>)." >&2
    exit 1
fi
if ss -ltn 2>/dev/null | grep -q ':11345'; then
    echo "[ERROR] Gazebo is already listening on port 11345." >&2
    echo "Stop the old simulation before starting the sim GUI." >&2
    exit 1
fi
if pgrep -f 'rosmaster --core' >/dev/null; then
    echo "[WARN] a ROS master is already running; this launcher will join it." >&2
    echo "Stop it first if it belongs to another robot stack." >&2
fi

source /opt/ros/noetic/setup.bash
cd "$(dirname "$0")/../../.."
source devel/setup.bash
exec roslaunch aic26_bringup bringup.launch \
    mode:=mission backend:=sim \
    use_rviz:=true gui:=false show_view:=false
