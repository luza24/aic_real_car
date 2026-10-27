#!/usr/bin/env bash
# Launch the real-robot mission together with RViz, camera_view and the
# detection result panel.
#
# Run this from a session with a valid DISPLAY:
#   - inside a NoMachine virtual desktop, or
#   - over SSH with X11 forwarding (ssh -Y), or
#   - on a monitor attached to the robot.
#
# Gazebo is intentionally NOT started with backend:=real. Use run_gui_sim.sh
# for the simulation (Gazebo + RViz).

set -eo pipefail

if [ -z "${DISPLAY:-}" ]; then
    cat >&2 <<'EOF'
[ERROR] DISPLAY is empty, so RViz / camera / result GUI cannot open.

Choose one of:
  1) Windows: double-click windows/aic26_gui_real.cmd (VcXsrv + ssh -Y).
  2) NoMachine: open a terminal inside the NoMachine desktop, then run this.
  3) Git Bash: export DISPLAY=localhost:0.0; ssh -Y vehicle ...

See tools/README_GUI.md for details.
EOF
    exit 1
fi

# Refuse to start a second full stack. The existing headless demo or another
# navigation/mission launch would otherwise share /cmd_vel, TF and the ROS
# master with this bringup.
if pgrep -f 'roslaunch .*aic26_(bringup|navigation|decision|gazebo)' >/dev/null; then
    echo "[ERROR] another aic26 stack is already running:" >&2
    pgrep -af 'roslaunch .*aic26_(bringup|navigation|decision|gazebo)' >&2 || true
    echo "Stop it first (Ctrl+C in its terminal, or: kill -INT <pid>)." >&2
    echo "This launcher starts a full bringup; it must not join a stale stack." >&2
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
    mode:=mission backend:=real \
    use_rviz:=true gui:=true show_view:=true
