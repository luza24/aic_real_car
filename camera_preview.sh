#!/usr/bin/env bash
# Start the AIC26 camera and show its live picture in a window.
#
#   ./camera_preview.sh                  # auto-detect DISPLAY (VcXsrv/NoMachine)
#   ./camera_preview.sh -d 192.168.1.189:0.0
#   ./camera_preview.sh -s 1.5           # 1.5x bigger window
#   ./camera_preview.sh -w 960 -H 540    # exact window size
#
# Close the window, press q/ESC in it, or Ctrl+C in this terminal to stop.
# The camera (and roscore) are started when needed and stopped on exit; a
# camera that is already running (e.g. inside a bringup) is reused untouched.

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TOPIC=/camera/image_raw
DEVICE=""
WIDTH=640
HEIGHT=360
SCALE=""
RATE=15.0
WINDOW_NAME="AIC26 camera"
DISPLAY_ARG=""
START_TIMEOUT=20

usage() {
    cat <<'EOF'
Usage: ./camera_preview.sh [options]

Show the AIC26 camera in an OpenCV window (aic26_utils/camera_view.py).

Options:
  -d, --display ADDR   X display to use, e.g. 192.168.1.189:0.0.
                       Default: $DISPLAY, else the SSH client IP + :0.0
                       (the automatically started VcXsrv on Windows).
  -t, --topic TOPIC    image topic (default /camera/image_raw)
      --device DEV     camera device for camera.launch (default /dev/video0)
  -w, --width PX       window width (default 640)
  -H, --height PX      window height (default 360)
  -s, --scale FACTOR   window scale multiplier, implies -w/-H * FACTOR
  -r, --rate HZ        refresh rate (default 15)
  -n, --name TITLE     window title (default "AIC26 camera")
      --timeout SEC    seconds to wait for roscore/camera (default 20)
  -h, --help           show this help

If the camera is not running this starts `roslaunch aic26_cobot camera.launch`.
If no ROS master is running either, a private roscore is started too; both
are stopped again when the window closes.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        -d|--display)  DISPLAY_ARG="${2:?--display needs a value}"; shift 2 ;;
        -t|--topic)    TOPIC="${2:?--topic needs a value}"; shift 2 ;;
        --device)      DEVICE="${2:?--device needs a value}"; shift 2 ;;
        -w|--width)    WIDTH="${2:?--width needs a value}"; shift 2 ;;
        -H|--height)   HEIGHT="${2:?--height needs a value}"; shift 2 ;;
        -s|--scale)    SCALE="${2:?--scale needs a value}"; shift 2 ;;
        -r|--rate)     RATE="${2:?--rate needs a value}"; shift 2 ;;
        -n|--name)     WINDOW_NAME="${2:?--name needs a value}"; shift 2 ;;
        --timeout)     START_TIMEOUT="${2:?--timeout needs a value}"; shift 2 ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

# `source` inherits our positional parameters when given no arguments, and
# setup.bash chokes on things like --help.  Clear them while sourcing.
set --

source /opt/ros/noetic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash"
fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

log() { echo "[preview] $*"; }

# ---------------------------------------------------------------- DISPLAY
# Same rule as aic26_bringup/bringup.launch: keep an existing DISPLAY
# (NoMachine, ssh -Y), otherwise assume VcXsrv on the SSH client's :0.
if [ -n "$DISPLAY_ARG" ]; then
    DISPLAY="$DISPLAY_ARG"
elif [ -z "${DISPLAY:-}" ]; then
    client="${SSH_CLIENT%% *}"
    [ -n "$client" ] || client="${SSH_CONNECTION%% *}"
    if [ -n "$client" ]; then
        DISPLAY="${client}:0.0"
    fi
fi

if [ -z "${DISPLAY:-}" ]; then
    cat >&2 <<'EOF'
[ERROR] No X display available (DISPLAY is empty).

Choose one:
  1) Windows: double-click the desktop start_vcxsrv.cmd, then run this
     script from the VS Code Remote-SSH terminal.
  2) Pass the display explicitly:
       ./camera_preview.sh -d <windows-ip>:0.0
  3) Open a NoMachine desktop and run this script inside it.
  4) Reconnect with X11 forwarding: ssh -Y vehicle, then run it.
EOF
    exit 1
fi
export DISPLAY
log "DISPLAY=$DISPLAY"

display_reachable() {
    local disp="$1" host num
    case "$disp" in
        *:[0-9]*) ;;
        *) return 0 ;;          # unusual form: let the viewer complain
    esac
    host="${disp%%:*}"
    num="${disp##*:}"
    num="${num%%.*}"
    if command -v xdpyinfo >/dev/null 2>&1; then
        DISPLAY="$disp" timeout 3 xdpyinfo >/dev/null 2>&1
    elif [ -z "$host" ] || [ "$host" = "unix" ]; then
        [ -S "/tmp/.X11-unix/X$num" ]
    else
        timeout 2 bash -c "exec 3<>/dev/tcp/$host/$((6000 + num))" 2>/dev/null
    fi
}

if ! display_reachable "$DISPLAY"; then
    cat >&2 <<EOF
[ERROR] Cannot open the X server at DISPLAY=$DISPLAY.

- Windows/VcXsrv: start it with "start_vcxsrv.cmd" (or the desktop
  aic26_gui_real.cmd) and make sure it uses -ac + "-listen tcp".  Its
  display must stay :0; VcXsrv may pick another one after a restart.
- Check that the car can reach this machine:  ping ${DISPLAY%%:*}
- Check firewall:  Windows must allow inbound TCP 6000 to vcxsrv.exe.
- NoMachine: run this script inside the NoMachine desktop instead.
- ssh -Y: run it from that shell (DISPLAY is set automatically).
EOF
    exit 1
fi

# ------------------------------------------------------- child processes
TMPDIR="$(mktemp -d /tmp/camera_preview.XXXXXX)" || exit 1
STARTED_MASTER=""
STARTED_CAMERA=""
VIEWER_PID=""

stop_group() {
    local pid="$1" name="$2" i
    [ -n "$pid" ] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    log "stopping $name"
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 20); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.5
    done
    log "$name ignored SIGINT, killing it"
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    sleep 0.5
}

cleanup() {
    # A second Ctrl+C / TERM must not kill us halfway through, otherwise the
    # roscore or camera we started would be orphaned.
    trap '' INT TERM HUP
    if [ -n "$VIEWER_PID" ] && kill -0 "$VIEWER_PID" 2>/dev/null; then
        kill -INT "$VIEWER_PID" 2>/dev/null
        local i
        for i in $(seq 1 10); do
            kill -0 "$VIEWER_PID" 2>/dev/null || break
            sleep 0.3
        done
        kill -KILL "$VIEWER_PID" 2>/dev/null
    fi
    VIEWER_PID=""
    stop_group "$STARTED_CAMERA" camera
    stop_group "$STARTED_MASTER" roscore
    rm -rf "$TMPDIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

master_up() { rostopic list >/dev/null 2>&1; }
topic_up() { rostopic list 2>/dev/null | grep -qxF "$TOPIC"; }

tail_log() {
    [ -f "$1" ] || return 0
    tail -n 15 "$1" 2>/dev/null
}

if master_up; then
    log "reusing running ROS master"
else
    log "starting roscore"
    setsid roscore >"$TMPDIR/roscore.log" 2>&1 </dev/null &
    STARTED_MASTER=$!
    for i in $(seq 1 "$START_TIMEOUT"); do
        master_up && break
        if ! kill -0 "$STARTED_MASTER" 2>/dev/null; then
            echo "[ERROR] roscore exited:" >&2
            tail_log "$TMPDIR/roscore.log" >&2
            exit 1
        fi
        sleep 1
    done
    if ! master_up; then
        echo "[ERROR] roscore did not come up in ${START_TIMEOUT}s:" >&2
        tail_log "$TMPDIR/roscore.log" >&2
        exit 1
    fi
    log "roscore is up"
fi

if topic_up; then
    log "reusing camera on $TOPIC"
else
    cam_args=()
    [ -n "$DEVICE" ] && cam_args+=("device:=$DEVICE")
    log "starting camera: roslaunch aic26_cobot camera.launch ${cam_args[*]:-}"
    setsid roslaunch aic26_cobot camera.launch "${cam_args[@]}" \
        >"$TMPDIR/camera.log" 2>&1 </dev/null &
    STARTED_CAMERA=$!
    for i in $(seq 1 "$START_TIMEOUT"); do
        topic_up && break
        if ! kill -0 "$STARTED_CAMERA" 2>/dev/null; then
            echo "[ERROR] camera launch exited:" >&2
            tail_log "$TMPDIR/camera.log" >&2
            exit 1
        fi
        sleep 1
    done
    if ! topic_up; then
        echo "[ERROR] no image on $TOPIC after ${START_TIMEOUT}s:" >&2
        tail_log "$TMPDIR/camera.log" >&2
        exit 1
    fi
    log "camera is up"
fi

# ------------------------------------------------------------- the window
if [ -n "$SCALE" ]; then
    export AIC26_GUI_SCALE="$SCALE"
fi

log "opening window '$WINDOW_NAME' (${WIDTH}x${HEIGHT}@${RATE}Hz); q/ESC or close it to stop"
roslaunch aic26_utils camera_view.launch \
    image:="$TOPIC" \
    width:="$WIDTH" \
    height:="$HEIGHT" \
    rate:="$RATE" \
    window_name:="$WINDOW_NAME" &
VIEWER_PID=$!
wait "$VIEWER_PID"
status=$?
VIEWER_PID=""
log "window closed"
exit "$status"
