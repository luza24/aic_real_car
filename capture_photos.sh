#!/usr/bin/env bash
# Capture photos from the AIC26 camera into <workspace>/photo/.
#
#   ./capture_photos.sh             # 20 photos, 0.5 s apart
#   ./capture_photos.sh 10          # 10 photos
#   ./capture_photos.sh -i 1.0 -o photo_test
#   ./capture_photos.sh --help
#
# The camera (and roscore) are started automatically when they are not
# running, and stopped again when this script is done with them.
# ROS's setup.bash reads unset variables, so no `set -u` here.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# `source` inherits our positional parameters when given no arguments, and
# setup.bash chokes on things like --help.  Clear them while sourcing.
ARGS=("$@")
set --

source /opt/ros/noetic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash"
fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

# /usr/bin/python3 keeps conda's interpreter (no cv_bridge) out of the way.
exec /usr/bin/python3 "$WS/capture_photos.py" "${ARGS[@]}"
