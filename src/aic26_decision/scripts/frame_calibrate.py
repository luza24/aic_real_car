"""Measure the frame correction between the waypoint frame and the map.

Put the robot on the documented start pose, start the navigation stack, then
run this node. It reads where AMCL thinks the robot is and prints (optionally
writes) the ``frame_correction`` block of a waypoint file:

    p_map = R(yaw) * p_venue + (x, y)

    rosrun aic26_decision frame_calibrate.py
    rosrun aic26_decision frame_calibrate.py --write \
        $(rospack find aic26_decision)/config/real_waypoints.yaml
"""

import argparse
import math
import re
import sys

import rospy
import tf
import tf.transformations


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-x', type=float, default=1.6)
    parser.add_argument('--start-y', type=float, default=1.7)
    parser.add_argument('--start-yaw', type=float, default=3.141592)
    parser.add_argument('--global-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--settle', type=float, default=6.0)
    parser.add_argument('--write', default='')
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node('frame_calibrate')
    listener = tf.TransformListener()
    rospy.loginfo('waiting %.0f s for the localiser to settle', args.settle)
    rospy.sleep(args.settle)

    try:
        trans, rot = listener.lookupTransform(
            args.global_frame, args.base_frame, rospy.Time(0))
    except Exception as exc:
        rospy.logfatal('no %s -> %s transform: %s', args.global_frame,
                       args.base_frame, exc)
        return 1

    rotated = tf.transformations.euler_from_quaternion(rot)
    print('amcl pose  x %.3f  y %.3f  yaw %.2f deg'
          % (trans[0], trans[1], math.degrees(rotated[2])))

    dyaw = rotated[2] - args.start_yaw
    while dyaw > math.pi:
        dyaw -= 2.0 * math.pi
    while dyaw <= -math.pi:
        dyaw += 2.0 * math.pi
    cos, sin = math.cos(dyaw), math.sin(dyaw)
    dx = trans[0] - (cos * args.start_x - sin * args.start_y)
    dy = trans[1] - (sin * args.start_x + cos * args.start_y)

    print('frame_correction  x %.3f  y %.3f  yaw %.4f  (%.2f deg)'
          % (dx, dy, dyaw, math.degrees(dyaw)))
    print('residual offset %.3f m, waypoints move by up to %.2f m'
          % (math.hypot(dx, dy), math.hypot(dx, dy) + 2.5 * abs(dyaw)))

    if args.write:
        block = ('frame_correction:\n'
                 '  x: %.4f\n  y: %.4f\n  yaw: %.6f\n' % (dx, dy, dyaw))
        with open(args.write) as handle:
            text = handle.read()
        if re.search(r'^frame_correction:', text, re.M):
            text = re.sub(r'^frame_correction:\n(?:  .*\n)+', block, text,
                          count=1, flags=re.M)
        else:
            text = block + '\n' + text
        with open(args.write, 'w') as handle:
            handle.write(text)
        print('written to %s' % args.write)
    return 0


if __name__ == '__main__':
    sys.exit(main())
