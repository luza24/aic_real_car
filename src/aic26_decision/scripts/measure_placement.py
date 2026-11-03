#!/usr/bin/env python3
"""Print the placement pose that cancels the tilt of a new gmapping run.

gmapping puts the robot at the origin of its own map frame, so the map is
rotated by (true heading - initial_pose_a). A hand-placed robot is a few
degrees off, and that error ends up baked into the map. Measuring the pose
against the already-realigned map and feeding it back as initial_pose_* makes
the new map axis aligned and co-located with the old one, so the waypoints stay
valid and no post-hoc rotation is needed.

    rosrun aic26_decision measure_placement.py --settle 12
"""

import argparse
import math

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--settle', type=float, default=12.0)
    parser.add_argument('--samples', type=int, default=20)
    args = parser.parse_args()

    rospy.init_node('measure_placement')
    pose = {}
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                     lambda m: pose.__setitem__('m', m))
    listener = tf.TransformListener()
    rospy.sleep(args.settle)

    samples = []
    deadline = rospy.Time.now() + rospy.Duration(20.0)
    while len(samples) < args.samples and rospy.Time.now() < deadline:
        msg = pose.get('m')
        if msg is not None:
            q = msg.pose.pose.orientation
            samples.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                            math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z)),
                            math.sqrt(max(msg.pose.covariance[0],
                                          msg.pose.covariance[7]))))
        rospy.sleep(0.5)

    if not samples:
        print('no /amcl_pose, is the localisation stack running?')
        return
    x = sum(s[0] for s in samples) / len(samples)
    y = sum(s[1] for s in samples) / len(samples)
    yaw = math.atan2(sum(math.sin(s[2]) for s in samples),
                     sum(math.cos(s[2]) for s in samples))
    sigma = max(s[3] for s in samples)
    spread = max(math.hypot(s[0] - x, s[1] - y) for s in samples)

    print('samples      %d   max sigma %.3f m   max spread %.3f m'
          % (len(samples), sigma, spread))
    print('pose in map  x %.3f  y %.3f  yaw %.1f deg'
          % (x, y, math.degrees(yaw)))
    print('')
    print('  initial_pose_x:=%.3f initial_pose_y:=%.3f initial_pose_a:=%.6f'
          % (x, y, yaw))
    if sigma > 0.10 or spread > 0.05:
        print('WARNING: the estimate is not settled, do not use these yet')
    try:
        translation, rotation = listener.lookupTransform('map', 'odom',
                                                         rospy.Time(0))
        offset = math.degrees(math.atan2(
            2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
            1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2])))
        print('map -> odom   %.3f, %.3f, %.2f deg  (= this placement error)'
              % (translation[0], translation[1], offset))
    except tf.Exception:
        pass


if __name__ == '__main__':
    main()
