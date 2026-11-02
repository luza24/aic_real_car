#!/usr/bin/env python3
"""Build a static map for the simulation world from ground truth scans.

Teleports the robot over a grid of free poses, reads one laser scan at
each pose and accumulates endpoints (occupied) and rays (free). The
pose comes from /gazebo/model_states, so there is no odometry drift.

    python3 src/aic26_slam/scripts/build_map_from_sim.py --write
"""
import argparse
import math
import os
import re

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan

ROBOT = 'armbot'
RES = 0.05
ORIGIN = (-6.0, -6.0)
SIZE = (224, 224)
OCC, FREE, UNKNOWN = 0, 254, 205

# Nothing the laser can see may be closer than this to a teleport target,
# otherwise the robot would be dropped inside a board or a car.
KEEP_CLEAR = 0.40


def load_obstacles(world_path):
    """Return the poses of the visible models the robot must not spawn on."""
    text = open(world_path).read()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    poses = []
    for match in re.finditer(r'<include>(.*?)</include>', text, re.S):
        block = match.group(1)
        name = re.search(r'<name>(.*?)</name>', block)
        pose = re.search(r'<pose>(.*?)</pose>', block)
        if not name or not pose:
            continue
        name = name.group(1)
        if name in ('map_plane', 'map_walls'):
            continue                      # arena boundary handled separately
        x, y = (float(v) for v in pose.group(1).split()[:2])
        poses.append((name, x, y))
    return poses


def free_poses(obstacles, spacing):
    """Grid of teleport targets inside the arena that avoid every model."""
    poses = []
    limit = 2.05 - KEEP_CLEAR
    xy = np.arange(-limit, limit + 1e-9, spacing)
    for x in xy:
        for y in xy:
            if any(math.hypot(x - ox, y - oy) < KEEP_CLEAR
                   for _, ox, oy in obstacles):
                continue
            poses.append((float(x), float(y)))
    return poses


class Builder(object):
    def __init__(self, yaw_list, settle):
        self.occ = np.zeros((SIZE[1], SIZE[0]), dtype=np.int32)
        self.free = np.zeros((SIZE[1], SIZE[0]), dtype=np.int32)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state',
                                            SetModelState)
        self.yaw_list = yaw_list
        self.settle = settle
        self.scans = 0

    def to_cell(self, x, y):
        return (int(round((x - ORIGIN[0]) / RES)),
                int(round((y - ORIGIN[1]) / RES)))

    def teleport(self, x, y, yaw):
        state = ModelState()
        state.model_name = ROBOT
        state.reference_frame = 'world'
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.0
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist = Twist()
        self.set_state(state)

    def accumulate(self, scan, x, y, yaw):
        angles = (scan.angle_min
                  + np.arange(len(scan.ranges)) * scan.angle_increment)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges) & (ranges > max(scan.range_min, 0.22))
                 & (ranges < scan.range_max - 1e-3))
        cos, sin = np.cos(angles + yaw), np.sin(angles + yaw)
        for r, c, s in zip(ranges[valid], cos[valid], sin[valid]):
            steps = int(r / (RES * 0.5)) + 1
            ts = np.linspace(0.0, r, steps)
            # ray: free space up to (but not including) the endpoint
            fx, fy = x + c * ts[:-1], y + s * ts[:-1]
            ci = np.round((fx - ORIGIN[0]) / RES).astype(int)
            ri = np.round((fy - ORIGIN[1]) / RES).astype(int)
            ok = (ci >= 0) & (ci < SIZE[0]) & (ri >= 0) & (ri < SIZE[1])
            self.free[ri[ok], ci[ok]] += 1
            # endpoint: occupied
            ci, ri = self.to_cell(x + c * r, y + s * r)
            if 0 <= ci < SIZE[0] and 0 <= ri < SIZE[1]:
                self.occ[ri, ci] += 1
        self.scans += 1

    def sweep(self, targets):
        for n, (x, y) in enumerate(targets):
            yaw = self.yaw_list[n % len(self.yaw_list)]
            self.teleport(x, y, yaw)
            rospy.sleep(self.settle)
            try:
                scan = rospy.wait_for_message('/scan', LaserScan, timeout=5.0)
            except rospy.ROSException:
                rospy.logwarn('no scan at (%.2f, %.2f), skipping', x, y)
                continue
            self.accumulate(scan, x, y, yaw)
            if n % 10 == 0:
                rospy.loginfo('scanned %d/%d poses', n + 1, len(targets))

    def grid(self):
        occupied = self.occ >= 2
        free = (self.free >= 3) & ~occupied
        data = np.full((SIZE[1], SIZE[0]), UNKNOWN, dtype=np.uint8)
        data[free] = FREE
        data[occupied] = OCC
        return data


def write_pgm(path, data):
    # A PGM stores row 0 at the TOP, i.e. at the highest y, while ``data`` is
    # indexed like the OccupancyGrid (row 0 = origin.y). map_server flips the
    # image back when it loads it, so writing ``data`` unflipped yields a map
    # that is mirrored about y = -0.425 - the north boundary showed up at
    # y = 1.20 instead of y = 2.05, and the planner walled off whole corridors.
    image = np.flipud(data)
    header = 'P5\n# generated by build_map_from_sim.py\n%d %d\n255\n' % (
        image.shape[1], image.shape[0])
    with open(path, 'wb') as handle:
        handle.write(header.encode('ascii'))
        handle.write(image.tobytes())


def write_yaml(path, image):
    with open(path, 'w') as handle:
        handle.write('image: %s\n' % image)
        handle.write('resolution: %.6f\n' % RES)
        handle.write('origin: [%.6f, %.6f, 0.000000]\n' % ORIGIN)
        handle.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default=os.path.join(
        here, '..', '..', 'aic26_gazebo', 'worlds', 'community.world'))
    parser.add_argument('--out', default=os.path.join(
        here, '..', 'maps', 'world_sim.pgm'))
    parser.add_argument('--spacing', type=float, default=0.45)
    parser.add_argument('--settle', type=float, default=0.45)
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()

    rospy.init_node('build_map_from_sim', anonymous=True)
    obstacles = load_obstacles(args.world)
    targets = free_poses(obstacles, args.spacing)
    rospy.loginfo('%d teleport targets, %d visible models', len(targets),
                  len(obstacles))

    builder = Builder((0.0,), args.settle)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=60.0)
    rospy.wait_for_message('/scan', LaserScan, timeout=60.0)
    builder.sweep(targets)

    data = builder.grid()
    total = data.size
    print('scans          : %d' % builder.scans)
    print('occupied cells : %d (%.1f%%)'
          % ((data == OCC).sum(), 100.0 * (data == OCC).sum() / total))
    print('free cells     : %d (%.1f%%)'
          % ((data == FREE).sum(), 100.0 * (data == FREE).sum() / total))
    print('unknown cells  : %d (%.1f%%)'
          % ((data == UNKNOWN).sum(), 100.0 * (data == UNKNOWN).sum() / total))
    if args.write:
        write_pgm(args.out, data)
        write_yaml(os.path.splitext(args.out)[0] + '.yaml',
                   os.path.basename(args.out))
        print('wrote %s' % args.out)


if __name__ == '__main__':
    main()
