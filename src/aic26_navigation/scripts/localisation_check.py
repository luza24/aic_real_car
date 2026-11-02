#!/usr/bin/env python3
"""Measure how well /scan matches /map, and correct the start pose.

Publishes the median distance from every valid beam to the nearest mapped
obstacle: AMCL's sigma only describes how spread the particle cloud is, so a
confidently wrong pose (sigma 0.02 while 0.2 m off) looks perfect there. At
start-up it also searches a bounded window around the AMCL pose and pushes the
best one through /initialpose, so a hand-placed robot is corrected before
anything drives. The window is bounded on purpose: the arena is almost square,
so an unbounded search falls into the 90/180 deg alias.
"""

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse


def yaw_of(quaternion):
    x, y, z, w = quaternion
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DistanceField(object):
    """Distance in metres from every cell centre to the nearest occupied cell."""

    def __init__(self, grid):
        self.resolution = grid.info.resolution
        self.origin_x = grid.info.origin.position.x
        self.origin_y = grid.info.origin.position.y
        self.height = grid.info.height
        self.width = grid.info.width
        data = np.asarray(grid.data, dtype=np.int16).reshape(
            self.height, self.width)
        rows, cols = np.nonzero(data > 65)
        self.field = None
        if not rows.size:
            return
        centre_x = self.origin_x + (cols + 0.5) * self.resolution
        centre_y = self.origin_y + (rows + 0.5) * self.resolution
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        field = np.empty((self.height, self.width), dtype=np.float32)
        for start in range(0, self.height, 32):
            stop = min(start + 32, self.height)
            gx, gy = np.meshgrid(xs, ys[start:stop])
            d2 = ((gx[..., None] - centre_x) ** 2 +
                  (gy[..., None] - centre_y) ** 2).min(axis=2)
            field[start:stop] = np.sqrt(d2)
        self.field = field

    def lookup(self, x, y):
        if self.field is None:
            return None
        col = np.rint((x - self.origin_x) / self.resolution - 0.5).astype(int)
        row = np.rint((y - self.origin_y) / self.resolution - 0.5).astype(int)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & \
                 (row < self.height)
        out = np.full(x.shape, np.nan, dtype=np.float32)
        out[inside] = self.field[row[inside], col[inside]]
        return out

    def residual(self, x, y, yaw, angles, ranges):
        """Low percentile of the beam to map distances.

        A median is hijacked by large objects that the map does not contain
        (the three cars next to car_1..3 cover a wide arc and pushed it to
        0.22 m while the pose was fine). The lower percentile only asks that
        a solid minority of the far beams still lands on mapped structure.
        """
        distance = self.lookup(x + np.cos(yaw + angles) * ranges,
                              y + np.sin(yaw + angles) * ranges)
        distance = distance[~np.isnan(distance)]
        if distance.size < 30:
            return None
        return float(np.percentile(distance, 35))


class LocalisationCheck(object):

    def __init__(self):
        self.field = None
        self.scan = None
        self.listener = tf.TransformListener()
        self.max_residual = float(rospy.get_param('~max_residual', 0.18))
        self.auto_align = bool(rospy.get_param('~auto_align', True))
        self.align_threshold = float(
            rospy.get_param('~align_threshold', 0.10))
        self.min_beam_range = float(rospy.get_param('~min_beam_range', 0.8))
        self.window_xy = float(rospy.get_param('~window_xy', 0.15))
        self.window_yaw = float(rospy.get_param('~window_yaw', 0.08))
        self.aligned = False

        rospy.Subscriber('/map', OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self._on_scan, queue_size=1)
        self.residual_pub = rospy.Publisher(
            rospy.get_param('~residual_topic', '/localisation/scan_residual'),
            Float32, queue_size=1)
        self.ok_pub = rospy.Publisher(
            rospy.get_param('~ok_topic', '/localisation/scan_ok'),
            Bool, queue_size=1)
        self.pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Service('~align', Trigger, self._on_align)
        rospy.Timer(rospy.Duration(1.0), self._on_timer)

    def _on_map(self, msg):
        self.field = DistanceField(msg)

    def _on_scan(self, msg):
        self.scan = msg

    def _beams(self):
        scan = self.scan
        if scan is None:
            return None
        angles = (scan.angle_min +
                  np.arange(len(scan.ranges)) * scan.angle_increment)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (np.isfinite(ranges) & (ranges > self.min_beam_range) &
                 (ranges < scan.range_max - 1e-6))
        return angles[valid], ranges[valid]

    def _lookup_pose(self):
        """Laser pose in the map frame, preferring the stamp of the last scan."""
        if self.scan is None:
            return None
        for when in (self.scan.header.stamp, rospy.Time(0)):
            try:
                translation, rotation = self.listener.lookupTransform(
                    'map', self.scan.header.frame_id, when)
            except (tf.LookupException, tf.ExtrapolationException):
                continue
            return translation[0], translation[1], yaw_of(rotation)
        return None

    def measure(self):
        if self.field is None or self.scan is None:
            return None
        pose = self._lookup_pose()
        beams = self._beams()
        if pose is None or beams is None:
            return None
        angles, ranges = beams
        return self.field.residual(pose[0], pose[1], pose[2], angles, ranges)

    def _on_timer(self, _event):
        value = self.measure()
        if value is None:
            return
        self.residual_pub.publish(Float32(value))
        self.ok_pub.publish(Bool(value <= self.max_residual + 1e-3))
        if (self.auto_align and not self.aligned and
                value > self.align_threshold):
            self.aligned = True
            rospy.logwarn('scan to map residual %.3f m, refining the pose',
                          value)
            self.align()

    def align(self):
        """Run the bounded search once and report what moved."""
        pose = self._lookup_pose()
        beams = self._beams()
        if pose is None or beams is None or self.field is None:
            return 'no pose, scan or map yet'
        angles, ranges = beams
        before = self.field.residual(pose[0], pose[1], pose[2], angles, ranges)
        best, offset = self._search(pose, angles, ranges)
        if best is None:
            return 'search failed'
        if before is not None and best >= before - 0.01:
            return 'already aligned, residual %.3f m' % before
        self._publish_pose(pose[0] + offset[0], pose[1] + offset[1],
                           pose[2] + offset[2])
        message = ('offset %.2f m / %.1f deg, residual %.3f -> %.3f m'
                   % (np.hypot(offset[0], offset[1]),
                      np.degrees(offset[2]),
                      before if before is not None else float('nan'), best))
        rospy.logwarn('start pose refined: %s', message)
        return message

    def _on_align(self, _request):
        return TriggerResponse(True, self.align())

    def _search(self, pose, angles, ranges):
        best = self._grid(pose, angles, ranges,
                          self.window_xy, 0.03, self.window_yaw, 0.02)
        if best[0] is None:
            return best
        centre = (pose[0] + best[1][0], pose[1] + best[1][1],
                  pose[2] + best[1][2])
        fine = self._grid(centre, angles, ranges, 0.04, 0.01, 0.03, 0.01)
        if fine[0] is not None and fine[0] < best[0]:
            return fine[0], (best[1][0] + fine[1][0],
                             best[1][1] + fine[1][1],
                             best[1][2] + fine[1][2])
        return best

    def _grid(self, pose, angles, ranges, span_xy, step_xy, span_yaw, step_yaw):
        dxs = np.arange(-span_xy, span_xy + 1e-9, step_xy)
        dys = np.arange(-span_xy, span_xy + 1e-9, step_xy)
        dyaws = np.arange(-span_yaw, span_yaw + 1e-9, step_yaw)
        best_value, best_offset = None, (0.0, 0.0, 0.0)
        for dyaw in dyaws:
            cos_a = np.cos(pose[2] + dyaw + angles)
            sin_a = np.sin(pose[2] + dyaw + angles)
            for dx in dxs:
                xs = pose[0] + dx + cos_a * ranges
                for dy in dys:
                    distance = self.field.lookup(xs, pose[1] + dy + sin_a * ranges)
                    distance = distance[~np.isnan(distance)]
                    if distance.size < 30:
                        continue
                    value = float(np.median(distance))
                    if best_value is None or value < best_value:
                        best_value, best_offset = value, (dx, dy, dyaw)
        return best_value, best_offset

    def _publish_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.0025
        covariance[35] = 0.0025
        msg.pose.covariance = covariance
        self.pose_pub.publish(msg)


def main():
    rospy.init_node('localisation_check')
    LocalisationCheck()
    rospy.spin()


if __name__ == '__main__':
    main()


