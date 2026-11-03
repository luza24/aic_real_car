#!/usr/bin/env python3
"""Publish the track boundary lines as an OccupancyGrid.

Both costmaps consume it through a StaticLayer so the painted road
lines are lethal obstacles even though the lidar cannot see them.
The same geometry is published as a LINE_LIST marker for RViz.
"""

import os

import numpy as np
import rospkg
import rospy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


def load_config(path):
    with open(path, 'r') as handle:
        config = yaml.safe_load(handle)
    if 'segments' not in config:
        raise ValueError('%s defines no segments' % path)
    return config


def point_segment_distance(xs, ys, x1, y1, x2, y2):
    """Distance from every point in the (xs, ys) mesh to one segment."""
    dx, dy = x2 - x1, y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return np.hypot(xs - x1, ys - y1)
    t = ((xs - x1) * dx + (ys - y1) * dy) / length_sq
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(xs - (x1 + t * dx), ys - (y1 + t * dy))


class WallGrid(object):
    """Rasterise the wall segments into an OccupancyGrid-shaped array."""

    def __init__(self, config, frame_id=None):
        self.resolution = float(config.get('resolution', 0.05))
        origin = config.get('origin', [-6.0, -6.0])
        size = config.get('size', [224, 224])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.width = int(size[0])
        self.height = int(size[1])
        self.frame_id = frame_id or str(config.get('frame_id', 'map'))
        self.cost = int(max(1, min(100, config.get('cost', 100))))
        # A line is at least one cell wide, however thin it is configured:
        # with half a cell as radius every cell whose centre lies on the line
        # is marked, which yields a continuous one-cell skeleton.
        self.radius = max(float(config.get('line_width', 0.05)),
                          self.resolution) / 2.0
        self.segments = [tuple(float(v) for v in segment)
                         for segment in config['segments']]
        for segment in self.segments:
            if len(segment) != 4:
                raise ValueError(
                    'segment %r is not [x1, y1, x2, y2]' % (segment,))
        self.data = self._rasterise()

    def _rasterise(self):
        data = np.zeros((self.height, self.width), dtype=np.uint8)
        for x1, y1, x2, y2 in self.segments:
            # Only look at the cells around the segment, not the whole grid.
            col0 = int(np.floor((min(x1, x2) - self.radius - self.origin_x) /
                                self.resolution))
            col1 = int(np.ceil((max(x1, x2) + self.radius - self.origin_x) /
                               self.resolution))
            row0 = int(np.floor((min(y1, y2) - self.radius - self.origin_y) /
                                self.resolution))
            row1 = int(np.ceil((max(y1, y2) + self.radius - self.origin_y) /
                               self.resolution))
            col0, row0 = max(col0, 0), max(row0, 0)
            col1, row1 = min(col1, self.width - 1), min(row1, self.height - 1)
            if col0 > col1 or row0 > row1:
                rospy.logwarn('virtual_walls: segment %s..%s is outside the '
                              'grid', (x1, y1), (x2, y2))
                continue

            cols = np.arange(col0, col1 + 1)
            rows = np.arange(row0, row1 + 1)
            # OccupancyGrid cell (i, j) has its centre at
            #   x = origin_x + (i + 0.5) * resolution
            #   y = origin_y + (j + 0.5) * resolution
            xs = self.origin_x + (cols + 0.5) * self.resolution
            ys = self.origin_y + (rows + 0.5) * self.resolution
            grid_x, grid_y = np.meshgrid(xs, ys)
            distance = point_segment_distance(
                grid_x, grid_y, x1, y1, x2, y2)
            block = data[np.ix_(rows, cols)]
            block[distance <= self.radius] = self.cost
            data[np.ix_(rows, cols)] = block
        return data

    def cell_count(self):
        return int((self.data > 0).sum())

    def message(self, stamp):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        # Row major, index 0 = lower left corner of the grid.
        msg.data = self.data.reshape(-1).tolist()
        return msg

    def marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'virtual_walls'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.resolution
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.points = []
        for x1, y1, x2, y2 in self.segments:
            marker.points.append(Point(x1, y1, 0.02))
            marker.points.append(Point(x2, y2, 0.02))
        marker.frame_locked = True
        return marker


class VirtualWalls(object):
    """Publish the wall grid and the matching RViz marker."""

    def __init__(self):
        config_path = rospy.get_param('~config', '')
        if not config_path:
            config_path = os.path.join(
                rospkg.RosPack().get_path('aic26_navigation'),
                'config', 'virtual_walls.yaml')
        config = load_config(config_path)

        self.grid = WallGrid(config, rospy.get_param('~frame_id', None))
        self.period = float(rospy.get_param('~republish_period', 1.0))

        grid_topic = rospy.get_param('~topic', '/virtual_walls')
        self.grid_pub = rospy.Publisher(
            grid_topic, OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(
            rospy.get_param('~marker_topic', '/virtual_walls_markers'),
            Marker, queue_size=1, latch=True)

        # Build the grid once; only the stamp changes between publications.
        self.grid_message = self.grid.message(rospy.Time.now())
        rospy.Timer(rospy.Duration(max(0.1, self.period)), self._publish)
        self._publish(None)

        rospy.loginfo(
            'virtual_walls: %d segments, %d wall cells at %d/100 '
            '(%dx%d cells, %.3f m, origin %.2f,%.2f, frame %s) -> %s',
            len(self.grid.segments), self.grid.cell_count(), self.grid.cost,
            self.grid.width, self.grid.height, self.grid.resolution,
            self.grid.origin_x, self.grid.origin_y, self.grid.frame_id,
            grid_topic)

    def _publish(self, _event):
        stamp = rospy.Time.now()
        self.grid_message.header.stamp = stamp
        self.grid_pub.publish(self.grid_message)
        self.marker_pub.publish(self.grid.marker(stamp))


def main():
    rospy.init_node('virtual_walls')
    VirtualWalls()
    rospy.spin()


if __name__ == '__main__':
    main()
