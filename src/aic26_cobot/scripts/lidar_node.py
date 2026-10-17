"""ROS interface for the Leishen N10_P serial lidar.

Reads the 108-byte A5 5A packets from /dev/lslidar at 460800 baud,
accumulates a full revolution and publishes sensor_msgs/LaserScan on
/scan with frame_id "laser".
"""

import math
import os
import sys

import rospy
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hardware_common as hw      # noqa: E402


class LidarNode(object):
    """N10_P packet parser and LaserScan publisher."""

    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/lslidar')
        self.baud = int(rospy.get_param('~baud', 460800))
        self.topic = rospy.get_param('~topic', '/scan')
        self.frame_id = rospy.get_param('~frame_id', 'laser')

        self.bins = max(36, int(rospy.get_param('~bins', 720)))
        self.min_range = float(rospy.get_param('~min_range', 0.2))
        self.max_range = float(rospy.get_param('~max_range', 4.0))
        self.scan_rate = max(
            1.0, float(rospy.get_param('~scan_rate', 10.0)))
        self.scan_time = 1.0 / self.scan_rate

        self.angle_offset = float(
            rospy.get_param('~angle_offset_deg', 0.0))
        self.angle_sign = float(rospy.get_param('~angle_sign', 1))
        self.mask_start = float(
            rospy.get_param('~mask_start_deg', 90.0))
        self.mask_end = float(rospy.get_param('~mask_end_deg', 270.0))

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 2.0 * math.pi / self.bins

        self.ranges = [float('inf')] * self.bins
        self.intensities = [0.0] * self.bins
        self.point_count = 0
        self.last_start = None
        self.scan_start_time = rospy.Time.now()

        self.serial = hw.SerialDevice(self.port, self.baud)
        self._open_serial()
        self.pub = rospy.Publisher(
            self.topic, LaserScan, queue_size=2)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            'lidar: %s @ %d -> %s (frame %s, %d bins)',
            self.port, self.baud, self.topic,
            self.frame_id, self.bins)

    def _open_serial(self):
        if self.serial.open():
            rospy.loginfo(
                'lidar: opened %s at %d', self.port, self.baud)
            return True
        rospy.logwarn_throttle(
            10.0, 'lidar: cannot open %s: %s',
            self.port, self.serial.last_error)
        return False

    def spin(self):
        buffer = bytearray()
        while not rospy.is_shutdown():
            if not self.serial.is_open:
                self._open_serial()
                rospy.sleep(1.0)
                continue

            data = self.serial.read(4096)
            if not data:
                continue
            buffer.extend(data)
            self._consume(buffer)

    def _consume(self, buffer):
        while len(buffer) >= hw.LIDAR_PACKET_SIZE:
            start = buffer.find(hw.LIDAR_HEADER)
            if start < 0:
                buffer[:] = buffer[-1:]
                return
            if start > 0:
                del buffer[:start]
            if len(buffer) < hw.LIDAR_PACKET_SIZE:
                return

            packet = bytes(buffer[:hw.LIDAR_PACKET_SIZE])
            parsed = hw.parse_n10p_packet(packet)
            if parsed is None:
                del buffer[:1]
                continue
            del buffer[:hw.LIDAR_PACKET_SIZE]
            self._handle_packet(*parsed)

    def _handle_packet(self, start, _end, points):
        now = rospy.Time.now()

        for angle, distance, intensity in points:
            angle = (angle * self.angle_sign +
                     self.angle_offset) % 360.0
            if self._masked(angle):
                continue
            if distance < self.min_range or distance > self.max_range:
                continue

            radians = math.radians(angle)
            if radians > math.pi:
                radians -= 2.0 * math.pi
            index = int(round(
                (radians - self.angle_min) /
                self.angle_increment)) % self.bins
            self.ranges[index] = distance
            self.intensities[index] = intensity
            self.point_count += 1

        if (self.last_start is not None and
                self.last_start > 300.0 and start < 60.0):
            self._publish_scan(now)
        elif (now - self.scan_start_time).to_sec() > 1.5 * self.scan_time:
            self._publish_scan(now)

        self.last_start = start

    def _masked(self, angle):
        if self.mask_start < 0.0 or self.mask_end < 0.0:
            return False
        if self.mask_start <= self.mask_end:
            return self.mask_start <= angle <= self.mask_end
        return angle >= self.mask_start or angle <= self.mask_end

    def _publish_scan(self, now):
        if self.point_count == 0:
            self.scan_start_time = now
            return

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = self.scan_time / self.bins
        scan.scan_time = self.scan_time
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = list(self.ranges)
        scan.intensities = list(self.intensities)
        self.pub.publish(scan)

        rospy.logdebug_throttle(
            5.0, 'lidar: published %d points', self.point_count)
        self.ranges = [float('inf')] * self.bins
        self.intensities = [0.0] * self.bins
        self.point_count = 0
        self.scan_start_time = now

    def _shutdown(self):
        self.serial.close()
        rospy.loginfo('lidar: stopped')


def main():
    rospy.init_node('lidar')
    try:
        node = LidarNode()
    except Exception as exc:
        rospy.logfatal('lidar: %s', exc)
        return
    node.spin()


if __name__ == '__main__':
    main()
