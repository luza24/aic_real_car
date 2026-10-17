"""Health monitor for the aic26_cobot hardware topics.

Subscribes to the chassis, IMU, lidar, camera and odometry topics and
publishes one diagnostic_msgs/DiagnosticArray on /diagnostics per second.
"""

import rospy
from diagnostic_msgs.msg import (
    DiagnosticArray, DiagnosticStatus, KeyValue)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan


class StatusNode(object):
    """Topic freshness monitor for the real hardware interfaces."""

    def __init__(self):
        self.timeout = float(rospy.get_param('~timeout', 2.0))
        self.hardware_id = rospy.get_param(
            '~hardware_id', 'aic26_cobot')

        self.topics = {
            'chassis': rospy.get_param('~vel_topic', '/vel_raw'),
            'imu': rospy.get_param('~imu_topic', '/imu/data'),
            'lidar': rospy.get_param('~scan_topic', '/scan'),
            'camera': rospy.get_param(
                '~image_topic', '/camera/image_raw'),
            'odom': rospy.get_param('~odom_topic', '/odom'),
        }
        self.last = {name: None for name in self.topics}

        self.pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)
        rospy.Subscriber(
            self.topics['chassis'], Twist,
            lambda msg, name='chassis': self._stamp(name))
        rospy.Subscriber(
            self.topics['imu'], Imu,
            lambda msg, name='imu': self._stamp(name))
        rospy.Subscriber(
            self.topics['lidar'], LaserScan,
            lambda msg, name='lidar': self._stamp(name))
        rospy.Subscriber(
            self.topics['camera'], Image,
            lambda msg, name='camera': self._stamp(name))
        rospy.Subscriber(
            self.topics['odom'], Odometry,
            lambda msg, name='odom': self._stamp(name))

        rospy.Timer(rospy.Duration(1.0), self._publish)
        rospy.loginfo('hardware_status: monitoring %d topics',
                      len(self.topics))

    def _stamp(self, name):
        self.last[name] = rospy.get_time()

    def _publish(self, _event):
        now = rospy.get_time()
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()

        for name, topic in self.topics.items():
            status = DiagnosticStatus()
            status.name = 'aic26_cobot: ' + name
            status.hardware_id = self.hardware_id

            stamp = self.last[name]
            if stamp is None:
                status.level = DiagnosticStatus.WARN
                status.message = 'no message yet'
            else:
                age = max(0.0, now - stamp)
                if age > self.timeout:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'stale %.1f s' % age
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'ok'
                status.values.append(
                    KeyValue(key='age', value='%.3f' % age))

            status.values.append(
                KeyValue(key='topic', value=topic))
            msg.status.append(status)

        self.pub.publish(msg)


def main():
    rospy.init_node('hardware_status')
    StatusNode()
    rospy.spin()


if __name__ == '__main__':
    main()
