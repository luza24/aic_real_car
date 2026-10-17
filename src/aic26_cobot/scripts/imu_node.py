"""ROS interface for the WIT normal-protocol IMU on /dev/imu.

The node parses the 0x51/0x52/0x53/0x54 frames, publishes
sensor_msgs/Imu on /imu/data and optionally the raw magnetic field on
/imu/mag.
"""

import math
import os
import sys
import threading

import rospy
from sensor_msgs.msg import Imu, MagneticField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hardware_common as hw      # noqa: E402


class ImuNode(object):
    """WIT serial protocol to sensor_msgs/Imu bridge."""

    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/imu')
        self.baud = int(rospy.get_param('~baud', 9600))
        self.topic = rospy.get_param('~topic', '/imu/data')
        self.mag_topic = rospy.get_param('~mag_topic', '/imu/mag')
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_mag = bool(
            rospy.get_param('~publish_mag', False))
        self.mag_scale = float(rospy.get_param('~mag_scale', 1.0))

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.angles = None

        self.serial = hw.SerialDevice(self.port, self.baud)
        self._open_serial()

        self.pub = rospy.Publisher(self.topic, Imu, queue_size=10)
        self.mag_pub = None
        if self.publish_mag:
            self.mag_pub = rospy.Publisher(
                self.mag_topic, MagneticField, queue_size=10)

        self._reader = threading.Thread(target=self._read_loop)
        self._reader.daemon = True
        self._reader.start()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            'imu: %s @ %d -> %s (frame %s)',
            self.port, self.baud, self.topic, self.frame_id)

    def _open_serial(self):
        if self.serial.open():
            rospy.loginfo(
                'imu: opened %s at %d', self.port, self.baud)
            return True
        rospy.logwarn_throttle(
            10.0, 'imu: cannot open %s: %s',
            self.port, self.serial.last_error)
        return False

    def _read_loop(self):
        buffer = bytearray()
        while not rospy.is_shutdown():
            if not self.serial.is_open:
                self._open_serial()
                rospy.sleep(1.0)
                continue

            data = self.serial.read(64)
            if not data:
                continue
            buffer.extend(data)

            while len(buffer) >= hw.WIT_FRAME_SIZE:
                start = buffer.find(bytes([hw.WIT_HEADER]))
                if start < 0:
                    buffer[:] = buffer[-1:]
                    break
                if start > 0:
                    del buffer[:start]
                if len(buffer) < hw.WIT_FRAME_SIZE:
                    break

                frame = bytes(buffer[:hw.WIT_FRAME_SIZE])
                parsed = hw.parse_wit_frame(frame)
                if parsed is None:
                    del buffer[:1]
                    continue
                del buffer[:hw.WIT_FRAME_SIZE]
                self._handle_frame(parsed)

    def _handle_frame(self, parsed):
        kind = parsed['type']
        if kind == 'accel':
            self.accel = parsed['values']
        elif kind == 'gyro':
            self.gyro = parsed['values']
        elif kind == 'angle':
            self.angles = parsed['values']
            self._publish_imu()
        elif kind == 'mag' and self.publish_mag:
            self._publish_mag(parsed['values'])

    def _publish_imu(self):
        roll, pitch, yaw = [math.radians(v) for v in self.angles]
        quat = hw.quaternion_from_rpy(roll, pitch, yaw)

        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = self.frame_id

        imu.orientation.x = quat[0]
        imu.orientation.y = quat[1]
        imu.orientation.z = quat[2]
        imu.orientation.w = quat[3]
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.05

        imu.angular_velocity.x = self.gyro[0]
        imu.angular_velocity.y = self.gyro[1]
        imu.angular_velocity.z = self.gyro[2]
        imu.angular_velocity_covariance[0] = 0.0004
        imu.angular_velocity_covariance[4] = 0.0004
        imu.angular_velocity_covariance[8] = 0.0004

        imu.linear_acceleration.x = self.accel[0]
        imu.linear_acceleration.y = self.accel[1]
        imu.linear_acceleration.z = self.accel[2]
        imu.linear_acceleration_covariance[0] = 0.04
        imu.linear_acceleration_covariance[4] = 0.04
        imu.linear_acceleration_covariance[8] = 0.04

        self.pub.publish(imu)

    def _publish_mag(self, values):
        mag = MagneticField()
        mag.header.stamp = rospy.Time.now()
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = values[0] * self.mag_scale
        mag.magnetic_field.y = values[1] * self.mag_scale
        mag.magnetic_field.z = values[2] * self.mag_scale
        mag.magnetic_field_covariance[0] = -1.0
        self.mag_pub.publish(mag)

    def _shutdown(self):
        self.serial.close()
        rospy.loginfo('imu: stopped')


def main():
    rospy.init_node('imu')
    try:
        ImuNode()
    except Exception as exc:
        rospy.logfatal('imu: %s', exc)
        return
    rospy.spin()


if __name__ == '__main__':
    main()
