"""ROS interface for the STM32 four-wheel chassis.

Subscribes /cmd_vel and sends the 12-byte AA BB 0A 12 02 frame on the
serial port. Reads the matching feedback frame, publishes /vel_raw and
/odom, and broadcasts odom -> base_footprint.
"""

import math
import os
import sys
import threading
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hardware_common as hw      # noqa: E402


class ChassisNode(object):
    """Bridge between ROS Twist messages and the chassis serial protocol."""

    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/carserial')
        self.baud = int(rospy.get_param('~baud', 115200))

        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.vel_topic = rospy.get_param('~vel_topic', '/vel_raw')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')

        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param(
            '~base_frame', 'base_footprint')
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))

        self.publish_rate = max(
            1.0, float(rospy.get_param('~publish_rate', 20.0)))
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.feedback_timeout = float(
            rospy.get_param('~feedback_timeout', 1.0))
        self.byte_delay = float(rospy.get_param('~byte_delay', 0.0))

        self.linear_scale = float(
            rospy.get_param('~linear_scale', 1.0))
        self.angular_scale = float(
            rospy.get_param('~angular_scale', 0.96))
        self.max_linear = float(rospy.get_param('~max_linear', 0.5))
        self.max_angular = float(rospy.get_param('~max_angular', 1.5))

        self.use_imu_yaw = bool(
            rospy.get_param('~use_imu_yaw', True))
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.imu_timeout = float(rospy.get_param('~imu_timeout', 0.2))
        self.imu_correction_gain = float(
            rospy.get_param('~imu_correction_gain', 0.1))

        self._lock = threading.Lock()
        self._pose = [0.0, 0.0, 0.0]
        self._last_odom_time = rospy.Time.now()
        self._last_feedback = (0.0, 0.0, 0.0)
        self._last_feedback_time = rospy.Time(0)
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = rospy.Time(0)
        self._enabled = True
        self._imu_yaw = None
        self._imu_yaw_time = rospy.Time(0)
        self._imu_offset = None

        self.serial = hw.SerialDevice(self.port, self.baud)
        self._open_serial()

        self.vel_pub = rospy.Publisher(
            self.vel_topic, Twist, queue_size=10)
        self.odom_pub = rospy.Publisher(
            self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb,
                         queue_size=10)
        if self.use_imu_yaw:
            rospy.Subscriber(self.imu_topic, Imu, self._imu_cb,
                             queue_size=10)

        rospy.Service('~enable', SetBool, self._enable_cb)
        rospy.Service('~reset_odom', Empty, self._reset_cb)

        self._reader = threading.Thread(target=self._read_loop)
        self._reader.daemon = True
        self._reader.start()

        period = rospy.Duration(1.0 / self.publish_rate)
        self._cmd_timer = rospy.Timer(period, self._send_command)
        self._odom_timer = rospy.Timer(period, self._publish_odom)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            'chassis: %s @ %d, odom %s -> %s',
            self.port, self.baud, self.odom_frame, self.base_frame)

    def _open_serial(self):
        if self.serial.open():
            rospy.loginfo(
                'chassis: opened %s at %d', self.port, self.baud)
            return True
        rospy.logwarn_throttle(
            10.0, 'chassis: cannot open %s: %s',
            self.port, self.serial.last_error)
        return False

    def _cmd_cb(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_cmd_time = rospy.Time.now()

    def _imu_cb(self, msg):
        q = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))
        with self._lock:
            self._imu_yaw = yaw
            self._imu_yaw_time = rospy.Time.now()
            if self._imu_offset is None:
                self._imu_offset = yaw

    def _send_command(self, _event):
        now = rospy.Time.now()
        with self._lock:
            cmd = self._cmd
            stamp = self._last_cmd_time
            enabled = self._enabled

        if not enabled:
            cmd = (0.0, 0.0, 0.0)
        elif (self.cmd_timeout > 0.0 and
              (now - stamp).to_sec() > self.cmd_timeout):
            cmd = (0.0, 0.0, 0.0)

        vx = cmd[0] * self.linear_scale
        vy = cmd[1] * self.linear_scale
        wz = cmd[2] * self.angular_scale
        frame = hw.encode_chassis_command(
            vx, vy, wz, self.max_linear, self.max_angular)

        if not self.serial.is_open:
            return

        if self.byte_delay > 0.0:
            for byte in frame:
                if not self.serial.write(bytes([byte])):
                    break
                time.sleep(self.byte_delay)
        else:
            self.serial.write(frame)

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

            while len(buffer) >= hw.CHASSIS_FEEDBACK_SIZE:
                start = buffer.find(hw.CHASSIS_HEADER)
                if start < 0:
                    buffer[:] = buffer[-1:]
                    break
                if start > 0:
                    del buffer[:start]
                if len(buffer) < hw.CHASSIS_FEEDBACK_SIZE:
                    break

                frame = bytes(buffer[:hw.CHASSIS_FEEDBACK_SIZE])
                feedback = hw.decode_chassis_feedback(frame)
                if feedback is None:
                    del buffer[:1]
                    continue
                del buffer[:hw.CHASSIS_FEEDBACK_SIZE]
                self._handle_feedback(feedback)

    def _handle_feedback(self, feedback):
        vx, vy, wz = feedback
        wz *= self.angular_scale
        now = rospy.Time.now()

        with self._lock:
            dt = (now - self._last_odom_time).to_sec()
            dt = hw.clamp(dt, 0.0, 0.5)
            yaw = self._pose[2] + wz * dt

            if self.use_imu_yaw and self._imu_yaw is not None:
                age = (now - self._imu_yaw_time).to_sec()
                if age <= self.imu_timeout:
                    target = hw.wrap_radians(
                        self._imu_yaw - self._imu_offset)
                    error = hw.wrap_radians(target - yaw)
                    yaw += self.imu_correction_gain * error

            yaw = hw.wrap_radians(yaw)
            self._pose[0] += (
                vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            self._pose[1] += (
                vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            self._pose[2] = yaw
            self._last_odom_time = now
            self._last_feedback = (vx, vy, wz)
            self._last_feedback_time = now

    def _publish_odom(self, _event):
        now = rospy.Time.now()
        with self._lock:
            x, y, yaw = self._pose
            vx, vy, wz = self._last_feedback
            stamp = self._last_feedback_time

        if (now - stamp).to_sec() > self.feedback_timeout:
            vx = vy = wz = 0.0

        odom = self._make_odom(now, x, y, yaw, vx, vy, wz)
        self.odom_pub.publish(odom)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.vel_pub.publish(twist)

        if self.publish_tf:
            quat = hw.quaternion_from_rpy(0.0, 0.0, yaw)
            self.tf_broadcaster.sendTransform(
                (x, y, 0.0), quat, now,
                self.base_frame, self.odom_frame)

    def _make_odom(self, now, x, y, yaw, vx, vy, wz):
        quat = hw.quaternion_from_rpy(0.0, 0.0, yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001
        return odom

    def _enable_cb(self, req):
        with self._lock:
            self._enabled = bool(req.data)
            self._cmd = (0.0, 0.0, 0.0)
            self._last_cmd_time = rospy.Time.now()
        if not req.data:
            self._stop_motors()
        message = 'enabled' if req.data else 'disabled'
        rospy.loginfo('chassis: motors %s', message)
        return SetBoolResponse(success=True, message=message)

    def _reset_cb(self, _req):
        now = rospy.Time.now()
        with self._lock:
            self._pose = [0.0, 0.0, 0.0]
            self._last_odom_time = now
            self._last_feedback = (0.0, 0.0, 0.0)
            self._last_feedback_time = now
            self._imu_offset = self._imu_yaw
        rospy.loginfo('chassis: odometry reset')
        return EmptyResponse()

    def _stop_motors(self):
        if self.serial.is_open:
            self.serial.write(hw.encode_chassis_command(0.0, 0.0, 0.0))

    def _shutdown(self):
        for _ in range(5):
            self._stop_motors()
            time.sleep(0.01)
        self.serial.close()
        rospy.loginfo('chassis: stopped')


def main():
    rospy.init_node('chassis')
    try:
        ChassisNode()
    except Exception as exc:
        rospy.logfatal('chassis: %s', exc)
        return
    rospy.spin()


if __name__ == '__main__':
    main()

