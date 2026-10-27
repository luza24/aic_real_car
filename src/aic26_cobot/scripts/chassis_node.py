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
        # The STM32 needs a legacy 0x11 handshake after power-up; without it
        # the motors ignore the speed frames. The old car_bringup sender used
        # a 1 ms gap for the init bytes and 0.4 ms between command bytes.
        self.send_init = bool(rospy.get_param('~send_init', True))
        self.init_delay = max(
            0.0, float(rospy.get_param('~init_delay', 0.001)))
        self.cmd_rate = max(
            1.0, float(rospy.get_param('~cmd_rate', 10.0)))

        self.linear_cmd_scale = float(
            rospy.get_param('~linear_cmd_scale', 1.0))
        self.angular_cmd_scale = float(
            rospy.get_param('~angular_cmd_scale', 1.0))
        self.linear_odom_scale = float(
            rospy.get_param('~linear_odom_scale', 1.0))
        self.angular_odom_scale = float(
            rospy.get_param('~angular_odom_scale', 1.0))
        self.max_linear = float(rospy.get_param('~max_linear', 0.5))
        self.max_angular = float(rospy.get_param('~max_angular', 1.5))

        self.use_imu_yaw = bool(
            rospy.get_param('~use_imu_yaw', True))
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.imu_timeout = float(rospy.get_param('~imu_timeout', 0.2))
        # The WIT gyro z rate is rate-only (not the magnetic yaw), so it is
        # stable indoors. A standstill bias estimate is folded in, and a
        # small encoder share keeps the long-term scale honest.
        self.imu_gyro_gain = hw.clamp(
            float(rospy.get_param('~imu_gyro_gain', 0.95)), 0.0, 1.0)
        self.imu_gyro_sign = float(
            rospy.get_param('~imu_gyro_sign', 1.0))
        self.imu_gyro_bias_alpha = hw.clamp(
            float(rospy.get_param('~imu_gyro_bias_alpha', 0.02)),
            0.0, 1.0)
        self.imu_gyro_bias_limit = abs(float(
            rospy.get_param('~imu_gyro_bias_limit', 0.15)))

        # The chassis sends one feedback frame at a fixed rate. The USB
        # adapter delivers those frames in bursts, so integrate each frame
        # with the nominal frame period instead of the wall-clock arrival
        # interval (see hardware_common.feedback_dt).
        self.feedback_rate = max(
            1.0, float(rospy.get_param('~feedback_rate', 20.0)))
        self.feedback_dt_max = max(
            1.0 / self.feedback_rate,
            float(rospy.get_param('~feedback_dt_max', 0.2)))
        self.linear_deadband = abs(float(
            rospy.get_param('~linear_deadband', 0.002)))
        self.angular_deadband = abs(float(
            rospy.get_param('~angular_deadband', 0.002)))

        # Start odometry at the simulated spawn pose so the RViz TF tree and
        # map->odom do not show a different origin on real hardware.
        self.initial_x = float(rospy.get_param('~initial_x', 0.0))
        self.initial_y = float(rospy.get_param('~initial_y', 0.0))
        self.initial_yaw = hw.wrap_radians(
            float(rospy.get_param('~initial_yaw', 0.0)))

        self._lock = threading.Lock()
        self._pose = [self.initial_x, self.initial_y, self.initial_yaw]
        self._last_odom_time = None
        self._last_feedback = (0.0, 0.0, 0.0)
        self._last_feedback_time = rospy.Time(0)
        self._last_moving_feedback_time = rospy.Time.now()
        self._nonzero_cmd_start = None
        self._cmd = (0.0, 0.0, 0.0)
        self._last_cmd_time = rospy.Time(0)
        self._enabled = True
        self._gyro = None
        self._gyro_time = rospy.Time(0)
        self._gyro_bias = 0.0

        self.serial = hw.SerialDevice(self.port, self.baud)
        self._open_serial()
        # Clear any velocity left over from a previous process before the first
        # command timer tick.
        self._stop_motors()

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

        odom_period = rospy.Duration(1.0 / self.publish_rate)
        cmd_period = rospy.Duration(1.0 / self.cmd_rate)
        self._cmd_timer = rospy.Timer(cmd_period, self._send_command)
        self._odom_timer = rospy.Timer(odom_period, self._publish_odom)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            'chassis: %s @ %d, odom %s -> %s',
            self.port, self.baud, self.odom_frame, self.base_frame)
        rospy.loginfo(
            'chassis: init=%s cmd_rate=%.1f Hz byte_delay=%.4f s',
            self.send_init, self.cmd_rate, self.byte_delay)

    def _open_serial(self):
        if self.serial.open():
            rospy.loginfo(
                'chassis: opened %s at %d', self.port, self.baud)
            # Bring the STM32 into command mode before sending any speed
            # frame. This handshake is required after the motor power is
            # switched on; the old car_bringup sender did the same.
            if self.send_init:
                self._send_init()
            # The STM32 may still hold the last frame from a previous process.
            self._stop_motors()
            return True
        rospy.logwarn_throttle(
            10.0, 'chassis: cannot open %s: %s',
            self.port, self.serial.last_error)
        return False

    def _send_init(self):
        """Send the legacy 0x11 + 8x00 chassis init handshake."""
        for byte in hw.CHASSIS_INIT:
            if not self.serial.write(bytes([byte])):
                rospy.logwarn('chassis: init write failed')
                return False
            if self.init_delay > 0.0:
                time.sleep(self.init_delay)
        rospy.loginfo(
            'chassis: sent init sequence (%d bytes)', len(hw.CHASSIS_INIT))
        return True

    def _cmd_cb(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_cmd_time = rospy.Time.now()

    def _imu_cb(self, msg):
        if msg.angular_velocity_covariance[0] < 0.0:
            with self._lock:
                self._gyro = None
            return
        with self._lock:
            self._gyro = self.imu_gyro_sign * msg.angular_velocity.z
            self._gyro_time = rospy.Time.now()

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

        vx = cmd[0] * self.linear_cmd_scale
        vy = cmd[1] * self.linear_cmd_scale
        wz = cmd[2] * self.angular_cmd_scale
        frame = hw.encode_chassis_command(
            vx, vy, wz, self.max_linear, self.max_angular)

        nonzero = (abs(vx) >= 0.05 or abs(vy) >= 0.05 or
                   abs(wz) >= 0.05)
        if enabled and nonzero:
            with self._lock:
                if self._nonzero_cmd_start is None:
                    self._nonzero_cmd_start = now
                moving = self._last_moving_feedback_time
                started = self._nonzero_cmd_start
            if (now - max(moving, started)).to_sec() > 1.0:
                rospy.logwarn_throttle(
                    2.0,
                    'chassis: nonzero command but feedback is zero for '
                    'over 1 s; check motor power and 0x11 init')
        else:
            with self._lock:
                self._nonzero_cmd_start = None

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

            # Read exactly one 12-byte frame at a time, like the old
            # car_bringup pubv.py.  Reading 64 bytes at once makes the
            # reader integrate a whole burst with one dt for the first
            # frame and ~0 for the rest, which shows up as pose jumps.
            data = self.serial.read(hw.CHASSIS_FEEDBACK_SIZE)
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
        vx = feedback[0] * self.linear_odom_scale
        vy = feedback[1] * self.linear_odom_scale
        wz = feedback[2] * self.angular_odom_scale
        now = rospy.Time.now()

        # Ignore encoder quantization / electrical noise at standstill.
        if abs(vx) < self.linear_deadband:
            vx = 0.0
        if abs(vy) < self.linear_deadband:
            vy = 0.0
        if abs(wz) < self.angular_deadband:
            wz = 0.0

        with self._lock:
            measured = None
            if self._last_odom_time is not None:
                measured = (now - self._last_odom_time).to_sec()
            self._last_odom_time = now
            dt = hw.feedback_dt(
                measured, 1.0 / self.feedback_rate, self.feedback_dt_max)

            yaw_rate = wz
            if (self.use_imu_yaw and self._gyro is not None and
                    (now - self._gyro_time).to_sec() <= self.imu_timeout):
                if (abs(wz) < self.angular_deadband and
                        abs(self._gyro) < self.imu_gyro_bias_limit):
                    self._gyro_bias = hw.update_bias(
                        self._gyro_bias, self._gyro,
                        self.imu_gyro_bias_alpha)
                yaw_rate = hw.blend_rates(
                    wz, self._gyro - self._gyro_bias, self.imu_gyro_gain)

            yaw = hw.wrap_radians(self._pose[2] + yaw_rate * dt)
            self._pose[0] += (
                vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            self._pose[1] += (
                vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            self._pose[2] = yaw
            self._last_feedback = (vx, vy, wz)
            self._last_feedback_time = now
            if (abs(vx) >= 0.01 or abs(vy) >= 0.01 or
                    abs(wz) >= 0.01):
                self._last_moving_feedback_time = now

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
            self._stop_burst()
        message = 'enabled' if req.data else 'disabled'
        rospy.loginfo('chassis: motors %s', message)
        return SetBoolResponse(success=True, message=message)

    def _reset_cb(self, _req):
        now = rospy.Time.now()
        with self._lock:
            self._pose = [self.initial_x, self.initial_y, self.initial_yaw]
            self._last_odom_time = now
            self._last_feedback = (0.0, 0.0, 0.0)
            self._last_feedback_time = now
            self._last_moving_feedback_time = now
            self._nonzero_cmd_start = None
        rospy.loginfo(
            'chassis: odometry reset to (%.3f, %.3f, %.3f)',
            self.initial_x, self.initial_y, self.initial_yaw)
        return EmptyResponse()

    def _stop_motors(self):
        if self.serial.is_open:
            self.serial.write(hw.encode_chassis_command(0.0, 0.0, 0.0))

    def _stop_burst(self, count=20):
        """Send a short burst of zero frames (firmware/UART may drop one)."""
        for _ in range(count):
            self._stop_motors()
            time.sleep(0.01)

    def _shutdown(self):
        with self._lock:
            self._enabled = False
            self._cmd = (0.0, 0.0, 0.0)
        self._stop_burst()
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

