"""Protocol helpers shared by the aic26_cobot hardware nodes.

The encoders and decoders below are deliberately free of rospy imports so
they can be unit-tested without a ROS master. The only optional import is
pyserial, used by :class:`SerialDevice`; if pyserial is missing the pure
functions still import and the nodes report a clear error when they open
the port.
"""

import math
import struct

try:
    import serial
except ImportError:  # pragma: no cover - exercised on a bare Python
    serial = None

CHASSIS_HEADER = b'\xaa\xbb'
# Command: AA BB 0A 12 02 + x,y,yaw + checksum
CHASSIS_COMMAND = b'\xaa\xbb\x0a\x12\x02'
# Feedback: AA BB 0A 0A + x,y,yaw + filler + checksum
CHASSIS_FEEDBACK_HEADER = b'\xaa\xbb\x0a\x0a'
CHASSIS_FEEDBACK_SIZE = 12
LIDAR_HEADER = b'\xa5\x5a'
LIDAR_PACKET_SIZE = 108
LIDAR_POINTS = 16
WIT_HEADER = 0x55
WIT_FRAME_SIZE = 11


def clamp(value, low, high):
    """Return value limited to the inclusive [low, high] interval."""
    return max(low, min(high, value))


def feedback_dt(measured, nominal, max_dt):
    """Return the integration interval for one chassis feedback frame.

    The STM32 sends a 12-byte frame at a fixed rate, but the USB/serial
    adapter delivers them in bursts: measured arrival intervals alternate
    between almost zero and about twice the frame period. Using the wall
    clock interval for every frame therefore integrates several samples
    with dt ~= 0 and then one sample with the whole gap, which shows up as
    pose jumps in RViz.

    Use the nominal frame period for normal arrivals and only fall back to
    the measured interval when frames were actually dropped.
    """
    nominal = max(1e-3, float(nominal))
    if measured is None or measured <= 0.0:
        return nominal
    if measured > 2.5 * nominal:
        return min(float(measured), max(nominal, float(max_dt)))
    return nominal


def imu_yaw_offset(imu_yaw, odom_yaw):
    """Return the offset that maps the IMU yaw onto the odometry yaw.

    The odometry may start at a non-zero yaw (the Gazebo spawn yaw pi).
    Subtracting the odom yaw keeps ``imu_yaw - offset == odom_yaw`` at the
    start, so enabling the IMU does not slowly rotate the robot back to 0.
    """
    return wrap_radians(imu_yaw - odom_yaw)


def imu_yaw_step(error, gain, deadband, max_step):
    """Return a limited yaw correction for the IMU/odometry fusion.

    Small errors are ignored (deadband) and one correction is clamped so
    that a noisy magnetic heading cannot move the odometry TF in a single
    high-frequency step.
    """
    if abs(error) <= abs(deadband):
        return 0.0
    return clamp(gain * error, -abs(max_step), abs(max_step))


def wrap_degrees(angle):
    """Bring an angle in degrees into [0, 360)."""
    return angle % 360.0


def wrap_radians(angle):
    """Bring an angle in radians into (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def checksum8(data):
    """Return the low byte of the sum of data."""
    return sum(bytearray(data)) & 0xff


def quaternion_from_rpy(roll, pitch, yaw):
    """Convert roll, pitch, yaw in radians to (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def encode_chassis_command(linear_x, linear_y, angular_z,
                           max_linear=None, max_angular=None):
    """Build the 12-byte frame understood by the STM32 chassis.

    linear_x and linear_y are m/s, angular_z is rad/s. The chassis expects
    millimetres per second and milliradians per second as signed 16-bit
    little-endian values.
    """
    if max_linear is not None:
        linear_x = clamp(linear_x, -max_linear, max_linear)
        linear_y = clamp(linear_y, -max_linear, max_linear)
    if max_angular is not None:
        angular_z = clamp(angular_z, -max_angular, max_angular)

    x = clamp(int(round(linear_x * 1000.0)), -32768, 32767)
    y = clamp(int(round(linear_y * 1000.0)), -32768, 32767)
    z = clamp(int(round(angular_z * 1000.0)), -32768, 32767)
    return CHASSIS_COMMAND + struct.pack('<hhh', x, y, z) + b'\x00'


def decode_chassis_feedback(frame):
    """Decode one 12-byte chassis feedback frame.

    Returns (linear_x, linear_y, angular_z) in m/s and rad/s, or None when
    the frame is malformed. The checksum is the low byte of the sum of
    bytes 2..9, which is what the firmware emits.
    """
    if len(frame) != CHASSIS_FEEDBACK_SIZE:
        return None
    if frame[0:4] != CHASSIS_FEEDBACK_HEADER:
        return None
    if checksum8(frame[2:10]) != frame[11]:
        return None

    x, y, z = struct.unpack('<hhh', frame[4:10])
    return x / 1000.0, y / 1000.0, z / 1000.0


def parse_n10p_packet(packet):
    """Parse one 108-byte Leishen N10_P single-echo packet.

    Returns (start_deg, end_deg, points) where points is a list of
    (angle_deg, range_m, intensity) tuples, or None on a bad packet.

    The N10_P stores the range in the low 15 bits of the first byte plus
    the second byte; the high bit of the first byte is the intensity flag.
    The angle is interpolated over the 16 points of the packet, matching
    the vendor driver's full-packet interpolation.
    """
    if len(packet) != LIDAR_PACKET_SIZE:
        return None
    if packet[0:2] != LIDAR_HEADER:
        return None
    if checksum8(packet[:LIDAR_PACKET_SIZE - 1]) != packet[-1]:
        return None

    start = (packet[5] * 256 + packet[6]) / 100.0
    end = (packet[105] * 256 + packet[106]) / 100.0
    delta = end - start
    if delta < 0.0:
        delta += 360.0

    points = []
    for index in range(LIDAR_POINTS):
        offset = 7 + index * 2
        raw = packet[offset] * 256 + packet[offset + 1]
        if raw == 0xffff:
            continue
        distance_mm = (packet[offset] & 0x7f) * 256 + packet[offset + 1]
        intensity = 1.0 if packet[offset] & 0x80 else 0.0
        angle = wrap_degrees(start + delta * index / LIDAR_POINTS)
        points.append((angle, distance_mm / 1000.0, intensity))
    return start, end, points


def parse_wit_frame(frame):
    """Parse one 11-byte WIT normal-protocol frame.

    Returns a dict with type and values, or None on a bad frame. The WIT
    module emits 0x51 acceleration, 0x52 angular rate, 0x53 Euler angle
    and 0x54 magnetic field frames.
    """
    if len(frame) != WIT_FRAME_SIZE or frame[0] != WIT_HEADER:
        return None
    if checksum8(frame[:10]) != frame[10]:
        return None

    kind = frame[1]
    values = struct.unpack('<hhhh', frame[2:10])
    if kind == 0x51:
        accel = [v / 32768.0 * 16.0 * 9.80665 for v in values[:3]]
        return {'type': 'accel', 'values': accel,
                'temperature': values[3]}
    if kind == 0x52:
        gyro = [v / 32768.0 * 2000.0 * math.pi / 180.0
                for v in values[:3]]
        return {'type': 'gyro', 'values': gyro,
                'temperature': values[3]}
    if kind == 0x53:
        angles = [v / 32768.0 * 180.0 for v in values[:3]]
        return {'type': 'angle', 'values': angles, 'version': values[3]}
    if kind == 0x54:
        return {'type': 'mag', 'values': list(values[:3])}
    return {'type': 'unknown', 'kind': kind, 'values': list(values)}


class SerialDevice(object):
    """Small reconnect-friendly wrapper around pyserial."""

    def __init__(self, port, baud, timeout=0.05):
        if serial is None:
            raise RuntimeError(
                'pyserial is not installed; install python3-serial')
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = None
        self.last_error = ''

    @property
    def is_open(self):
        return self._serial is not None and self._serial.is_open

    def open(self):
        if self.is_open:
            return True
        try:
            self._serial = serial.Serial(
                self.port, self.baud, timeout=self.timeout,
                write_timeout=0.5)
            self._serial.reset_input_buffer()
            self.last_error = ''
            return True
        except (serial.SerialException, OSError, ValueError) as exc:
            self._serial = None
            self.last_error = str(exc)
            return False

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def read(self, size=1):
        # Keep a local reference: close() may set self._serial to None from
        # another thread between the is_open check and the read.
        device = self._serial
        if device is None or not device.is_open:
            return b''
        try:
            return device.read(size)
        except (serial.SerialException, OSError, TypeError) as exc:
            self.last_error = str(exc)
            self.close()
            return b''

    def write(self, data):
        device = self._serial
        if device is None or not device.is_open:
            return False
        try:
            device.write(data)
            device.flush()
            return True
        except (serial.SerialException, OSError, TypeError) as exc:
            self.last_error = str(exc)
            self.close()
            return False

