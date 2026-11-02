"""Protocol helpers shared by the aic26_cobot hardware nodes."""

import math
import struct

try:
    import serial
except ImportError:
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
LIDAR_POINT_SIZE = 6
WIT_HEADER = 0x55
WIT_FRAME_SIZE = 11


def clamp(value, low, high):
    """Return value limited to the inclusive [low, high] interval."""
    return max(low, min(high, value))


def feedback_dt(measured, nominal, max_dt):
    """Return the integration interval for one chassis feedback frame."""
    nominal = max(1e-3, float(nominal))
    if measured is None or measured <= 0.0:
        return nominal
    if measured > 2.5 * nominal:
        return min(float(measured), max(nominal, float(max_dt)))
    return nominal


def blend_rates(encoder_rate, gyro_rate, gyro_weight):
    """Return the yaw rate from the encoder rate and the gyro rate."""
    weight = clamp(gyro_weight, 0.0, 1.0)
    return (1.0 - weight) * encoder_rate + weight * gyro_rate


def update_bias(bias, sample, alpha):
    """Fold one standstill gyro sample into the bias estimate."""
    return bias + clamp(alpha, 0.0, 1.0) * (sample - bias)


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
    """Build the 12-byte command frame (mm/s and mrad/s, int16 LE)."""
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
    """Decode one 12-byte feedback frame into m/s and rad/s, or None."""
    if len(frame) != CHASSIS_FEEDBACK_SIZE:
        return None
    if frame[0:4] != CHASSIS_FEEDBACK_HEADER:
        return None
    if checksum8(frame[2:10]) != frame[11]:
        return None

    x, y, z = struct.unpack('<hhh', frame[4:10])
    return x / 1000.0, y / 1000.0, z / 1000.0


def parse_n10p_packet(packet):
    """Parse one 108-byte N10_P packet into (start_deg, end_deg, points)."""
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
        offset = 7 + index * LIDAR_POINT_SIZE
        distance_mm = packet[offset] * 256 + packet[offset + 1]
        if distance_mm == 0:
            continue
        intensity = float(packet[offset + 2])
        angle = wrap_degrees(start + delta * index / LIDAR_POINTS)
        points.append((angle, distance_mm / 1000.0, intensity))
    return start, end, points


def fill_gaps(ranges, max_holes=2, max_diff=0.2):
    """Fill short holes between two close returns; return the filled count."""
    if max_holes <= 0 or len(ranges) < 3:
        return 0
    filled = 0
    index = 0
    total = len(ranges)
    while index < total:
        if math.isfinite(ranges[index]):
            index += 1
            continue
        start = index
        while index < total and not math.isfinite(ranges[index]):
            index += 1
        holes = index - start
        if holes > max_holes or start == 0 or index >= total:
            continue
        left = ranges[start - 1]
        right = ranges[index]
        if not (math.isfinite(left) and math.isfinite(right)):
            continue
        if abs(left - right) > max_diff:
            continue
        for step in range(holes):
            weight = (step + 1.0) / (holes + 1.0)
            ranges[start + step] = left + (right - left) * weight
            filled += 1
    return filled


def parse_wit_frame(frame, accel_range_g=16.0, gyro_range_dps=2000.0):
    """Parse one 11-byte WIT frame; the full scale is configurable."""
    if len(frame) != WIT_FRAME_SIZE or frame[0] != WIT_HEADER:
        return None
    if checksum8(frame[:10]) != frame[10]:
        return None

    kind = frame[1]
    values = struct.unpack('<hhhh', frame[2:10])
    if kind == 0x51:
        accel = [v / 32768.0 * accel_range_g * 9.80665 for v in values[:3]]
        return {'type': 'accel', 'values': accel,
                'temperature': values[3]}
    if kind == 0x52:
        gyro = [v / 32768.0 * gyro_range_dps * math.pi / 180.0
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

