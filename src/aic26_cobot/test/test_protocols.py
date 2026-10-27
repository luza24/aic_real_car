"""Unit tests for the aic26_cobot protocol helpers."""

import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))

import hardware_common as hw  # noqa: E402


class ChassisProtocolTest(unittest.TestCase):

    def test_command_frame(self):
        frame = hw.encode_chassis_command(0.1, -0.2, 0.3)
        self.assertEqual(len(frame), 12)
        self.assertEqual(frame[:5], b'\xaa\xbb\x0a\x12\x02')
        x, y, z = struct.unpack('<hhh', frame[5:11])
        self.assertEqual((x, y, z), (100, -200, 300))
        self.assertEqual(frame[11], 0)

    def test_command_limits(self):
        frame = hw.encode_chassis_command(
            10.0, -10.0, 10.0, max_linear=0.5, max_angular=1.0)
        x, y, z = struct.unpack('<hhh', frame[5:11])
        self.assertEqual((x, y, z), (500, -500, 1000))

    def test_feedback_round_trip(self):
        payload = struct.pack('<hhh', 120, -80, 45)
        frame = bytearray(b'\xaa\xbb\x0a\x0a' + payload + b'\x00\x00')
        frame[11] = sum(frame[2:10]) & 0xff
        decoded = hw.decode_chassis_feedback(bytes(frame))
        self.assertAlmostEqual(decoded[0], 0.12)
        self.assertAlmostEqual(decoded[1], -0.08)
        self.assertAlmostEqual(decoded[2], 0.045)

    def test_feedback_bad_checksum(self):
        payload = struct.pack('<hhh', 1, 2, 3)
        frame = bytearray(b'\xaa\xbb\x0a\x0a' + payload + b'\x00\x00')
        frame[11] = 0
        self.assertIsNone(hw.decode_chassis_feedback(bytes(frame)))

    def test_feedback_bad_header(self):
        payload = struct.pack('<hhh', 1, 2, 3)
        frame = bytearray(b'\xaa\xbb\x00\x00' + payload + b'\x00\x00')
        frame[11] = sum(frame[2:10]) & 0xff
        self.assertIsNone(hw.decode_chassis_feedback(bytes(frame)))

    def test_real_feedback_frame(self):
        # Captured from the STM32: feedback header is 0A 0A, not 0A 12.
        frame = bytes.fromhex('aabb0a0a0000000000000014')
        self.assertEqual(hw.decode_chassis_feedback(frame),
                         (0.0, 0.0, 0.0))


class ChassisTimingTest(unittest.TestCase):

    def test_feedback_dt_uses_nominal_for_bursts(self):
        # The serial adapter delivers a 20 Hz stream in bursts.
        self.assertAlmostEqual(hw.feedback_dt(0.00002, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(0.049, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(None, 0.05, 0.2), 0.05)

    def test_feedback_dt_keeps_dropped_frames(self):
        # A 0.118 s gap is still a delayed frame in the same burst, not a
        # dropped sample: keep the nominal period for smooth integration.
        self.assertAlmostEqual(hw.feedback_dt(0.118, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(0.20, 0.05, 0.2), 0.20)
        self.assertAlmostEqual(hw.feedback_dt(1.0, 0.05, 0.2), 0.2)

    def test_imu_yaw_step_deadband_and_limit(self):
        self.assertEqual(hw.imu_yaw_step(0.001, 0.1, 0.005, 0.02), 0.0)
        self.assertAlmostEqual(hw.imu_yaw_step(0.1, 0.1, 0.005, 0.02), 0.01)
        self.assertAlmostEqual(hw.imu_yaw_step(1.0, 0.1, 0.005, 0.02), 0.02)
        self.assertAlmostEqual(hw.imu_yaw_step(-1.0, 0.1, 0.005, 0.02),
                               -0.02)

    def test_gyro_blend_and_bias(self):
        self.assertAlmostEqual(hw.blend_rates(1.0, 0.0, 0.25), 0.75)
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 0.0), 1.0)
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 1.0), 2.0)
        self.assertAlmostEqual(hw.update_bias(1.0, 0.0, 0.5), 0.5)

    def test_imu_offset_preserves_start_yaw(self):
        odom_yaw = hw.wrap_radians(3.141592)
        offset = hw.imu_yaw_offset(-0.00489, odom_yaw)
        self.assertAlmostEqual(
            hw.wrap_radians(-0.00489 - offset), odom_yaw, places=9)
        offset = hw.imu_yaw_offset(3.0, -3.0)
        self.assertAlmostEqual(hw.wrap_radians(3.0 - offset), -3.0,
                               places=9)


class WitProtocolTest(unittest.TestCase):

    @staticmethod
    def _frame(kind, values):
        payload = struct.pack('<hhhh', *values)
        frame = bytearray(b'\x55' + bytes([kind]) + payload)
        frame.append(sum(frame) & 0xff)
        return bytes(frame)

    def test_angle_frame(self):
        frame = self._frame(0x53, (0, 0, 16384, 0))
        parsed = hw.parse_wit_frame(frame)
        self.assertEqual(parsed['type'], 'angle')
        self.assertAlmostEqual(parsed['values'][2], 90.0)

    def test_accel_frame(self):
        frame = self._frame(0x51, (16384, 0, 0, 25))
        parsed = hw.parse_wit_frame(frame)
        self.assertEqual(parsed['type'], 'accel')
        self.assertAlmostEqual(parsed['values'][0], 16.0 * 9.80665 / 2.0)
        self.assertEqual(parsed['temperature'], 25)

    def test_bad_checksum(self):
        frame = bytearray(self._frame(0x53, (0, 0, 0, 0)))
        frame[-1] ^= 0xff
        self.assertIsNone(hw.parse_wit_frame(bytes(frame)))


class LidarProtocolTest(unittest.TestCase):

    def _packet(self, start_deg, end_deg, distances):
        packet = bytearray(108)
        packet[0:2] = b'\xa5\x5a'
        start = int(round(start_deg * 100.0))
        end = int(round(end_deg * 100.0))
        packet[5] = start // 256
        packet[6] = start % 256
        packet[105] = end // 256
        packet[106] = end % 256
        for index, distance_mm in enumerate(distances):
            packet[7 + 2 * index] = (distance_mm >> 8) & 0x7f
            packet[8 + 2 * index] = distance_mm & 0xff
        packet[107] = sum(packet[:107]) & 0xff
        return bytes(packet)

    def test_packet(self):
        distances = [1000 + index * 10 for index in range(16)]
        packet = self._packet(10.0, 20.0, distances)
        start, end, points = hw.parse_n10p_packet(packet)
        self.assertAlmostEqual(start, 10.0)
        self.assertAlmostEqual(end, 20.0)
        self.assertEqual(len(points), 16)
        self.assertAlmostEqual(points[0][1], 1.0)
        self.assertAlmostEqual(points[-1][1], 1.15)
        self.assertGreaterEqual(points[0][0], 10.0)
        self.assertLess(points[-1][0], 20.0)

    def test_invalid_point_is_skipped(self):
        distances = [1000] * 16
        packet = bytearray(self._packet(0.0, 10.0, distances))
        packet[7] = 0xff
        packet[8] = 0xff
        packet[107] = sum(packet[:107]) & 0xff
        _, _, points = hw.parse_n10p_packet(bytes(packet))
        self.assertEqual(len(points), 15)

    def test_bad_crc(self):
        packet = bytearray(self._packet(0.0, 10.0, [1000] * 16))
        packet[-1] ^= 0xff
        self.assertIsNone(hw.parse_n10p_packet(bytes(packet)))


if __name__ == '__main__':
    unittest.main()
