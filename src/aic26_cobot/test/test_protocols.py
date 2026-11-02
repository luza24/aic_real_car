"""Unit tests for the aic26_cobot protocol helpers."""

import math
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
        self.assertAlmostEqual(hw.feedback_dt(0.00002, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(0.049, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(None, 0.05, 0.2), 0.05)

    def test_feedback_dt_keeps_dropped_frames(self):
        self.assertAlmostEqual(hw.feedback_dt(0.118, 0.05, 0.2), 0.05)
        self.assertAlmostEqual(hw.feedback_dt(0.20, 0.05, 0.2), 0.20)
        self.assertAlmostEqual(hw.feedback_dt(1.0, 0.05, 0.2), 0.2)

    def test_blend_rates(self):
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 0.0), 1.0)
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 1.0), 2.0)
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 0.25), 1.25)
        self.assertAlmostEqual(hw.blend_rates(1.0, 2.0, 2.0), 2.0)

    def test_update_bias(self):
        self.assertAlmostEqual(hw.update_bias(0.0, 0.1, 0.5), 0.05)
        self.assertAlmostEqual(hw.update_bias(0.05, 0.1, 0.0), 0.05)
        self.assertAlmostEqual(hw.update_bias(0.05, 0.1, 2.0), 0.1)


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

    def test_accel_frame_custom_range(self):
        frame = self._frame(0x51, (16384, 0, 0, 25))
        parsed = hw.parse_wit_frame(frame, accel_range_g=2.0)
        self.assertAlmostEqual(parsed['values'][0], 2.0 * 9.80665 / 2.0)

    def test_gyro_frame_custom_range(self):
        frame = self._frame(0x52, (16384, 0, 0, 25))
        parsed = hw.parse_wit_frame(frame, gyro_range_dps=250.0)
        self.assertAlmostEqual(
            parsed['values'][0], 125.0 * math.pi / 180.0)

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
            offset = 7 + index * 6
            packet[offset] = (distance_mm >> 8) & 0xff
            packet[offset + 1] = distance_mm & 0xff
            packet[offset + 2] = 40 + index
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

    def test_real_packet_layout(self):
        # Captured from the N10_P: 16 points of 6 bytes, distance big endian
        # in millimetres, then the intensity, then the second echo.
        packet = bytes.fromhex(
            'a55a6c104106cb084248000000083ad50000'
            '000845dd000000083cde0000000845df0000'
            '000849e100000004a0a7083e2606a5790000'
            '0006c197000000085588046e44085262046c'
            '5713c37304794a13c672048534049f3313cc'
            '1b0420220bf5080417915e1d09000d0ac579')
        self.assertEqual(len(packet), 108)
        parsed = hw.parse_n10p_packet(packet)
        self.assertIsNotNone(parsed)
        start, end, points = parsed
        self.assertAlmostEqual(start, 17.39)
        self.assertAlmostEqual(end, 27.57)
        self.assertEqual(len(points), 16)
        self.assertEqual([round(p[1] * 1000) for p in points],
                         [2114, 2106, 2117, 2108, 2117, 2121, 1184, 1701,
                          1729, 2133, 2130, 5059, 5062, 1183, 1056, 1047])
        self.assertEqual([int(p[2]) for p in points],
                         [72, 213, 221, 222, 223, 225, 167, 121,
                          151, 136, 98, 115, 114, 51, 34, 145])
        angles = [p[0] for p in points]
        self.assertTrue(all(b > a for a, b in zip(angles[:-1], angles[1:])))
        self.assertAlmostEqual(angles[1] - angles[0], 10.18 / 16.0, places=2)

    def test_invalid_point_is_skipped(self):
        distances = [1000] * 16
        packet = bytearray(self._packet(0.0, 10.0, distances))
        packet[7:9] = b'\x00\x00'
        packet[107] = sum(packet[:107]) & 0xff
        _, _, points = hw.parse_n10p_packet(bytes(packet))
        self.assertEqual(len(points), 15)

    def test_bad_crc(self):
        packet = bytearray(self._packet(0.0, 10.0, [1000] * 16))
        packet[-1] ^= 0xff
        self.assertIsNone(hw.parse_n10p_packet(bytes(packet)))


class ScanFillTest(unittest.TestCase):

    def test_fills_short_hole(self):
        ranges = [1.0, float('inf'), 1.1]
        self.assertEqual(hw.fill_gaps(ranges, 2, 0.2), 1)
        self.assertAlmostEqual(ranges[1], 1.05)

    def test_keeps_wide_hole(self):
        ranges = [1.0, float('inf'), float('inf'), float('inf'), 1.1]
        self.assertEqual(hw.fill_gaps(ranges, 2, 0.2), 0)
        self.assertFalse(math.isfinite(ranges[2]))

    def test_skips_distance_jump(self):
        ranges = [1.0, float('inf'), 3.0]
        self.assertEqual(hw.fill_gaps(ranges, 2, 0.2), 0)

    def test_skips_open_ends(self):
        ranges = [float('inf'), 1.0, float('inf')]
        self.assertEqual(hw.fill_gaps(ranges, 2, 0.2), 0)
        self.assertFalse(math.isfinite(ranges[0]))
        self.assertFalse(math.isfinite(ranges[2]))

    def test_disabled(self):
        ranges = [1.0, float('inf'), 1.1]
        self.assertEqual(hw.fill_gaps(ranges, 0, 0.2), 0)


if __name__ == '__main__':
    unittest.main()
