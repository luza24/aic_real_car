"""Unit tests for the checkpoint announcement text helpers.

The helpers are pure functions in mission_common; importing that module needs
the ROS Python packages, so the tests skip themselves outside a ROS shell.
"""

import os
import sys
import unittest

SCRIPTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')
sys.path.insert(0, SCRIPTS)

try:
    from mission_common import format_result_message, normalize_plate
    IMPORT_ERROR = None
except ImportError as err:          # no ROS packages on this interpreter
    format_result_message = None
    normalize_plate = None
    IMPORT_ERROR = err


class SpeechMessageTest(unittest.TestCase):
    def setUp(self):
        if IMPORT_ERROR is not None:
            raise unittest.SkipTest(
                'mission_common is not importable: %s' % IMPORT_ERROR)

    # -------------------------------------------------------------- people
    def test_people_with_intruders(self):
        payload = {'target': 'people', 'total': 12, 'intruder': 2}
        self.assertEqual(
            format_result_message('people', payload),
            '本区域共检测到 12 人，其中 2 人为非社区人员。')

    def test_people_without_intruders(self):
        payload = {'target': 'people', 'total': 12, 'intruder': 0}
        self.assertEqual(
            format_result_message('people', payload),
            '本区域共检测到 12 人，未发现非社区人员。')

    def test_people_zero(self):
        payload = {'target': 'people', 'total': 0}
        self.assertEqual(
            format_result_message('people', payload),
            '本区域共检测到 0 人，未发现非社区人员。')

    def test_people_without_total_is_silent(self):
        self.assertEqual(
            format_result_message('people', {'target': 'people'}), '')

    def test_people_bad_total_is_silent(self):
        self.assertEqual(
            format_result_message('people',
                                  {'target': 'people', 'total': 'many'}),
            '')

    # ----------------------------------------------------------------- car
    def test_car_plate_is_normalised(self):
        payload = {'target': 'car', 'total': 1, 'plate': '苏A·B8Q62'}
        self.assertEqual(
            format_result_message('car', payload),
            '检测到车辆，车牌号为 苏AB8Q62。')

    def test_car_without_plate(self):
        payload = {'target': 'car', 'total': 1, 'plate': ''}
        self.assertEqual(
            format_result_message('car', payload),
            '检测到车辆，但未能识别车牌。')

    def test_car_without_detection_is_silent(self):
        self.assertEqual(
            format_result_message('car',
                                  {'target': 'car', 'total': 0,
                                   'plate': ''}),
            '')

    # -------------------------------------------------------------- common
    def test_missing_payload_is_silent(self):
        self.assertEqual(format_result_message('people', None), '')
        self.assertEqual(format_result_message('car', {}), '')

    def test_unknown_target_is_silent(self):
        self.assertEqual(
            format_result_message('traffic_light',
                                  {'target': 'traffic_light',
                                   'state': 'GREEN'}),
            '')

    def test_english_template(self):
        payload = {'target': 'people', 'total': 3, 'intruder': 1}
        self.assertEqual(
            format_result_message('people', payload, 'en'),
            'Community has 3 people, 1 non-community people found.')

    def test_unknown_language_falls_back_to_chinese(self):
        payload = {'target': 'people', 'total': 1, 'intruder': 0}
        self.assertEqual(
            format_result_message('people', payload, 'fr'),
            '本区域共检测到 1 人，未发现非社区人员。')

    def test_normalize_plate(self):
        self.assertEqual(normalize_plate(' 苏A · B8Q62 '), '苏AB8Q62')
        self.assertEqual(normalize_plate(None), '')


if __name__ == '__main__':
    unittest.main()
