"""
Traffic light state detector.

Runs an int8 ONNX model on the camera stream and publishes the light state.

    sub : /camera/image_raw                    (sensor_msgs/Image)
    sub : /perception/detect                   (std_msgs/String, JSON or text)
    pub : /perception/traffic_light            (aic26_msgs/TrafficLightState,
                                                 latched, kept for old clients)
    pub : /perception/traffic_light/event      (std_msgs/String, json, v2)
    pub : /perception/traffic_light/image      (sensor_msgs/Image, debug)
    pub : /perception/traffic_light/protocol   (std_msgs/String, latched, json)

The reported ``model_name`` comes from the ``~node_name`` parameter and has to
match the traffic light model in the world file (``traffic_light_1`` /
``traffic_light_2``), so the mission controller can key the state by light.

The int8 ONNX model is included in the package. To train a replacement, use
the classes below, export it to int8 ONNX and place it at:

    aic26_perception/models/traffic_int8.onnx

Model classes (fixed order):
    0: red_off
    1: red_on
    2: green_off
    3: green_on
    4: yellow_off
    5: yellow_on

A traffic light shows its three lenses at the same time, so every lens is
detected and the light state is the color whose lens is "on".
"""

import json
import os
import sys
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from aic26_msgs.msg import TrafficLightState

# the shared YOLO backend/helpers live next to this script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_common import (SCHEMA_VERSION, YoloNet,
                               parse_detect_command, to_imgmsg)  # noqa: E402

CLASSES = ['red_off', 'red_on',
           'green_off', 'green_on',
           'yellow_off', 'yellow_on']

COLORS = {'red': (0, 0, 255),
          'green': (0, 255, 0),
          'yellow': (0, 255, 255)}

STATE_MAP = {'red': 'RED',
             'green': 'GREEN',
             'yellow': 'YELLOW'}


def parse_class(name):
    # 'red_on' -> ('red', True)
    color, _, flag = name.rpartition('_')
    return color, flag == 'on'


class TrafficLightDetector(object):
    def __init__(self):
        self.model_path = rospy.get_param(
            '~model_path', self.default_model_path())
        self.size = rospy.get_param('~input_size', 640)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)
        self.max_rate = float(rospy.get_param('~max_rate', 5.0))
        self.last_infer = rospy.Time(0)

        # model_name reported to the mission controller; must match the light
        # model in the world file, e.g. traffic_light_1 / traffic_light_2
        self.model_name = rospy.get_param('~node_name', 'traffic_light')

        self.bridge = CvBridge()
        self.debug_warned = False   # log the first debug-image failure only
        self.active = False         # only run while "traffic_light" is asked
        self.request_id = None
        self.session = 0
        self.state_lock = threading.Lock()
        self.image_lock = threading.Lock()
        self.net = YoloNet(self.model_path, size=self.size, conf=self.conf,
                           nms=self.nms, num_classes=len(CLASSES),
                           log=self._log)

        self.state_pub = rospy.Publisher(
            rospy.get_param('~state_topic', '/perception/traffic_light'),
            TrafficLightState, queue_size=10, latch=True)
        self.event_pub = rospy.Publisher(
            rospy.get_param('~event_topic',
                            '/perception/traffic_light/event'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/traffic_light/image'),
            Image, queue_size=1)
        self.protocol_pub = rospy.Publisher(
            rospy.get_param(
                '~protocol_topic', '/perception/traffic_light/protocol'),
            String, queue_size=1, latch=True)

        self.image_topic = rospy.get_param('~image_in', '/camera/image_raw')
        self.image_sub = None
        rospy.Subscriber(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, self.on_detect)
        self.protocol_pub.publish(self._protocol_msg())
        # A latched message can be missed when a subscriber connects after the
        # initial publish on some real-car startup schedules. Re-publishing
        # keeps the protocol handshake deterministic without affecting results.
        self.protocol_timer = rospy.Timer(
            rospy.Duration(1.0), self._republish_protocol)
        rospy.loginfo("traffic_light_detector ready (classes: %s)",
                      ','.join(CLASSES))

    def _republish_protocol(self, _event):
        self.protocol_pub.publish(self._protocol_msg())

    @staticmethod
    def _log(level, msg, *args):
        getattr(rospy, 'log' + level, rospy.loginfo)(msg, *args)

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'traffic_int8.onnx')

    def _protocol_msg(self):
        out = String()
        out.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'traffic_light',
            'protocol': 'v2',
            'request_id': True,
            'events': True,
        })
        return out

    def infer(self, frame):
        """Return (class_name, score, box) detections for ``frame``."""
        dets = []
        for cls_id, score, box in self.net.infer(frame):
            if 0 <= cls_id < len(CLASSES):
                dets.append((CLASSES[cls_id], score, box))
        return dets

    def light_state(self, dets):
        # the state is the color whose lens is detected as "on"
        on_colors = set()
        for name, _, _ in dets:
            color, is_on = parse_class(name)
            if is_on:
                on_colors.add(color)
        if len(on_colors) == 1:
            return STATE_MAP[on_colors.pop()]
        return 'UNKNOWN'

    def on_detect(self, msg):
        cmd = parse_detect_command(msg.data)
        target = cmd.get('target')
        request_id = cmd.get('request_id')
        if target == 'traffic_light':
            with self.state_lock:
                self.session += 1
                self.active = True
                self.request_id = request_id
                self.last_infer = rospy.Time(0)
                session = self.session
            self._enable_image()
            rospy.loginfo("traffic light request started (session %d, id %s)",
                          session, request_id or 'legacy')
            return

        if target == 'stop':
            with self.state_lock:
                current = self.request_id
                if request_id is not None and request_id != current:
                    rospy.logwarn("traffic: ignoring stale stop (%s != %s)",
                                  request_id, current)
                    return
                if request_id is None and current is not None:
                    rospy.logwarn("traffic: ignoring legacy stop while "
                                  "request %s is active", current)
                    return
                self.session += 1
                self.active = False
                self.request_id = None
            self._disable_image()
            return

        # A request for another detector switches this one off.
        with self.state_lock:
            if self.active:
                self.session += 1
                self.active = False
                self.request_id = None
        self._disable_image()

    def _enable_image(self):
        with self.image_lock:
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber(
                    self.image_topic, Image, self.on_image, queue_size=1)

    def _disable_image(self):
        with self.image_lock:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None

    def on_image(self, msg):
        # the light state is only inspected while the mission has a
        # traffic_light task at a stop line; between stops nothing is published
        with self.state_lock:
            if not self.active or not self.net.ready:
                return
            now = rospy.Time.now()
            if (self.max_rate > 0.0 and
                    (now - self.last_infer).to_sec() <
                    1.0 / self.max_rate):
                return
            self.last_infer = now
            session = self.session
            request_id = self.request_id

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        dets = self.infer(frame)
        state = self.light_state(dets)

        with self.state_lock:
            if (session != self.session or not self.active or
                    request_id != self.request_id):
                return
        self.publish_state(state, session, request_id)
        self.publish_debug(frame, dets)

    def publish_state(self, state, session=None, request_id=None):
        out = TrafficLightState()
        out.model_name = self.model_name
        out.state = state
        self.state_pub.publish(out)

        event = String()
        event.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'traffic_light',
            'model_name': self.model_name,
            'state': state,
            'request_id': request_id,
            'session': session,
        })
        self.event_pub.publish(event)

    def publish_debug(self, frame, dets):
        for name, score, box in dets:
            color, _ = parse_class(name)
            x, y, w, h = [int(v) for v in box]
            c = COLORS.get(color, (255, 255, 255))
            cv2.rectangle(frame, (x, y), (x + w, y + h), c, 2)
            cv2.putText(frame, '%s %.2f' % (name, score), (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        try:
            self.image_pub.publish(to_imgmsg(frame))
            self.debug_warned = False
        except Exception as err:                       # noqa: BLE001
            # the debug stream must never break the detector, but a persistent
            # failure (e.g. a broken cv_bridge) has to be visible, not silent
            if not self.debug_warned:
                self.debug_warned = True
                rospy.logwarn(
                    "traffic_light_detector: debug publish skipped: %s", err)


def main():
    rospy.init_node('traffic_light_detector')
    TrafficLightDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
