"""
Community people detector.

Runs an int8 ONNX model on the camera stream and counts the people found in
the current view. Every person model has its own class, so each class is
counted at most once (no double counting within one detection).

    sub : /camera/image_raw              (sensor_msgs/Image)
    sub : /perception/detect             (std_msgs/String, "people" starts a
                                          count, anything else stops it)
    pub : /perception/result             (std_msgs/String, json)
    pub : /perception/people/image       (sensor_msgs/Image, debug)

The int8 ONNX model is included in the package. To train a replacement, use
the classes below, export it to int8 ONNX and place it at:

    aic26_perception/models/people_int8.onnx

Model classes (fixed order):
    p1 .. p16, pF1, pF2

pF1 and pF2 are the non-community people, all the others belong to the
community.
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

# the shared YOLO backend/helpers live next to this script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_common import YoloNet, to_imgmsg      # noqa: E402

CLASSES = ['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9',
           'p10', 'p11', 'p12', 'p13', 'p14', 'p15', 'p16', 'pF1', 'pF2']

INTRUDERS = {'pF1', 'pF2'}

COLOR_COMMUNITY = (0, 255, 0)
COLOR_INTRUDER = (0, 0, 255)


class PeopleDetector(object):
    def __init__(self):
        self.model_path = rospy.get_param(
            '~model_path', self.default_model_path())
        self.size = rospy.get_param('~input_size', 640)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)
        # Inference is ~100 ms on the N100; processing every 30 FPS camera
        # frame would just build a queue.  5 Hz is plenty for a detection
        # dwell and leaves CPU for navigation and the other nodes.
        self.max_rate = float(rospy.get_param('~max_rate', 5.0))
        self.last_infer = rospy.Time(0)

        self.bridge = CvBridge()
        self.net = YoloNet(self.model_path, size=self.size, conf=self.conf,
                           nms=self.nms, num_classes=len(CLASSES),
                           log=self._log)

        self.seen = set()     # classes already counted in this detection
        self.active = False   # only detect while "people" is requested
        self.debug_warned = False   # log the first debug-image failure only

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/people/image'),
            Image, queue_size=1)

        # The image subscription is created on demand (see _set_active): a
        # 1280x720 raw frame is ~2.8 MB and rospy deserializes every message
        # before the callback can drop it, so an idle subscriber wastes CPU.
        self.image_in = rospy.get_param('~image_in', '/camera/image_raw')
        self.image_sub = None
        self.image_lock = threading.Lock()
        rospy.Subscriber(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, self.on_detect)
        rospy.loginfo("people_detector ready (%d classes)", len(CLASSES))

    @staticmethod
    def _log(level, msg, *args):
        getattr(rospy, 'log' + level, rospy.loginfo)(msg, *args)

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'people_int8.onnx')

    def infer(self, frame):
        """Return (class_name, score, box) detections for ``frame``."""
        dets = []
        for cls_id, score, box in self.net.infer(frame):
            if 0 <= cls_id < len(CLASSES):
                dets.append((CLASSES[cls_id], score, box))
        return dets

    def _set_active(self, active):
        """Subscribe to the camera only while this detector is requested."""
        self.active = active
        with self.image_lock:
            if active and self.image_sub is None:
                self.image_sub = rospy.Subscriber(
                    self.image_in, Image, self.on_image, queue_size=1)
            elif not active and self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None

    def on_detect(self, msg):
        # only the node whose target was requested runs; a request for another
        # target (or "stop") switches this one off again
        req = msg.data.strip().lower()
        if req == 'people':
            self.seen = set()
            self.last_infer = rospy.Time(0)
            self._set_active(True)
            rospy.loginfo("people count restarted")
        else:
            self._set_active(False)

    def on_image(self, msg):
        if not self.active:
            return
        if not self.net.ready:
            return
        now = rospy.Time.now()
        if (self.max_rate > 0.0 and
                (now - self.last_infer).to_sec() < 1.0 / self.max_rate):
            return
        self.last_infer = now

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        dets = self.infer(frame)
        self.update_seen(dets)
        self.publish_result(dets)
        self.publish_debug(frame, dets)

    def update_seen(self, dets):
        for name, _, _ in dets:
            self.seen.add(name)

    def publish_result(self, dets):
        intruders = sorted(c for c in self.seen if c in INTRUDERS)
        community = len(self.seen) - len(intruders)
        # one entry per detection, keyed by class name (a class may be seen
        # several times in one frame, so the value is a list of boxes)
        boxes = {}
        for name, _, box in dets:
            boxes.setdefault(name, []).append(
                [round(float(v), 1) for v in box])

        out = String()
        out.data = json.dumps({
            'target': 'people',
            'total': len(self.seen),
            'community': community,
            'intruder': len(intruders),
            'intruders': intruders,
            'boxes': boxes,
        })
        self.result_pub.publish(out)

    def publish_debug(self, frame, dets):
        if self.image_pub.get_num_connections() == 0:
            return
        for name, score, box in dets:
            color = COLOR_INTRUDER if name in INTRUDERS else COLOR_COMMUNITY
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, '%s %.2f' % (name, score), (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        try:
            self.image_pub.publish(to_imgmsg(frame))
            self.debug_warned = False
        except Exception as err:                       # noqa: BLE001
            # the debug stream must never break the detector, but a persistent
            # failure (e.g. a broken cv_bridge) has to be visible, not silent
            if not self.debug_warned:
                self.debug_warned = True
                rospy.logwarn("people_detector: debug publish skipped: %s",
                              err)


def main():
    rospy.init_node('people_detector')
    PeopleDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

