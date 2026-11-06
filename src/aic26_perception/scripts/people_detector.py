"""
Community people detector.

Runs an int8 ONNX model on the camera stream and counts the people found in
the current view. Every person model has its own class, so each class is
counted at most once (no double counting within one detection).

    sub : /camera/image_raw              (sensor_msgs/Image)
    sub : /perception/detect             (std_msgs/String, JSON or legacy text)
    pub : /perception/result             (std_msgs/String, json)
    pub : /perception/people/image       (sensor_msgs/Image, debug)
    pub : /perception/people/protocol    (std_msgs/String, latched, json)

A request is observed over a short window that starts with the first
decoded camera frame. Each frame is counted on its own (the same class is
counted once per frame); classes from different frames are never added
together. The final result (``complete=true``) is published exactly once at
the end of the window and uses the mode of the per-frame counts. Intermediate
frames are published as ``complete=false`` for the GUI.

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
from perception_common import (SCHEMA_VERSION, YoloNet,
                               parse_detect_command, to_imgmsg)  # noqa: E402

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
        # The window starts with the first frame, not with the request, so a
        # cold camera does not consume the observation budget.
        self.observe_time = float(rospy.get_param('~observe_time', 0.8))
        self.min_frames = max(1, int(rospy.get_param('~min_frames', 2)))

        self.bridge = CvBridge()
        self.net = YoloNet(self.model_path, size=self.size, conf=self.conf,
                           nms=self.nms, num_classes=len(CLASSES),
                           log=self._log)

        self.active = False       # only detect while "people" is requested
        self.request_id = None    # v2 request id, None for legacy commands
        self.session = 0          # invalidates callbacks from older requests
        self.finalized = False    # the one complete result already went out
        # Per-frame snapshots.  The final count is the mode of these counts;
        # classes from different frames are never added together.
        self.frame_counts = []
        self.frame_snapshots = []
        self.frames = 0           # frames processed in this request
        self.first_frame_time = None
        self.window_end = None
        self.state_lock = threading.Lock()
        self.image_lock = threading.Lock()
        self.debug_warned = False  # log the first debug-image failure only

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/people/image'),
            Image, queue_size=1)
        self.protocol_pub = rospy.Publisher(
            rospy.get_param('~protocol_topic', '/perception/people/protocol'),
            String, queue_size=1, latch=True)

        self.image_topic = rospy.get_param('~image_in', '/camera/image_raw')
        self.image_sub = None
        rospy.Subscriber(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, self.on_detect)
        self.protocol_pub.publish(self._protocol_msg())
        # Also finalise on a timer: the window may end between two camera
        # frames, and waiting for the next frame would add latency.
        self.finish_timer = rospy.Timer(
            rospy.Duration(0.1), self._on_finish_timer)
        rospy.loginfo("people_detector ready (%d classes, observe %.1fs/%d)",
                      len(CLASSES), self.observe_time, self.min_frames)

    @staticmethod
    def _log(level, msg, *args):
        getattr(rospy, 'log' + level, rospy.loginfo)(msg, *args)

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'people_int8.onnx')

    def _protocol_msg(self):
        out = String()
        out.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'people',
            'protocol': 'v2',
            'request_id': True,
            'complete': True,
            'observation': True,
        })
        return out

    def infer(self, frame):
        """Return (class_name, score, box) detections for ``frame``."""
        dets = []
        for cls_id, score, box in self.net.infer(frame):
            if 0 <= cls_id < len(CLASSES):
                dets.append((CLASSES[cls_id], score, box))
        return dets

    def on_detect(self, msg):
        cmd = parse_detect_command(msg.data)
        target = cmd.get('target')
        request_id = cmd.get('request_id')
        if target == 'people':
            with self.state_lock:
                self.session += 1
                self.active = True
                self.request_id = request_id
                self.finalized = False
                self.frame_counts = []
                self.frame_snapshots = []
                self.frames = 0
                self.first_frame_time = None
                self.window_end = None
                self.last_infer = rospy.Time(0)
                session = self.session
            self._enable_image()
            rospy.loginfo("people request started (session %d, id %s)",
                          session, request_id or 'legacy')
            return

        if target == 'stop':
            with self.state_lock:
                current = self.request_id
                if request_id is not None and request_id != current:
                    rospy.logwarn("people: ignoring stale stop (%s != %s)",
                                  request_id, current)
                    return
                if request_id is None and current is not None:
                    rospy.logwarn("people: ignoring legacy stop while "
                                  "request %s is active", current)
                    return
                self.session += 1
                self.active = False
                self.request_id = None
                self.finalized = False
            self._disable_image()
            return

        # A request for another detector switches this one off.
        with self.state_lock:
            if self.active:
                self.session += 1
                self.active = False
                self.request_id = None
                self.finalized = False
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
        with self.state_lock:
            if not self.active or self.finalized or not self.net.ready:
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

        with self.state_lock:
            if (session != self.session or not self.active or
                    self.finalized or request_id != self.request_id):
                return
            if self.first_frame_time is None:
                self.first_frame_time = now
                self.window_end = now + rospy.Duration(self.observe_time)
            self.frames += 1
            frame_seen = sorted({name for name, _, _ in dets})
            frame_boxes = {}
            for name, _, box in dets:
                frame_boxes.setdefault(name, []).append(
                    [round(float(v), 1) for v in box])
            self.frame_counts.append(len(frame_seen))
            self.frame_snapshots.append({
                'seen': frame_seen,
                'boxes': frame_boxes,
            })
            frame_counts = list(self.frame_counts)
            frames = self.frames
            observe_s = (now - self.first_frame_time).to_sec()

        self.publish_result(frame_seen, frame_boxes, frame_counts, frames,
                            observe_s, complete=False, session=session,
                            request_id=request_id)
        self.publish_debug(frame, dets)
        self._maybe_finalize()

    def _on_finish_timer(self, _event):
        try:
            self._maybe_finalize()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("people_detector: finalise failed: %s", err)

    def _maybe_finalize(self):
        with self.state_lock:
            if (not self.active or self.finalized or
                    self.first_frame_time is None):
                return
            if self.frames < self.min_frames:
                return
            if rospy.Time.now() < self.window_end:
                return
            counts = list(self.frame_counts)
            frequency = {}
            for count in counts:
                frequency[count] = frequency.get(count, 0) + 1
            best_frequency = max(frequency.values())
            mode_count = max(count for count, number in frequency.items()
                             if number == best_frequency)
            matching = [i for i, count in enumerate(counts)
                        if count == mode_count]
            selected_index = matching[-1]
            snapshot = self.frame_snapshots[selected_index]
            seen = list(snapshot['seen'])
            boxes = {name: [list(box) for box in class_boxes]
                     for name, class_boxes in snapshot['boxes'].items()}
            self.finalized = True
            session = self.session
            request_id = self.request_id
            frames = self.frames
            selected_frame = selected_index + 1
            observe_s = (rospy.Time.now() -
                         self.first_frame_time).to_sec()

        self._publish_payload(seen, boxes, frames, observe_s,
                              complete=True, session=session,
                              request_id=request_id,
                              frame_counts=counts,
                              selected_frame=selected_frame)
        self._disable_image()
        rospy.loginfo("people request finished (session %d, id %s, "
                      "%d frames, counts=%s, mode=%d, selected=%d)",
                      session, request_id or 'legacy', frames, counts,
                      mode_count, selected_frame)

    def publish_result(self, seen, boxes, frame_counts, frames, observe_s,
                       complete, session, request_id):
        with self.state_lock:
            if (session != self.session or request_id != self.request_id or
                    not self.active):
                return
        self._publish_payload(seen, boxes, frames, observe_s,
                              complete=complete, session=session,
                              request_id=request_id,
                              frame_counts=frame_counts,
                              selected_frame=None)

    def _publish_payload(self, seen, boxes, frames, observe_s,
                         complete, session, request_id,
                         frame_counts=None, selected_frame=None):
        intruders = sorted(c for c in seen if c in INTRUDERS)
        community = len(seen) - len(intruders)
        out = String()
        out.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'people',
            'request_id': request_id,
            'session': session,
            'complete': bool(complete),
            'total': len(seen),
            'community': community,
            'intruder': len(intruders),
            'intruders': intruders,
            'boxes': boxes,
            'frames': frames,
            'observe_s': round(float(observe_s), 3),
            'frame_counts': list(frame_counts or []),
            'selected_frame': selected_frame,
        })
        self.result_pub.publish(out)

    def publish_debug(self, frame, dets):
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
