"""
Car and license plate detector.

Runs an int8 ONNX model on the camera stream, looks for the single class
``car`` and reads the license plate of the car parked at the inspected
waypoint. Only the plate differs between the cars in this scene, so the
detector crops the plate region out of the detected car and recognises it
with easyocr (Chinese simplified + English, so the province character comes
out as real UTF-8 text).

    sub : /camera/image_raw              (sensor_msgs/Image)
    sub : /perception/detect             (std_msgs/String, JSON or legacy text)
    pub : /perception/result             (std_msgs/String, json)
    pub : /perception/car/image          (sensor_msgs/Image, debug)
    pub : /perception/car/protocol       (std_msgs/String, latched, json)
    pub : /perception/car/ocr_ready      (std_msgs/String, latched, json)

EasyOCR is loaded in a background worker at node start-up when
``~ocr_preload`` is true. Failure is only reported on the ready topic; it
must never stop the mission. The first car request can retry the load in its
OCR worker.

A car request publishes progress with ``complete=false`` and exactly one
final result with ``complete=true``. The final result can be a car without a
recognised plate, no car at all, or an OCR timeout; a late OCR result after
the final result is discarded.

The int8 ONNX model is included in the package. To train a replacement, use a
single ``car`` class, export it to int8 ONNX and place it at:

    aic26_perception/models/car_int8.onnx
"""

import json
import os
import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# the shared YOLO backend/helpers live next to this script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_common import (SCHEMA_VERSION, YoloNet, crop_roi,
                               parse_detect_command, to_imgmsg)  # noqa: E402

CLASSES = ['car']

# Plate is a fixed fraction of the rear view texture / detected car box.
PLATE_ROI = [0.30, 0.40, 0.70, 0.60]

# Province characters plus every legal plate letter/digit, I and O excluded.
PLATE_ALLOWLIST = ("苏沪京渝津冀晋蒙辽吉黑皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青"
                   "宁新ABCDEFGHJKLMNPQRSTUVWXYZ0123456789")

COLOR_CAR = (255, 200, 0)
COLOR_PLATE = (0, 255, 255)


def preprocess(crop, scale=3.0, binary=False):
    """Grayscale + cubic upscale; grayscale gave the best Chinese reads."""
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    if binary:
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 31, 10)
    if scale and scale != 1.0:
        interp = cv2.INTER_CUBIC if scale >= 2 else cv2.INTER_LINEAR
        gray = cv2.resize(gray, None, fx=scale, fy=scale,
                          interpolation=interp)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def tidy(text):
    """Drop the plate separator and any whitespace the recogniser injected."""
    if not text:
        return ''
    return ''.join(ch for ch in text
                   if not ch.isspace() and ch not in '·•.。,，:：-_/\\|')


class CarDetector(object):
    def __init__(self):
        self.model_path = rospy.get_param(
            '~model_path', self.default_model_path())
        self.size = rospy.get_param('~input_size', 640)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)
        self.max_rate = float(rospy.get_param('~max_rate', 5.0))
        self.last_infer = rospy.Time(0)

        # OCR settings
        self.plate_roi = rospy.get_param('~plate_roi', PLATE_ROI)
        self.ocr_scale = float(rospy.get_param('~ocr_scale', 3.0))
        self.ocr_binary = bool(rospy.get_param('~ocr_binary', False))
        self.ocr_gpu = bool(rospy.get_param('~ocr_gpu', False))
        self.ocr_min_chars = int(rospy.get_param('~ocr_min_chars', 4))
        self.ocr_preload = bool(rospy.get_param('~ocr_preload', True))
        self.ocr_num_threads = max(1, int(
            rospy.get_param('~ocr_num_threads', 2)))
        self.request_timeout = float(
            rospy.get_param('~request_timeout', 10.0))
        self.empty_observe_time = float(
            rospy.get_param('~empty_observe_time', 1.0))

        self.bridge = CvBridge()
        self.debug_warned = False   # log the first debug-image failure only
        self.net = YoloNet(self.model_path, size=self.size, conf=self.conf,
                           nms=self.nms, num_classes=len(CLASSES),
                           log=self._log)

        self.reader = None
        self.reader_lock = threading.Lock()
        self.reader_ready = False
        self.reader_status = 'idle'
        self.reader_reason = ''

        # Per-request state.  All fields are guarded by state_lock; inference,
        # OCR and ROS publishing never happen while holding it.
        self.active = False       # only detect while "car" is requested
        self.request_id = None
        self.session = 0
        self.finalized = False    # exactly one complete result per request
        self.seen = set()
        self.latest_boxes = {}
        self.plate = ''
        self.plate_conf = 0.0
        self.ocr_started = False
        self.ocr_pending = False
        self.ocr_status = 'idle'
        self.ocr_thread = None
        self.frames = 0
        self.first_frame_time = None
        self.request_start = None
        self.state_lock = threading.Lock()
        self.image_lock = threading.Lock()

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/car/image'),
            Image, queue_size=1)
        self.protocol_pub = rospy.Publisher(
            rospy.get_param('~protocol_topic', '/perception/car/protocol'),
            String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher(
            rospy.get_param('~ocr_ready_topic', '/perception/car/ocr_ready'),
            String, queue_size=1, latch=True)

        self.image_topic = rospy.get_param('~image_in', '/camera/image_raw')
        self.image_sub = None
        rospy.Subscriber(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, self.on_detect)

        self.protocol_pub.publish(self._protocol_msg())
        self._publish_ready(False, 'loading')
        if self.ocr_preload:
            self.preload_thread = threading.Thread(
                target=self._preload_worker, name='car_ocr_preload')
            self.preload_thread.daemon = True
            self.preload_thread.start()
        else:
            self._publish_ready(True, 'disabled')

        # Finalise on a timer as well: OCR may finish between two frames, and
        # the empty-car / request deadline must not depend on the next frame.
        self.finish_timer = rospy.Timer(
            rospy.Duration(0.1), self._on_finish_timer)
        rospy.loginfo("car_detector ready (class 'car', ocr_preload=%s, "
                      "easyocr=%s)", self.ocr_preload,
                      'loading' if self.ocr_preload else 'disabled')

    @staticmethod
    def _log(level, msg, *args):
        getattr(rospy, 'log' + level, rospy.loginfo)(msg, *args)

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'car_int8.onnx')

    # ---------------------------------------------------------- protocol/ocr
    def _protocol_msg(self):
        out = String()
        out.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'car',
            'protocol': 'v2',
            'request_id': True,
            'complete': True,
            'ocr_ready': True,
        })
        return out

    def _publish_ready(self, ready, status, reason=''):
        self.reader_ready = bool(ready)
        self.reader_status = status
        self.reader_reason = reason
        out = String()
        out.data = json.dumps({
            'schema': SCHEMA_VERSION,
            'target': 'car',
            'ready': bool(ready),
            'status': status,
            'reason': reason,
        })
        self.ready_pub.publish(out)

    def _preload_worker(self):
        try:
            self.load_reader()
        except Exception as err:                       # noqa: BLE001
            self._publish_ready(False, 'failed', str(err))
            rospy.logwarn("car_detector: easyocr preload failed: %s", err)
            return
        self._publish_ready(True, 'ready')
        rospy.loginfo("car_detector: easyocr preloaded")

    def load_reader(self):
        """Load EasyOCR once; failure is reported by the caller.

        The import and the Reader construction are both inside the lock so a
        preload worker and the first car request cannot build two readers.
        """
        with self.reader_lock:
            if self.reader is not None:
                self.reader_ready = True
                return self.reader
            if not self.ocr_gpu:
                os.environ['CUDA_VISIBLE_DEVICES'] = ''
            # These must be set before torch/easyocr imports; the process is
            # started fresh, so this is early enough.
            threads = str(self.ocr_num_threads)
            os.environ['OMP_NUM_THREADS'] = threads
            os.environ['MKL_NUM_THREADS'] = threads
            rospy.loginfo("car_detector: loading easyocr (CPU, %s threads) ...",
                          threads)
            import torch                                # noqa: F401
            import easyocr                              # noqa: F401
            try:
                torch.set_num_threads(self.ocr_num_threads)
                torch.set_num_interop_threads(1)
            except Exception as err:                   # noqa: BLE001
                rospy.logwarn("car_detector: torch thread cap failed: %s", err)
            self.reader = easyocr.Reader(
                ['ch_sim', 'en'], gpu=self.ocr_gpu, verbose=False)
            self.reader_ready = True
            self.reader_status = 'ready'
            rospy.loginfo("car_detector: easyocr ready")
            return self.reader

    def recognize(self, crop):
        """Return (text, conf, status) for the plate crop."""
        try:
            reader = self.load_reader()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: OCR unavailable: %s", err)
            return '', 0.0, 'failed'
        best = ('', 0.0)
        img = preprocess(crop, scale=self.ocr_scale, binary=self.ocr_binary)
        try:
            found = reader.readtext(img, detail=1,
                                    allowlist=PLATE_ALLOWLIST,
                                    paragraph=False)
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: readtext failed: %s", err)
            return best[0], best[1], 'failed'
        results = []
        for _, text, conf in found:
            clean = tidy(text)
            if len(clean) >= self.ocr_min_chars:
                results.append((clean, float(conf)))
        if results:
            # prefer the longest read, break ties on confidence
            results.sort(key=lambda r: (len(r[0]), r[1]), reverse=True)
            best = results[0]
        return best[0], best[1], 'ok' if best[0] else 'empty'

    # ------------------------------------------------------------- requests
    def _stop_request(self):
        with self.state_lock:
            if self.active:
                self.session += 1
                self.active = False
                self.request_id = None
                self.finalized = False
                self.ocr_pending = False
        self._disable_image()

    def on_detect(self, msg):
        cmd = parse_detect_command(msg.data)
        target = cmd.get('target')
        request_id = cmd.get('request_id')
        if target == 'car':
            with self.state_lock:
                self.session += 1
                self.active = True
                self.request_id = request_id
                self.finalized = False
                self.seen = set()
                self.latest_boxes = {}
                self.plate = ''
                self.plate_conf = 0.0
                self.ocr_started = False
                self.ocr_pending = False
                self.ocr_status = 'idle'
                self.ocr_thread = None
                self.frames = 0
                self.first_frame_time = None
                self.request_start = rospy.Time.now()
                self.last_infer = rospy.Time(0)
                session = self.session
            self._enable_image()
            rospy.loginfo("car request started (session %d, id %s)",
                          session, request_id or 'legacy')
            return

        if target == 'stop':
            with self.state_lock:
                current = self.request_id
                if request_id is not None and request_id != current:
                    rospy.logwarn("car: ignoring stale stop (%s != %s)",
                                  request_id, current)
                    return
                if request_id is None and current is not None:
                    rospy.logwarn("car: ignoring legacy stop while "
                                  "request %s is active", current)
                    return
            self._stop_request()
            return

        # A request for another detector switches this one off.
        self._stop_request()

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

    # ---------------------------------------------------------------- frames
    def infer(self, frame):
        """Return (class_name, score, box) detections for ``frame``."""
        dets = []
        for cls_id, score, box in self.net.infer(frame):
            if 0 <= cls_id < len(CLASSES):
                dets.append((CLASSES[cls_id], score, box))
        return dets

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

        start_ocr = None
        with self.state_lock:
            if (session != self.session or not self.active or
                    self.finalized or request_id != self.request_id):
                return
            if self.first_frame_time is None:
                self.first_frame_time = now
            self.frames += 1
            if dets:
                best = max(dets, key=lambda d: d[1])
                self.seen.add(best[0])
                self.latest_boxes = {}
                for name, _, box in dets:
                    self.latest_boxes.setdefault(name, []).append(
                        [round(float(v), 1) for v in box])
                if not self.ocr_started:
                    self.ocr_started = True
                    self.ocr_pending = True
                    self.ocr_status = 'pending'
                    start_ocr = best[2]

        if start_ocr is not None:
            self.read_plate_async(frame, start_ocr, session, request_id)
        self.publish_result(dets, complete=False, session=session,
                            request_id=request_id)
        self.publish_debug(frame, dets, session, request_id)
        self._maybe_finalize()

    # ------------------------------------------------------------------ ocr
    def read_plate_async(self, frame, box, session, request_id):
        """Crop the plate of ``box`` and OCR it on a worker thread."""
        x, y, w, h = [int(v) for v in box]
        x, y = max(0, x), max(0, y)
        w = min(w, frame.shape[1] - x)
        h = min(h, frame.shape[0] - y)
        if w < 8 or h < 8:
            with self.state_lock:
                if session == self.session and request_id == self.request_id:
                    self.ocr_pending = False
                    self.ocr_status = 'failed'
            self._maybe_finalize(session, request_id)
            return
        car = frame[y:y + h, x:x + w]
        crop = crop_roi(car, self.plate_roi)
        if crop.size == 0:
            with self.state_lock:
                if session == self.session and request_id == self.request_id:
                    self.ocr_pending = False
                    self.ocr_status = 'failed'
            self._maybe_finalize(session, request_id)
            return
        crop = crop.copy()

        with self.state_lock:
            if (session != self.session or request_id != self.request_id or
                    not self.active or self.finalized or
                    self.ocr_thread is not None):
                return
            self.ocr_thread = threading.Thread(
                target=self._run_ocr, args=(crop, session, request_id),
                name='car_plate_ocr')
            self.ocr_thread.daemon = True
            self.ocr_thread.start()

    def _run_ocr(self, crop, session, request_id):
        t0 = time.time()
        try:
            text, conf, status = self.recognize(crop)
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: OCR failed: %s", err)
            text, conf, status = '', 0.0, 'failed'

        current = False
        with self.state_lock:
            if (session == self.session and request_id == self.request_id and
                    self.active and not self.finalized):
                self.plate, self.plate_conf = text, conf
                self.ocr_status = status
                self.ocr_pending = False
                current = True
            if self.ocr_thread is threading.current_thread():
                self.ocr_thread = None

        if current:
            rospy.loginfo("car_detector: plate '%s' (%.2f, %s, %.2fs)",
                          text, conf, status, time.time() - t0)
        else:
            rospy.loginfo("car_detector: discarding stale OCR result "
                          "(session %d, request %s)", session, request_id)
        # Only the request that performed this OCR may finalise it; an old
        # worker must not publish the final result of a newer request.
        self._maybe_finalize(session, request_id)

    # -------------------------------------------------------------- results
    def _on_finish_timer(self, _event):
        try:
            self._maybe_finalize()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: finalise failed: %s", err)

    def _maybe_finalize(self, session=None, request_id=None):
        finalize = False
        payload = None
        with self.state_lock:
            if not self.active or self.finalized:
                return
            if session is not None and (
                    session != self.session or request_id != self.request_id):
                return
            now = rospy.Time.now()
            if self.request_start is not None:
                timed_out = ((now - self.request_start).to_sec() >=
                             self.request_timeout)
            else:
                timed_out = False
            if self.first_frame_time is not None:
                empty_elapsed = ((now - self.first_frame_time).to_sec() >=
                                 self.empty_observe_time)
            else:
                empty_elapsed = False

            if self.frames == 0:
                # Without a first frame there is no evidence for "no car";
                # let the mission's request timeout handle a dead camera.
                return
            if self.seen:
                if self.ocr_pending and not timed_out:
                    return
                status = self.ocr_status
                if self.ocr_pending:
                    status = 'timeout'
                if not self.plate and status == 'pending':
                    status = 'failed'
                total = len(self.seen)
            else:
                if not empty_elapsed:
                    return
                status = 'no_car'
                total = 0

            self.finalized = True
            finalize = True
            payload = self._payload(
                total=total, boxes=self.latest_boxes,
                complete=True, session=self.session,
                request_id=self.request_id, ocr_status=status)

        if finalize:
            self._publish(payload)
            self._disable_image()
            rospy.loginfo("car request finished (session %d, id %s, "
                          "total %d, ocr_status %s)",
                          payload['session'], payload['request_id'] or 'legacy',
                          payload['total'], payload['ocr_status'])

    def _payload(self, total, boxes, complete, session, request_id,
                 ocr_status):
        return {
            'schema': SCHEMA_VERSION,
            'target': 'car',
            'request_id': request_id,
            'session': session,
            'complete': bool(complete),
            'total': int(total),
            'cars': sorted(self.seen),
            'plate': self.plate,
            'plate_conf': round(float(self.plate_conf), 3),
            'ocr_status': ocr_status,
            'boxes': boxes,
        }

    def _publish(self, payload):
        out = String()
        out.data = json.dumps(payload)
        self.result_pub.publish(out)

    def publish_result(self, dets, complete, session, request_id):
        with self.state_lock:
            if (session != self.session or request_id != self.request_id or
                    not self.active or self.finalized):
                return
            boxes = {}
            for name, _, box in dets:
                boxes.setdefault(name, []).append(
                    [round(float(v), 1) for v in box])
            payload = self._payload(
                total=len(self.seen), boxes=boxes, complete=complete,
                session=session, request_id=request_id,
                ocr_status=self.ocr_status)
        self._publish(payload)

    # --------------------------------------------------------------- debug
    def publish_debug(self, frame, dets, session, request_id):
        with self.state_lock:
            if (session != self.session or request_id != self.request_id or
                    not self.active):
                return
            plate = self.plate
            plate_conf = self.plate_conf

        for name, score, box in dets:
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), COLOR_CAR, 2)
            label = '%s %.2f' % (name, score)
            cv2.putText(frame, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CAR, 2)
            # mark the plate region that is sent to OCR
            px0 = x + int(self.plate_roi[0] * w)
            py0 = y + int(self.plate_roi[1] * h)
            px1 = x + int(self.plate_roi[2] * w)
            py1 = y + int(self.plate_roi[3] * h)
            cv2.rectangle(frame, (px0, py0), (px1, py1), COLOR_PLATE, 1)
            if plate:
                cv2.putText(frame, '%.2f' % plate_conf, (x, y + h + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_PLATE, 1)
        try:
            self.image_pub.publish(to_imgmsg(frame))
            self.debug_warned = False
        except Exception as err:                       # noqa: BLE001
            # the debug stream must never break the detector, but a persistent
            # failure (e.g. a broken cv_bridge) has to be visible, not silent
            if not self.debug_warned:
                self.debug_warned = True
                rospy.logwarn("car_detector: debug publish skipped: %s", err)


def main():
    rospy.init_node('car_detector')
    CarDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
