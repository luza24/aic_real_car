#!/usr/bin/env python3

"""
Car and license plate detector.

Runs an int8 ONNX model on the camera stream, looks for the single class
``car`` and reads the license plate of the car parked at the inspected
waypoint. Only the plate differs between the cars in this scene, so the
detector crops the plate region out of the detected car and recognises it
with easyocr (Chinese simplified + English, so the province character comes
out as real UTF-8 text).

    sub : /camera/image_raw              (sensor_msgs/Image)
    sub : /perception/detect             (std_msgs/String, "car" starts a read)
    pub : /perception/result             (std_msgs/String, json)
    pub : /perception/car/image          (sensor_msgs/Image, debug)

The ONNX model is not shipped with the repository. Train a detector with a
single ``car`` class, export it to int8 ONNX and place it at:

    aic26_perception/models/car_int8.onnx

easyocr is loaded lazily on the first read so the node still starts (and the
detector still works) when the plugin or its models are unavailable.
"""

import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import easyocr
except ImportError:      # reported on the topic instead of crashing the node
    easyocr = None

CLASSES = ['car']

# Plate is a fixed fraction of the rear view texture / detected car box.
PLATE_ROI = [0.30, 0.40, 0.70, 0.60]

# Province characters plus every legal plate letter/digit, I and O excluded.
PLATE_ALLOWLIST = ("苏沪京渝津冀晋蒙辽吉黑皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青"
                   "宁新ABCDEFGHJKLMNPQRSTUVWXYZ0123456789")

COLOR_CAR = (255, 200, 0)
COLOR_PLATE = (0, 255, 255)


def letterbox(img, size):
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


def crop_roi(img, roi, pad=0.05):
    """Crop ``roi`` = [x0, y0, x1, y1] fractions of ``img`` (clamped)."""
    h, w = img.shape[:2]
    x0 = max(0, min(w, int(round(w * (roi[0] - pad)))))
    y0 = max(0, min(h, int(round(h * (roi[1] - pad)))))
    x1 = max(0, min(w, int(round(w * (roi[2] + pad)))))
    y1 = max(0, min(h, int(round(h * (roi[3] + pad)))))
    return img[y0:y1, x0:x1]


def preprocess(crop, scale=3.0, binary=False):
    """Grayscale + cubic upscale; grayscale gave the best Chinese reads."""
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    if binary:
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 31, 10)
    if scale and scale != 1.0:
        interp = cv2.INTER_CUBIC if scale >= 2 else cv2.INTER_LINEAR
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=interp)
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
        self.size = rospy.get_param('~input_size', 320)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)

        # OCR settings
        self.plate_roi = rospy.get_param('~plate_roi', PLATE_ROI)
        self.ocr_scale = float(rospy.get_param('~ocr_scale', 3.0))
        self.ocr_binary = bool(rospy.get_param('~ocr_binary', False))
        self.ocr_gpu = bool(rospy.get_param('~ocr_gpu', False))
        self.ocr_min_chars = int(rospy.get_param('~ocr_min_chars', 4))
        self.ocr_wait = float(rospy.get_param('~ocr_wait', 8.0))

        self.bridge = CvBridge()
        self.net = self.load_model()

        self.reader = None
        self.reader_lock = threading.Lock()
        self.ocr_thread = None

        self.seen = set()        # classes seen in this detection
        self.plate = ''          # last recognised plate
        self.plate_conf = 0.0
        self.plate_done = True   # nothing pending until a "car" detect arrives

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/car/image'),
            Image, queue_size=1)

        rospy.Subscriber(rospy.get_param('~image_in', '/camera/image_raw'),
                         Image, self.on_image)
        rospy.Subscriber(rospy.get_param('~detect_topic', '/perception/detect'),
                         String, self.on_detect)
        rospy.loginfo("car_detector ready (class 'car', easyocr=%s)",
                      'yes' if easyocr else 'MISSING')

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'car_int8.onnx')

    def load_model(self):
        if not os.path.isfile(self.model_path):
            rospy.logwarn("model not found: %s, no car will be read",
                          self.model_path)
            return None
        try:
            net = cv2.dnn.readNetFromONNX(self.model_path)
        except cv2.error as err:
            rospy.logwarn("failed to load model: %s", err)
            return None
        return net

    def infer(self, frame):
        # returns a list of (class_name, score, box)
        img, scale, dx, dy = letterbox(frame, self.size)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.size, self.size),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()[0]

        # accept both [4+nc, N] and [N, 4+nc] export layouts
        if out.shape[0] < out.shape[1]:
            preds = out.T
        else:
            preds = out

        boxes, scores, ids = [], [], []
        for det in preds:
            cls_scores = det[4:]
            obj = float(np.max(cls_scores))
            if obj < self.conf:
                continue
            cx, cy, bw, bh = [float(v) for v in det[:4]]
            x = (cx - bw / 2 - dx) / scale
            y = (cy - bh / 2 - dy) / scale
            boxes.append([x, y, bw / scale, bh / scale])
            scores.append(obj)
            ids.append(int(np.argmax(cls_scores)))

        if not boxes:
            return []

        keep = np.array(cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.nms))
        if keep.size == 0:
            return []
        dets = []
        for i in keep.flatten():
            if ids[i] < len(CLASSES):
                dets.append((CLASSES[ids[i]], scores[i], boxes[i]))
        return dets

    # ------------------------------------------------------------------ ocr
    def load_reader(self):
        with self.reader_lock:
            if self.reader is None:
                if easyocr is None:
                    raise RuntimeError("easyocr is not installed")
                if not self.ocr_gpu:
                    os.environ.setdefault('CUDA_VISIBLE_DEVICES', '')
                rospy.loginfo("car_detector: loading easyocr (CPU) ...")
                self.reader = easyocr.Reader(['ch_sim', 'en'],
                                             gpu=self.ocr_gpu, verbose=False)
                rospy.loginfo("car_detector: easyocr ready")
            return self.reader

    def recognize(self, crop):
        """Return (text, conf) for the plate crop, or ('', 0.0)."""
        try:
            reader = self.load_reader()
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: OCR unavailable: %s", err)
            return '', 0.0
        best = ('', 0.0)
        results = []
        for variant in ([self.ocr_binary] if self.ocr_binary else [False]):
            img = preprocess(crop, scale=self.ocr_scale, binary=variant)
            try:
                found = reader.readtext(img, detail=1,
                                        allowlist=PLATE_ALLOWLIST,
                                        paragraph=False)
            except Exception as err:                   # noqa: BLE001
                rospy.logwarn("car_detector: readtext failed: %s", err)
                continue
            for _, text, conf in found:
                clean = tidy(text)
                if len(clean) >= self.ocr_min_chars:
                    results.append((clean, float(conf)))
        if results:
            # prefer the longest read, break ties on confidence
            results.sort(key=lambda r: (len(r[0]), r[1]), reverse=True)
            best = results[0]
        return best

    def read_plate_async(self, frame, box):
        """Crop the plate of ``box`` and OCR it on a worker thread."""
        if not self.plate_done or self.ocr_thread is not None:
            return
        x, y, w, h = [int(v) for v in box]
        x, y = max(0, x), max(0, y)
        w = min(w, frame.shape[1] - x)
        h = min(h, frame.shape[0] - y)
        if w < 8 or h < 8:
            return
        car = frame[y:y + h, x:x + w]
        crop = crop_roi(car, self.plate_roi)
        if crop.size == 0:
            return
        self.plate_done = False
        self.ocr_thread = threading.Thread(
            target=self._run_ocr, args=(crop.copy(),),
            name='car_plate_ocr')
        self.ocr_thread.daemon = True
        self.ocr_thread.start()

    def _run_ocr(self, crop):
        t0 = time.time()
        try:
            text, conf = self.recognize(crop)
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("car_detector: OCR failed: %s", err)
            text, conf = '', 0.0
        self.plate, self.plate_conf = text, conf
        self.plate_done = True
        self.ocr_thread = None
        rospy.loginfo("car_detector: plate '%s' (%.2f, %.2fs)",
                      text, conf, time.time() - t0)

    # ----------------------------------------------------------------- io
    def on_detect(self, msg):
        if msg.data.strip().lower() == 'car':
            self.seen = set()
            self.plate = ''
            self.plate_conf = 0.0
            self.plate_done = True
            rospy.loginfo("car read restarted")

    def on_image(self, msg):
        if self.net is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        dets = self.infer(frame)
        for name, score, box in dets:
            self.seen.add(name)
            # kick off OCR for the strongest car once per detection
            if box is dets[0][2]:
                self.read_plate_async(frame, box)
        self.publish_result(dets)
        self.publish_debug(frame, dets)

    def publish_result(self, dets):
        boxes = {name: [round(float(v), 1) for v in box]
                 for name, _, box in dets}

        out = String()
        out.data = json.dumps({
            'target': 'car',
            'total': len(dets),
            'cars': sorted(self.seen),
            'plate': self.plate,
            'plate_conf': round(float(self.plate_conf), 3),
            'boxes': boxes,
        })
        self.result_pub.publish(out)

    def publish_debug(self, frame, dets):
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
            if self.plate:
                cv2.putText(frame, '%.2f' % self.plate_conf, (x, y + h + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_PLATE, 1)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception as err:                       # noqa: BLE001
            # the debug stream must never break the detector
            rospy.logdebug("car_detector: debug publish skipped: %s", err)


def main():
    rospy.init_node('car_detector')
    CarDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
