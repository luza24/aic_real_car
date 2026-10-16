#!/usr/bin/env python3

"""
Shared helpers for the aic26_perception detectors.

The three detector nodes (traffic_light_detector, people_detector,
car_detector) all run the same kind of model: a YOLOv5/v8 style detector
exported to int8 ONNX with a fixed input size and an output tensor of shape
[1, 4 + num_classes, num_anchors] (or the transposed [1, num_anchors,
4 + num_classes] layout). This module keeps that common part in one place so
the nodes only deal with their own classes and messages.

Backend
-------
OpenCV's ``dnn`` module cannot execute the int8 quantisation found in these
models (it raises an assertion in ``pool2_int8_layer``), so onnxruntime is
used as the primary backend when it is installed. ``cv2.dnn`` is kept as a
fallback for float models / environments without onnxruntime.

Everything here is plain Python + OpenCV/numpy and has no ROS dependency, so
the module can also be imported and unit tested without a running master.
"""

import os

import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError:          # fall back to cv2.dnn only
    ort = None


def to_imgmsg(frame, frame_id=''):
    """Convert a BGR (or gray) numpy frame to a ``sensor_msgs/Image``.

    ``cv_bridge.cv2_to_imgmsg`` cannot be used here: this workspace runs
    OpenCV 5, whose ``CV_8UC3`` constant (64) no longer matches the value the
    ROS Noetic C++ bridge reports for ``bgr8`` (16), so the encoding check
    raises ``KeyError: 16``. Building the message by hand keeps the debug
    image stream working regardless of the OpenCV / cv_bridge versions.
    """
    from sensor_msgs.msg import Image           # local import: ROS-only module

    img = np.ascontiguousarray(frame)
    if img.ndim == 2:
        height, width = img.shape
        channels = 1
    else:
        height, width, channels = img.shape
    if channels == 3:
        encoding = 'bgr8'
    elif channels == 1:
        encoding = 'mono8'
    else:                                        # BGRA / RGBA
        encoding = 'bgra8'

    msg = Image()
    msg.header.frame_id = frame_id
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(width * channels * img.dtype.itemsize)
    msg.data = img.tobytes()
    return msg


def letterbox(img, size):
    """Resize ``img`` into a ``size`` x ``size`` canvas, keeping aspect ratio.

    Returns ``(canvas, scale, dx, dy)`` where ``scale`` is the resize factor
    and ``(dx, dy)`` the padding offset of the letterboxed image, so detection
    boxes can be mapped back to the original frame.
    """
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


def _nms(boxes, scores, conf, nms):
    """Thin wrapper around ``cv2.dnn.NMSBoxes`` that always returns a list.

    Depending on the OpenCV version the function returns either an ndarray or
    a tuple, and an empty one when nothing survives; flatten it to a plain
    list of indices so callers do not have to care.
    """
    if not boxes:
        return []
    keep = cv2.dnn.NMSBoxes(boxes, scores, conf, nms)
    if keep is None:
        return []
    keep = np.array(keep).flatten()
    return [int(i) for i in keep]


class YoloNet(object):
    """YOLOv5/v8 ONNX detector with an onnxruntime/cv2.dnn backend."""

    def __init__(self, model_path, size=640, conf=0.5, nms=0.45,
                 num_classes=None, log=None):
        self.model_path = model_path
        self.size = int(size)
        self.conf = float(conf)
        self.nms = float(nms)
        self.num_classes = num_classes
        self.log = log or (lambda *a: None)

        self.session = None
        self.net = None
        self.input_name = None
        self._load()

    # ----------------------------------------------------------------- load
    def _load(self):
        if not os.path.isfile(self.model_path):
            self.log('warn', 'model not found: %s', self.model_path)
            return
        if ort is not None:
            try:
                providers = [p for p in ('CPUExecutionProvider',)
                             if p in ort.get_available_providers()]
                self.session = ort.InferenceSession(self.model_path,
                                                    providers=providers or None)
                self.input_name = self.session.get_inputs()[0].name
                self._resolve_size(self.session.get_inputs()[0].shape)
                self.log('info', 'loaded %s with onnxruntime (%s)',
                         os.path.basename(self.model_path),
                         ','.join(self.session.get_providers()))
                return
            except Exception as err:                       # noqa: BLE001
                self.log('warn', 'onnxruntime failed (%s), trying cv2.dnn',
                         err)
                self.session = None
        self._load_cv2()

    def _load_cv2(self):
        try:
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
        except cv2.error as err:
            self.log('warn', 'failed to load model: %s', err)
            return
        self.log('info', 'loaded %s with cv2.dnn',
                 os.path.basename(self.model_path))

    def _resolve_size(self, shape):
        """Adopt the model's fixed input size when it is statically known."""
        try:
            h, w = int(shape[2]), int(shape[3])
        except (IndexError, TypeError, ValueError):
            return
        if 0 < h and 0 < w and (h != self.size or w != self.size):
            self.log('info', 'using model input size %dx%d (was %d)',
                     w, h, self.size)
            self.size = max(h, w)

    @property
    def ready(self):
        return self.session is not None or self.net is not None

    # ---------------------------------------------------------------- infer
    def _forward(self, blob):
        if self.session is not None:
            return self.session.run(None, {self.input_name: blob})[0]
        self.net.setInput(blob)
        return self.net.forward()

    def infer(self, frame):
        """Detect in ``frame``; returns a list of (cls_id, score, box).

        ``box`` is [x, y, w, h] in pixels of the original frame, clamped to it.
        """
        if not self.ready:
            return []

        img, scale, dx, dy = letterbox(frame, self.size)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.size, self.size),
                                     swapRB=True, crop=False)
        out = np.asarray(self._forward(blob))
        if out.ndim == 3:
            out = out[0]
        return self._decode(out, scale, dx, dy, frame.shape)

    def _decode(self, out, scale, dx, dy, frame_shape):
        # accept both [4+nc, N] and [N, 4+nc] export layouts
        if out.shape[0] < out.shape[1]:
            preds = out.T
        else:
            preds = out

        if preds.shape[1] < 5:
            return []

        boxes, scores, ids = [], [], []
        for det in preds:
            cls_scores = det[4:]
            if cls_scores.size == 0:
                continue
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

        return [(ids[i], scores[i], self._clamp(boxes[i], frame_shape))
                for i in _nms(boxes, scores, self.conf, self.nms)]

    def _clamp(self, box, frame_shape):
        h, w = frame_shape[:2]
        x, y, bw, bh = box
        x0 = max(0, min(w, x))
        y0 = max(0, min(h, y))
        x1 = max(0, min(w, x + bw))
        y1 = max(0, min(h, y + bh))
        return [x0, y0, x1 - x0, y1 - y0]
