"""Shared helpers for the aic26_perception detectors."""

import os

import cv2
import numpy as np

try:
    import openvino as ov
except ImportError:
    ov = None

try:
    import onnxruntime as ort
except ImportError:
    ort = None


def to_imgmsg(frame, frame_id=''):
    """Convert a BGR/gray numpy frame to ``sensor_msgs/Image``."""
    import rospy
    from sensor_msgs.msg import Image

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
    else:
        encoding = 'bgra8'

    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(width * channels * img.dtype.itemsize)
    msg.data = img.tobytes()
    return msg


def letterbox(img, size):
    """Resize ``img`` into a ``size`` x ``size`` canvas and keep the offset."""
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


def crop_roi(img, roi, pad=0.05):
    """Crop ``roi`` = [x0, y0, x1, y1] fractions of ``img``."""
    h, w = img.shape[:2]
    x0 = max(0, min(w, int(round(w * (roi[0] - pad)))))
    y0 = max(0, min(h, int(round(h * (roi[1] - pad)))))
    x1 = max(0, min(w, int(round(w * (roi[2] + pad)))))
    y1 = max(0, min(h, int(round(h * (roi[3] + pad)))))
    return img[y0:y1, x0:x1]


def _nms(boxes, scores, conf, nms):
    if not boxes:
        return []
    keep = cv2.dnn.NMSBoxes(boxes, scores, conf, nms)
    if keep is None:
        return []
    return [int(i) for i in np.array(keep).flatten()]


class YoloNet(object):
    """YOLOv5/v8 ONNX detector with OpenVINO preferred on the CPU."""

    def __init__(self, model_path, size=640, conf=0.5, nms=0.45,
                 num_classes=None, fallback_path=None, threads=0, log=None):
        self.model_path = model_path
        self.fallback_path = fallback_path
        self.size = max(32, int(size))
        self.conf = float(conf)
        self.nms = float(nms)
        self.num_classes = int(num_classes) if num_classes else None
        self.threads = max(0, int(threads))
        self.log = log or (lambda *a: None)

        self.ov_compiled = None
        self.ov_request = None
        self.ov_input = None
        self.session = None
        self.input_name = None
        self.net = None
        self._load()

    def _load(self):
        path = self.model_path
        if not os.path.isfile(path):
            fallback = self.fallback_path
            if fallback and fallback != path and os.path.isfile(fallback):
                self.log('warn', '%s not found, using %s', path, fallback)
                path = self.model_path = fallback
            else:
                self.log('error', 'model not found: %s', path)
                return
        if self._load_openvino(path):
            return
        if self._load_onnxruntime(path):
            return
        self._load_cv2(path)

    def _load_openvino(self, path):
        if ov is None:
            self.log('warn', 'openvino is not installed, trying onnxruntime')
            return False
        try:
            threads = self._thread_count()
            core = ov.Core()
            properties = {
                'INFERENCE_NUM_THREADS': threads,
                'NUM_STREAMS': 1,
                'PERFORMANCE_HINT': 'LATENCY',
            }
            try:
                core.set_property('CPU', properties)
            except Exception:                             # noqa: BLE001
                core.set_property('CPU', 'INFERENCE_NUM_THREADS', threads)
            # core.read_model() drops the class scores of these int8 ONNX
            # graphs; convert_model() runs the full frontend and is correct.
            if path.lower().endswith('.xml'):
                model = core.read_model(path)
            else:
                model = ov.convert_model(path)
            self.ov_compiled = core.compile_model(model, 'CPU')
            self.ov_input = self.ov_compiled.input(0)
            self._resolve_size(self.ov_input.shape)
            self.ov_request = self.ov_compiled.create_infer_request()
            self.log('info', 'loaded %s with openvino (CPU, %d threads)',
                     os.path.basename(path), threads)
            return True
        except Exception as err:                           # noqa: BLE001
            self.log('warn', 'openvino failed to load %s: %s', path, err)
            self.ov_compiled = None
            self.ov_request = None
            self.ov_input = None
            return False

    def _load_onnxruntime(self, path):
        if ort is None:
            self.log('warn', 'onnxruntime is not installed, trying cv2.dnn')
            return False
        try:
            available = ort.get_available_providers()
            providers = [p for p in ('CUDAExecutionProvider',
                                     'CPUExecutionProvider')
                         if p in available]
            self.session = ort.InferenceSession(
                path, providers=providers or None)
            self.input_name = self.session.get_inputs()[0].name
            self._resolve_size(self.session.get_inputs()[0].shape)
            self.log('info', 'loaded %s with onnxruntime (%s)',
                     os.path.basename(path),
                     ','.join(self.session.get_providers()))
            return True
        except Exception as err:                           # noqa: BLE001
            self.log('warn', 'onnxruntime failed to load %s: %s', path, err)
            self.session = None
            self.input_name = None
            return False

    def _load_cv2(self, path):
        try:
            self.net = cv2.dnn.readNetFromONNX(path)
        except cv2.error as err:
            self.log('error', 'cv2.dnn failed to load %s: %s', path, err)
            self.net = None
            return
        self.log('info', 'loaded %s with cv2.dnn', os.path.basename(path))

    def _thread_count(self):
        if self.threads > 0:
            wanted = self.threads
        else:
            raw = os.environ.get('AIC26_OV_THREADS')
            wanted = 2
            if raw:
                try:
                    wanted = int(raw)
                except ValueError:
                    pass
        return max(1, min(wanted, os.cpu_count() or 1))

    def _resolve_size(self, shape):
        try:
            h, w = int(shape[-2]), int(shape[-1])
        except (IndexError, TypeError, ValueError):
            return
        if h > 0 and w > 0 and (h != self.size or w != self.size):
            self.log('info', 'using model input size %dx%d (was %d)',
                     w, h, self.size)
            self.size = max(h, w)

    @property
    def ready(self):
        return (self.ov_request is not None or self.session is not None or
                self.net is not None)

    def _forward(self, blob):
        if self.ov_request is not None:
            self.ov_request.infer({self.ov_input: blob})
            return np.array(self.ov_request.get_output_tensor(0).data,
                            copy=True)
        if self.session is not None:
            return self.session.run(None, {self.input_name: blob})[0]
        return self.net.forward()

    def infer(self, frame):
        """Return (class_id, score, [x, y, w, h]) detections for ``frame``."""
        if not self.ready:
            return []

        img, scale, dx, dy = letterbox(frame, self.size)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.size, self.size),
                                     swapRB=True, crop=False)
        return self._decode(self._forward(blob), scale, dx, dy, frame.shape)

    def _decode(self, out, scale, dx, dy, frame_shape):
        if out.ndim == 3:
            out = out[0]
        if out.ndim != 2 or out.shape[0] == 0 or out.shape[1] == 0:
            return []

        preds = out.T if out.shape[0] < out.shape[1] else out
        if preds.shape[1] < 5:
            return []

        expected = 4 + self.num_classes if self.num_classes else 0
        if expected and preds.shape[1] != expected:
            if preds.shape[1] < expected:
                self.log('warn',
                         'model output has %d values per anchor, expected %d',
                         preds.shape[1], expected)
                return []
            preds = preds[:, :expected]

        cls_scores = preds[:, 4:]
        scores = cls_scores.max(axis=1)
        ids = cls_scores.argmax(axis=1)
        mask = scores >= self.conf
        if not np.any(mask):
            return []

        det = preds[mask]
        scores = scores[mask]
        ids = ids[mask]
        cx, cy, bw, bh = det[:, 0], det[:, 1], det[:, 2], det[:, 3]

        boxes = np.empty((det.shape[0], 4), dtype=np.float64)
        boxes[:, 0] = (cx - bw / 2.0 - dx) / scale
        boxes[:, 1] = (cy - bh / 2.0 - dy) / scale
        boxes[:, 2] = bw / scale
        boxes[:, 3] = bh / scale

        boxes = boxes.tolist()
        scores = scores.tolist()
        ids = ids.tolist()
        return [(int(ids[i]), float(scores[i]),
                 self._clamp(boxes[i], frame_shape))
                for i in _nms(boxes, scores, self.conf, self.nms)]

    @staticmethod
    def _clamp(box, frame_shape):
        h, w = frame_shape[:2]
        x, y, bw, bh = box
        x0 = max(0, min(w, x))
        y0 = max(0, min(h, y))
        x1 = max(0, min(w, x + bw))
        y1 = max(0, min(h, y + bh))
        return [x0, y0, x1 - x0, y1 - y0]
