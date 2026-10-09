#!/usr/bin/env python3

"""
Community people detector.

Runs an int8 ONNX model on the camera stream and counts the people found in
the current view. Every person model has its own class, so each class is
counted at most once (no double counting within one detection).

    sub : /camera/image_raw              (sensor_msgs/Image)
    sub : /perception/detect             (std_msgs/String, "people" starts a count)
    pub : /perception/result             (std_msgs/String, json)
    pub : /perception/people/image       (sensor_msgs/Image, debug)

The model is not shipped with the repository. Train a detector with the
classes below, export it to int8 ONNX and place it at:

    aic26_perception/models/people_int8.onnx

Model classes (fixed order):
    p1 .. p16, pF1, pF2

pF1 and pF2 are the non-community people, all the others belong to the
community.
"""

import json
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

CLASSES = ['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9',
           'p10', 'p11', 'p12', 'p13', 'p14', 'p15', 'p16', 'pF1', 'pF2']

INTRUDERS = {'pF1', 'pF2'}

COLOR_COMMUNITY = (0, 255, 0)
COLOR_INTRUDER = (0, 0, 255)


def letterbox(img, size):
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


class PeopleDetector(object):
    def __init__(self):
        self.model_path = rospy.get_param(
            '~model_path', self.default_model_path())
        self.size = rospy.get_param('~input_size', 320)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)

        self.bridge = CvBridge()
        self.net = self.load_model()

        self.seen = set()     # classes already counted in this detection

        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/people/image'),
            Image, queue_size=1)

        rospy.Subscriber(rospy.get_param('~image_in', '/camera/image_raw'),
                         Image, self.on_image)
        rospy.Subscriber(rospy.get_param('~detect_topic', '/perception/detect'),
                         String, self.on_detect)
        rospy.loginfo("people_detector ready (%d classes)", len(CLASSES))

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'people_int8.onnx')

    def load_model(self):
        if not os.path.isfile(self.model_path):
            rospy.logwarn("model not found: %s, no people will be detected",
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

    def on_detect(self, msg):
        if msg.data.strip().lower() == 'people':
            self.seen = set()
            rospy.loginfo("people count restarted")

    def on_image(self, msg):
        if self.net is None:
            return

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
        boxes = {name: [round(float(v), 1) for v in box] for name, _, box in dets}

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
        for name, score, box in dets:
            color = COLOR_INTRUDER if name in INTRUDERS else COLOR_COMMUNITY
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, '%s %.2f' % (name, score), (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            # the debug stream must never break the detector
            pass


def main():
    rospy.init_node('people_detector')
    PeopleDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

