#!/usr/bin/env python3

"""
Traffic light state detector.

Runs an int8 ONNX model on the camera stream and publishes the light state.

    sub : /camera/image_raw                    (sensor_msgs/Image)
    pub : /perception/traffic_light            (aic26_msgs/TrafficLightState)
    pub : /perception/traffic_light/image      (sensor_msgs/Image, debug)

The model is not shipped with the repository. Train a detector with the
classes below, export it to int8 ONNX and place it at:

    aic26_perception/models/traffic_light_int8.onnx

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

import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from aic26_msgs.msg import TrafficLightState

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



def letterbox(img, size):
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


class TrafficLightDetector(object):
    def __init__(self):
        self.model_path = rospy.get_param(
            '~model_path', self.default_model_path())
        self.size = rospy.get_param('~input_size', 320)
        self.conf = rospy.get_param('~conf', 0.5)
        self.nms = rospy.get_param('~nms', 0.45)

        self.bridge = CvBridge()
        self.net = self.load_model()

        self.state_pub = rospy.Publisher(
            rospy.get_param('~state_topic', '/perception/traffic_light'),
            TrafficLightState, queue_size=10, latch=True)
        self.image_pub = rospy.Publisher(
            rospy.get_param('~image_topic', '/perception/traffic_light/image'),
            Image, queue_size=1)

        rospy.Subscriber(rospy.get_param('~image_in', '/camera/image_raw'),
                         Image, self.on_image)
        rospy.loginfo("traffic_light_detector ready (classes: %s)",
                      ','.join(CLASSES))

    def default_model_path(self):
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'models', 'traffic_light_int8.onnx')

    def load_model(self):
        if not os.path.isfile(self.model_path):
            rospy.logwarn("model not found: %s, state will be UNKNOWN",
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

    def on_image(self, msg):
        if self.net is None:
            self.publish_state('UNKNOWN')
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        dets = self.infer(frame)
        self.publish_state(self.light_state(dets))
        self.publish_debug(frame, dets)

    def publish_state(self, state):
        out = TrafficLightState()
        out.model_name = rospy.get_param('~node_name', 'traffic_light')
        out.state = state
        self.state_pub.publish(out)

    def publish_debug(self, frame, dets):
        for name, score, box in dets:
            color, _ = parse_class(name)
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), COLORS[color], 2)
            cv2.putText(frame, '%s %.2f' % (name, score), (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[color], 2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except Exception:
            # the debug stream must never break the detector
            pass


def main():
    rospy.init_node('traffic_light_detector')
    TrafficLightDetector()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
