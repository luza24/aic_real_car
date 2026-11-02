"""Demand-driven Orbbec DaBai DCW2 RGB bridge (UVC via OpenCV).

The capture device is only open while somebody listens:

    /camera/image_raw             full resolution bgr8, raw subscribers
    /camera/image_raw/compressed  small JPEG preview for camera_view
    /camera/active                std_msgs/Bool, device is streaming

A raw subscriber (perception, capture_cycle) gets ``width x height`` at
``fps``; ``camera_view`` alone gets ``view_width x view_height`` at
``view_fps`` and a JPEG at ``view_quality``. With no subscriber the device is
released, so an idle robot pays neither USB bandwidth nor CPU.
"""

import glob
import math
import os
import time

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool

IDLE = 'idle'
VIEW = 'view'
FULL = 'full'


class CameraNode(object):

    def __init__(self):
        self.device = rospy.get_param('~device', '/dev/video0')
        if isinstance(self.device, str) and self.device.isdigit():
            self.device = int(self.device)
        self.autodetect = bool(rospy.get_param('~autodetect', True))

        self.width = int(rospy.get_param('~width', 1280))
        self.height = int(rospy.get_param('~height', 720))
        self.fps = max(1, int(rospy.get_param('~fps', 30)))
        self.fourcc = str(rospy.get_param('~fourcc', 'MJPG')).strip()

        self.view_width = int(rospy.get_param('~view_width', 640))
        self.view_height = int(rospy.get_param('~view_height', 360))
        self.view_fps = max(1, int(rospy.get_param('~view_fps', 10)))
        self.view_quality = min(95, max(1, int(
            rospy.get_param('~view_quality', 30))))

        self.idle_timeout = max(0.0, float(
            rospy.get_param('~idle_timeout', 1.0)))
        self.always_on = bool(rospy.get_param('~always_on', False))

        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.image_topic = rospy.get_param(
            '~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param(
            '~camera_info_topic', '/camera/camera_info')
        self.compressed_topic = rospy.get_param(
            '~compressed_topic', self.image_topic + '/compressed')
        self.active_topic = rospy.get_param(
            '~active_topic', '/camera/active')

        self.hfov = float(rospy.get_param('~hfov', math.pi / 2.0))
        self.fx = float(rospy.get_param('~fx', 0.0))
        self.fy = float(rospy.get_param('~fy', 0.0))
        self.cx = float(rospy.get_param('~cx', 0.0))
        self.cy = float(rospy.get_param('~cy', 0.0))
        self.flip_horizontal = bool(
            rospy.get_param('~flip_horizontal', False))
        self.flip_vertical = bool(
            rospy.get_param('~flip_vertical', False))

        self.cap = None
        self.mode = IDLE
        self.last_consumer = 0.0
        self.bridge = cv_bridge.CvBridge()

        self.image_pub = rospy.Publisher(
            self.image_topic, Image, queue_size=1)
        self.compressed_pub = rospy.Publisher(
            self.compressed_topic, CompressedImage, queue_size=1)
        self.info_pub = rospy.Publisher(
            self.camera_info_topic, CameraInfo, queue_size=1)
        self.active_pub = rospy.Publisher(
            self.active_topic, Bool, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(1.0), self._publish_active)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            'camera: %s full %dx%d @ %d fps, view %dx%d @ %d fps q%d -> %s',
            self.device, self.width, self.height, self.fps,
            self.view_width, self.view_height, self.view_fps,
            self.view_quality, self.image_topic)

    def _candidate_devices(self):
        candidates = []
        if self.device not in ('', 'auto'):
            candidates.append(self.device)
        if self.autodetect:
            for path in sorted(glob.glob('/dev/video*')):
                if path not in candidates:
                    candidates.append(path)
        return candidates

    def _open(self):
        backend = getattr(cv2, 'CAP_V4L2', 0)
        candidates = self._candidate_devices()
        if not candidates:
            rospy.logerr_throttle(
                10.0, 'camera: no /dev/video* devices; check USB/power '
                      'and run v4l2-ctl --list-devices')
            return False

        if self.mode == FULL:
            width, height, fps = self.width, self.height, self.fps
        else:
            width, height = self.view_width, self.view_height
            fps = self.view_fps

        fourccs = []
        for fourcc in (self.fourcc, 'MJPG', 'YUYV', ''):
            if fourcc not in fourccs:
                fourccs.append(fourcc)

        for device in candidates:
            if (isinstance(device, str) and device.startswith('/dev/') and
                    not os.path.exists(device)):
                continue
            for fourcc in fourccs:
                cap = cv2.VideoCapture(device, backend)
                if not cap.isOpened():
                    cap.release()
                    continue

                if fourcc and len(fourcc) == 4:
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                ok, _frame = cap.read()
                if not ok:
                    cap.release()
                    continue

                self.device = device
                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                if self.mode == FULL:
                    self.width = actual_width or self.width
                    self.height = actual_height or self.height
                    if actual_fps > 0:
                        self.fps = max(1, int(round(actual_fps)))
                else:
                    self.view_width = actual_width or self.view_width
                    self.view_height = actual_height or self.view_height

                self.cap = cap
                rospy.loginfo(
                    'camera: opened %s (%dx%d @ %.1f fps, %s, %s mode)',
                    self.device, actual_width, actual_height, actual_fps,
                    fourcc or 'driver default', self.mode)
                return True

        rospy.logwarn_throttle(
            10.0, 'camera: cannot open any of %s at %dx%d', candidates,
            width, height)
        return False

    def _close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _desired_mode(self):
        if self.always_on:
            return FULL
        if (self.image_pub.get_num_connections() or
                self.info_pub.get_num_connections()):
            return FULL
        if self.compressed_pub.get_num_connections():
            return VIEW
        return IDLE

    def _enter(self, mode):
        if mode != self.mode:
            if self.cap is not None:
                self._close()
            rospy.loginfo('camera: %s -> %s', self.mode, mode)
            self.mode = mode

    def _publish_active(self, _event):
        msg = Bool()
        msg.data = self.cap is not None and self.cap.isOpened()
        self.active_pub.publish(msg)

    def _make_camera_info(self, header, width, height):
        if self.hfov <= 0.0:
            self.hfov = math.pi / 2.0
        fx = self.fx or (width * 0.5) / math.tan(self.hfov * 0.5)
        fy = self.fy or fx
        cx = self.cx or width * 0.5
        cy = self.cy or height * 0.5

        info = CameraInfo()
        info.header = header
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish(self, frame):
        stamp = rospy.Time.now()
        if self.mode == FULL:
            image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image.header.stamp = stamp
            image.header.frame_id = self.frame_id
            self.image_pub.publish(image)
            self.info_pub.publish(self._make_camera_info(
                image.header, image.width, image.height))
            return

        ok, buffer = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.view_quality])
        if not ok:
            rospy.logwarn_throttle(5.0, 'camera: jpeg encode failed')
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        self.compressed_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            wanted = self._desired_mode()
            if wanted != IDLE:
                self.last_consumer = time.time()
            elif (self.cap is not None and
                    time.time() - self.last_consumer < self.idle_timeout):
                wanted = self.mode
            if wanted != self.mode:
                self._enter(wanted)
                rate = rospy.Rate(
                    self.fps if self.mode == FULL else self.view_fps)

            if self.mode == IDLE:
                rospy.sleep(0.1)
                continue

            if self.cap is None and not self._open():
                rospy.sleep(1.0)
                continue

            ok, frame = self.cap.read()
            if not ok:
                rospy.logwarn_throttle(
                    5.0, 'camera: read failed, reopening %s', self.device)
                self._close()
                continue

            if self.flip_horizontal and self.flip_vertical:
                frame = cv2.flip(frame, -1)
            elif self.flip_horizontal:
                frame = cv2.flip(frame, 1)
            elif self.flip_vertical:
                frame = cv2.flip(frame, 0)

            self._publish(frame)
            rate.sleep()

    def _shutdown(self):
        self._close()
        rospy.loginfo('camera: stopped')


def main():
    rospy.init_node('camera')
    try:
        node = CameraNode()
    except Exception as exc:
        rospy.logfatal('camera: %s', exc)
        return
    node.spin()


if __name__ == '__main__':
    main()
