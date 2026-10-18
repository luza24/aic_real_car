"""ROS interface for the Orbbec DaBai DCW2 RGB stream.

The RGB part of the camera is a standard UVC device, so this node opens it
with OpenCV and publishes /camera/image_raw and /camera/camera_info. The
depth stream is a vendor-specific USB interface and is intentionally not
handled here; the AIC26 perception stack only uses the colour image.
"""

import glob
import math
import os

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(object):
    """UVC camera to sensor_msgs/Image bridge."""

    def __init__(self):
        device = rospy.get_param('~device', '/dev/video0')
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        self.device = device
        self.autodetect = bool(rospy.get_param('~autodetect', True))

        self.width = int(rospy.get_param('~width', 1280))
        self.height = int(rospy.get_param('~height', 720))
        self.fps = max(1, int(rospy.get_param('~fps', 30)))
        self.fourcc = str(rospy.get_param('~fourcc', 'MJPG')).strip()

        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.image_topic = rospy.get_param(
            '~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param(
            '~camera_info_topic', '/camera/camera_info')

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
        self.bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher(
            self.image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(
            self.camera_info_topic, CameraInfo, queue_size=1)

        self._open_camera()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            'camera: %s %dx%d @ %d fps -> %s',
            self.device, self.width, self.height,
            self.fps, self.image_topic)

    def _candidate_devices(self):
        """Return the configured device plus any other /dev/video* nodes."""
        candidates = []
        if self.device not in ('', 'auto'):
            candidates.append(self.device)
        if self.autodetect:
            for path in sorted(glob.glob('/dev/video*')):
                if path not in candidates:
                    candidates.append(path)
        return candidates

    def _open_camera(self):
        backend = getattr(cv2, 'CAP_V4L2', 0)
        candidates = self._candidate_devices()
        if not candidates:
            rospy.logerr_throttle(
                10.0, 'camera: no /dev/video* devices; check USB/power '
                      'and run v4l2-ctl --list-devices')
            return False

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

                # Set the pixel format first: some V4L2 drivers reset the
                # frame size and rate when the format changes.
                if fourcc and len(fourcc) == 4:
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                ok, _frame = cap.read()
                if not ok:
                    cap.release()
                    continue

                self.device = device
                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                if actual_width > 0:
                    self.width = actual_width
                if actual_height > 0:
                    self.height = actual_height
                if actual_fps > 0:
                    self.fps = max(1, int(round(actual_fps)))

                self.cap = cap
                rospy.loginfo(
                    'camera: opened %s (%dx%d @ %.1f fps, %s)',
                    self.device, self.width, self.height, actual_fps,
                    fourcc or 'driver default')
                return True

        rospy.logwarn_throttle(
            10.0, 'camera: cannot open any of %s', candidates)
        return False

    def _make_camera_info(self):
        if self.hfov <= 0.0:
            self.hfov = math.pi / 2.0
        if self.fx <= 0.0:
            self.fx = (self.width * 0.5) / math.tan(self.hfov * 0.5)
        if self.fy <= 0.0:
            self.fy = self.fx
        if self.cx <= 0.0:
            self.cx = self.width * 0.5
        if self.cy <= 0.0:
            self.cy = self.height * 0.5

        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.P = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def spin(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            cap = self.cap
            if cap is None or not cap.isOpened():
                if not self._open_camera():
                    rospy.sleep(1.0)
                    continue
                cap = self.cap

            ok, frame = cap.read()
            if not ok:
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(
                        5.0, 'camera: read failed, reopening %s',
                        self.device)
                if self.cap is not None:
                    self.cap.release()
                self.cap = None
                continue

            if self.flip_horizontal and self.flip_vertical:
                frame = cv2.flip(frame, -1)
            elif self.flip_horizontal:
                frame = cv2.flip(frame, 1)
            elif self.flip_vertical:
                frame = cv2.flip(frame, 0)

            image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image.header.stamp = rospy.Time.now()
            image.header.frame_id = self.frame_id
            self.image_pub.publish(image)

            info = self._make_camera_info()
            info.header = image.header
            self.info_pub.publish(info)
            rate.sleep()

    def _shutdown(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
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
