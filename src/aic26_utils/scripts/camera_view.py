"""Live view of a camera topic with frame capture.

    sub : ~image (sensor_msgs/CompressedImage, or Image with ~transport raw)

The preview subscribes to the compressed stream by default, so the camera
node stays at ``view_width x view_height`` instead of streaming full
resolution to the display. ``~transport raw`` keeps the old behaviour.

Params:
    ~image        image topic (default /camera/image_raw)
    ~transport    'compressed' (default) or 'raw'
    ~width        window width in pixels (default 640)
    ~height       window height in pixels (default 360)
    ~scale        multiplies width/height (default 1.0, AIC26_GUI_SCALE wins)
    ~window_name  window title (default 'camera')
    ~x, ~y        window position in pixels (default 40, 40)
    ~rate         refresh rate in Hz (default 15.0)
    ~save         write frames to disk (default true)
    ~save_fps     automatic captures per second, 0 means key only (default 0)
    ~save_dir     output folder, default <pkg>/../../.datasets/capture
    ~filename     name pattern, default frame_{index:04d}.jpg
    ~label        text drawn in the window (default '')

's' saves the current frame, 'q'/ESC or closing the window quits.
{index} counts without overwriting, {stamp} is YYYYmmdd_HHMMSS and {label}
is the ~label text.
"""

import os
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

GUI_SCALE_ENV = 'AIC26_GUI_SCALE'
DEFAULT_FILENAME = 'frame_{index:04d}.jpg'
FONT = cv2.FONT_HERSHEY_SIMPLEX


def get_bool(name, default):
    """Boolean param; 'false'/'0'/'no'/'off' from rosrun count as False."""
    raw = rospy.get_param(name, default)
    if isinstance(raw, str):
        return raw.strip().lower() not in ('', '0', 'false', 'no', 'off')
    return bool(raw)


def default_save_dir():
    """<pkg>/../../.datasets/capture, i.e. <workspace>/.datasets/capture."""
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    try:
        import rospkg
        pkg = rospkg.RosPack().get_path('aic26_utils')
    except Exception:                                  # noqa: BLE001
        pass
    return os.path.normpath(os.path.join(pkg, os.pardir, os.pardir,
                                         '.datasets', 'capture'))


def draw_text(frame, lines):
    """Draw (text, colour) pairs into the top left corner of ``frame``."""
    size = max(0.4, 0.6 * frame.shape[0] / 360.0)
    step = int(round(22 * size))
    for i, (text, color) in enumerate(lines):
        pos = (8, step * (i + 1))
        cv2.putText(frame, text, (pos[0] + 1, pos[1] + 1), FONT, size,
                    (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, text, pos, FONT, size, color, 1, cv2.LINE_AA)


def gui_scale(default=1.0):
    """Window size multiplier: AIC26_GUI_SCALE, else ~scale / ~gui_scale."""
    raw = os.environ.get(GUI_SCALE_ENV)
    if raw is None:
        raw = rospy.get_param('~scale', rospy.get_param('~gui_scale', default))
    try:
        scale = float(raw)
    except (TypeError, ValueError):
        rospy.logwarn('%s=%r is not a number, using %.2f',
                      GUI_SCALE_ENV, raw, default)
        return default
    return min(max(scale, 0.1), 8.0)


class SnapshotWriter(object):
    """Writes frames on demand and, optionally, at a steady rate."""

    def __init__(self, directory, pattern=DEFAULT_FILENAME, fps=1.0,
                 label=''):
        self.dir = os.path.abspath(os.path.expanduser(directory))
        self.pattern = pattern or DEFAULT_FILENAME
        self.label = label
        self.counting = '{index' in self.pattern
        self.period = 0.0 if fps <= 0.0 else 1.0 / fps
        self.next_time = 0.0
        self.index = 0
        self.count = 0
        os.makedirs(self.dir, exist_ok=True)
        rospy.loginfo("camera view: saving to %s/%s (%s)", self.dir,
                      self.pattern,
                      'manual only' if self.period <= 0.0
                      else '%.1f Hz' % (1.0 / self.period))

    def due(self, now):
        """True when an automatic capture is due at ``now`` (seconds)."""
        if self.period <= 0.0 or now < self.next_time:
            return False
        self.next_time = now + self.period
        return True

    def save(self, frame):
        """Write ``frame`` and return its path, or None."""
        stamp = time.strftime('%Y%m%d_%H%M%S')
        while True:
            self.index += 1
            try:
                name = self.pattern.format(index=self.index, stamp=stamp,
                                           label=self.label)
            except (KeyError, IndexError, ValueError) as err:
                rospy.logwarn_throttle(10.0, "camera view: bad ~filename "
                                       "%r: %s", self.pattern, err)
                return None
            path = os.path.join(self.dir, name)
            if not self.counting or not os.path.exists(path):
                break
        try:
            folder = os.path.dirname(path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            written = cv2.imwrite(path, frame)
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn("camera view: could not save %s: %s", path, err)
            return None
        if not written:
            rospy.logwarn("camera view: could not write %s", path)
            return None
        self.count += 1
        rospy.loginfo("camera view: saved %s (%d)", path, self.count)
        return path


class CameraView(object):
    """Subscribes to the camera and displays the newest frame on demand."""

    def __init__(self, topic, transport='compressed'):
        self.transport = transport
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.msg = None
        if transport == 'compressed':
            if not topic.endswith('/compressed'):
                topic += '/compressed'
            rospy.Subscriber(topic, CompressedImage, self.on_image,
                             queue_size=1)
        else:
            rospy.Subscriber(topic, Image, self.on_image, queue_size=1)
        self.topic = topic

    def on_image(self, msg):
        with self.lock:
            self.msg = msg

    def latest(self):
        with self.lock:
            return self.msg

    def to_bgr(self, msg):
        if isinstance(msg, CompressedImage):
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        return self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def run(self):
        """Show the window in the main thread (GTK is not thread safe)."""
        scale = gui_scale()
        width = int(rospy.get_param('~width', 640) * scale)
        height = int(rospy.get_param('~height', 360) * scale)
        name = rospy.get_param('~window_name', 'camera')
        label = rospy.get_param('~label', '')

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, max(64, width), max(64, height))
        cv2.moveWindow(name, int(rospy.get_param('~x', 40) * scale),
                       int(rospy.get_param('~y', 40) * scale))
        rospy.loginfo("camera view: %s on %s (%dx%d)",
                      name, self.topic, width, height)

        writer = None
        if get_bool('~save', True):
            save_dir = rospy.get_param('~save_dir', '') or default_save_dir()
            try:
                writer = SnapshotWriter(
                    save_dir, rospy.get_param('~filename', DEFAULT_FILENAME),
                    float(rospy.get_param('~save_fps', 0.0)), label)
            except (OSError, ValueError) as err:
                rospy.logerr("camera view: saving disabled, %s: %s",
                             save_dir, err)

        hint = 's: save  q: quit' if writer is not None else 'q: quit'
        status, status_until = hint, 0.0
        rate = rospy.Rate(rospy.get_param('~rate', 15.0))
        visible_seen = False
        frame = None
        decoded = None
        while not rospy.is_shutdown():
            msg = self.latest()
            if msg is not None and msg is not decoded:
                decoded = msg
                try:
                    frame = self.to_bgr(msg)
                except Exception as err:                   # noqa: BLE001
                    rospy.logwarn_throttle(
                        10.0, "camera view: bad frame: %s", err)
            if frame is not None and msg is not None:
                now = time.time()
                if writer is not None and writer.due(now):
                    writer.save(frame)
                if (frame.shape[1] != width or
                        frame.shape[0] != height):
                    view = cv2.resize(
                        frame, (width, height),
                        interpolation=cv2.INTER_AREA)
                else:
                    view = frame.copy()
                if status_until and now > status_until:
                    status, status_until = hint, 0.0
                lines = [(status, (0, 255, 0) if status_until
                          else (200, 200, 200))]
                if label:
                    lines.insert(0, (label, (0, 255, 255)))
                draw_text(view, lines)
                cv2.imshow(name, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if key in (ord('s'), ord('S')):
                if writer is None:
                    rospy.logwarn("camera view: saving is disabled")
                elif frame is None:
                    rospy.logwarn("camera view: no frame to save yet")
                else:
                    path = writer.save(frame)
                    if path:
                        status = 'saved %s' % os.path.basename(path)
                        status_until = time.time() + 2.0
            if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1:
                visible_seen = True
            elif visible_seen:
                rospy.loginfo("camera view: window closed")
                break
            rate.sleep()

        cv2.destroyWindow(name)


def main():
    rospy.init_node('camera_view')
    view = CameraView(rospy.get_param('~image', '/camera/image_raw'),
                      rospy.get_param('~transport', 'compressed'))
    try:
        view.run()
    except cv2.error as err:
        rospy.logerr("camera view: no usable display / OpenCV window (%s)",
                     err)
    rospy.signal_shutdown('camera view finished')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
