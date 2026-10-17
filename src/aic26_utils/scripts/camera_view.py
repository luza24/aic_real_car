"""Live view of a camera topic in an OpenCV window.

Replaces the stock ``image_view`` node, which keeps OpenCV's 320x240 GTK
window over ssh -X because it resizes only *after* mapping the window. Here
the window is sized before the first ``imshow`` and every GUI call stays in
the main thread, which GTK requires.

    sub : ~image (sensor_msgs/Image)

Params:
    ~image        image topic (default /camera/image_raw)
    ~width        window width in pixels (default 640)
    ~height       window height in pixels (default 360)
    ~scale        multiplies width/height (default 1.0, AIC26_GUI_SCALE wins)
    ~window_name  window title (default 'camera')
    ~x, ~y        window position in pixels (default 40, 40)
    ~rate         refresh rate in Hz (default 30.0)

q, ESC or closing the window stops the node, so it can be relaunched together
with the rest of the launch file.
"""

import os
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

GUI_SCALE_ENV = 'AIC26_GUI_SCALE'


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


class CameraView(object):
    """Subscribes to the camera and displays the newest frame on demand."""

    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None         # newest bgr frame, or None before the first
        rospy.Subscriber(topic, Image, self.on_image, queue_size=1)

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:                       # noqa: BLE001
            rospy.logwarn_throttle(10.0, "camera view: bad frame: %s", err)
            return
        with self.lock:
            self.frame = frame

    def latest(self):
        with self.lock:
            return self.frame

    def run(self):
        """Show the window in the main thread (GTK is not thread safe)."""
        scale = gui_scale()
        width = int(rospy.get_param('~width', 640) * scale)
        height = int(rospy.get_param('~height', 360) * scale)
        name = rospy.get_param('~window_name', 'camera')

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        # before the first imshow: resizing a mapped window is dropped here
        cv2.resizeWindow(name, max(64, width), max(64, height))
        cv2.moveWindow(name, int(rospy.get_param('~x', 40) * scale),
                       int(rospy.get_param('~y', 40) * scale))
        rospy.loginfo("camera view: %s on %s (%dx%d)",
                      name, self.topic, width, height)

        rate = rospy.Rate(rospy.get_param('~rate', 30.0))
        visible_seen = False      # WND_PROP_VISIBLE is 0 before the first map
        while not rospy.is_shutdown():
            frame = self.latest()
            if frame is not None:
                cv2.imshow(name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1:
                visible_seen = True
            elif visible_seen:
                # the window was closed, either by the user or by the X server
                rospy.loginfo("camera view: window closed")
                break
            rate.sleep()

        cv2.destroyWindow(name)


def main():
    rospy.init_node('camera_view')
    view = CameraView(rospy.get_param('~image', '/camera/image_raw'))
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
