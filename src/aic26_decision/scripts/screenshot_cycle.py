#!/usr/bin/env python3

"""
Dataset capture cycle.

Drives the robot along the real inspection route (config/screenshot_cycle.yaml
mirrors config/tasks.yaml) and saves labelled screenshots at every stop. The
vision detectors are replaced by a directly published result, so the whole
cycle runs end to end without any model and the images are stored under

    .datasets/screenshots/<label>/<label>_%04i.jpg

    pub : /perception/result   (std_msgs/String, json, faked vision output)
    pub : /perception/detect   (std_msgs/String, detection trigger)
    sub : /traffic_light/state (aic26_msgs/TrafficLightState, sim truth)
    sub : /camera/image_raw    (sensor_msgs/Image)
"""

import json
import os

import actionlib
import rospy
import yaml
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

try:
    from aic26_msgs.msg import TrafficLightState
    HAVE_LIGHT_MSG = True
except ImportError:
    TrafficLightState = None
    HAVE_LIGHT_MSG = False

GREEN = 'GREEN'

TURN_LEFT = 1.570796
TURN_RIGHT = -1.570796


def make_goal(x, y, yaw):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal


class ScreenshotCycle(object):
    def __init__(self):
        self.waypoints = self.load_waypoints(rospy.get_param('~waypoints_file'))
        self.stops = self.load_stops(rospy.get_param('~route_file'))
        self.out_dir = rospy.get_param('~output_dir')

        self.shots = 0
        self.lights = {}
        self.frame = None

        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.detect_pub = rospy.Publisher(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, queue_size=10)

        rospy.Subscriber(rospy.get_param('~image_topic', '/camera/image_raw'),
                         Image, self.on_image)
        if HAVE_LIGHT_MSG:
            rospy.Subscriber(rospy.get_param('~light_topic',
                                             '/traffic_light/state'),
                             TrafficLightState, self.on_light)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def load_waypoints(self, path):
        with open(path) as f:
            return {w["name"]: w for w in yaml.safe_load(f)["waypoints"]}

    def load_stops(self, path):
        with open(path) as f:
            return yaml.safe_load(f)["waypoints"]

    def on_image(self, msg):
        self.frame = msg

    def on_light(self, msg):
        self.lights[msg.model_name] = msg.state

    def go_to(self, wp):
        self.client.send_goal(make_goal(wp["x"], wp["y"], wp["yaw"]))
        if not self.client.wait_for_result():
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def turn_in_place(self, wp, direction):
        # same 90 degree in place rotation the mission controller uses
        yaw = wp["yaw"] + (TURN_LEFT if direction == "left" else TURN_RIGHT)
        self.client.send_goal(make_goal(wp["x"], wp["y"], yaw))
        if not self.client.wait_for_result():
            rospy.logwarn("turn %s failed", direction)
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def wait_green(self, model, timeout):
        # mirror mission_controller: leave on green, never block the cycle
        if not HAVE_LIGHT_MSG:
            return
        start = rospy.Time.now()
        deadline = start + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state = self.lights.get(model)
            if state == GREEN:
                rospy.loginfo("%s is GREEN", model)
                return
            rospy.sleep(0.1)
        rospy.logwarn("%s not green within %.0fs, passing", model, timeout)

    def save_shot(self, label):
        if self.frame is None:
            rospy.logwarn("no camera frame for %s", label)
            return
        folder = os.path.join(self.out_dir, label)
        if not os.path.isdir(folder):
            os.makedirs(folder)

        self.shots += 1
        name = "%s_%04i.jpg" % (label, self.shots)
        path = os.path.join(folder, name)
        try:
            import cv2
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            cv2.imwrite(path, frame)
            rospy.loginfo("saved %s", path)
        except Exception as err:
            rospy.logwarn("failed to save %s: %s", path, err)

    def fake_vision(self, stop):
        # replace the detector output with the planned result
        result = stop.get("result")
        if not result:
            return
        try:
            target = json.loads(result).get("target", "")
        except ValueError:
            target = ""

        trigger = String()
        trigger.data = target
        if target:
            self.detect_pub.publish(trigger)

        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        rospy.loginfo("fake result: %s", result)

    def visit(self, stop):
        name = stop["name"]
        if name not in self.waypoints:
            rospy.logwarn("unknown waypoint %s, skipping", name)
            return
        wp = self.waypoints[name]

        rospy.loginfo("stop %s at %s", stop.get("label", name), name)
        if not self.go_to(wp):
            rospy.logwarn("failed to reach %s", name)
            return

        turn = stop.get("turn")
        if turn in ("left", "right"):
            self.turn_in_place(wp, turn)
        if stop.get("light"):
            self.wait_green(stop["light"], stop.get("wait", 30.0))

        rospy.sleep(stop.get("dwell", 0.0))
        self.fake_vision(stop)

        for _ in range(stop.get("shots", 1)):
            self.save_shot(stop.get("label", name))
            rospy.sleep(rospy.get_param('~shot_interval', 0.5))

    def run(self):
        self.client.wait_for_server()
        rospy.loginfo("capture cycle started, %d stops", len(self.stops))
        for stop in self.stops:
            if rospy.is_shutdown():
                break
            self.visit(stop)
        rospy.loginfo("capture cycle finished, %d shots", self.shots)


def main():
    rospy.init_node("screenshot_cycle")
    ScreenshotCycle().run()


if __name__ == "__main__":
    main()


def main():
    rospy.init_node("screenshot_cycle")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        rospy.logerr("move_base action server load failed")
        return

    waypoints = load_waypoints(os.path.join(cur_pkg_path(), "config", "screenshot_cycle.yaml"))

    while not rospy.is_shutdown():
        for wp in waypoints:
            if rospy.is_shutdown():
                break
            rospy.loginfo("Going to %s", wp["name"])
            client.send_goal(make_goal(wp))
            if client.wait_for_result() and client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached %s", wp["name"])
            else:
                rospy.logwarn("Failed to reach %s", wp["name"])

if __name__ == "__main__":
    main()