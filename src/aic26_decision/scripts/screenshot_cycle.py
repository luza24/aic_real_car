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
import tf
import tf.transformations
import yaml
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
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


def wrap_angle(angle):
    # bring an angle into (-pi, pi]
    while angle > 3.141592653589793:
        angle -= 6.283185307179586
    while angle <= -3.141592653589793:
        angle += 6.283185307179586
    return angle


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

        # per label shot bookkeeping so the cycle can loop until every stop has
        # reached its target count (see run())
        self.targets = {s.get("label", s["name"]): int(s.get("shots", 1))
                        for s in self.stops}
        self.counts = dict((label, 0) for label in self.targets)
        self.cycle = 0
        self.until_done = rospy.get_param('~until_done', True)
        self.resume()

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(
            rospy.get_param('~cmd_topic', '/cmd_vel'), Twist, queue_size=10)

        self.turn_speed = rospy.get_param('~turn_speed', 0.8)
        self.turn_tolerance = rospy.get_param('~turn_tolerance', 0.05)
        self.turn_timeout = rospy.get_param('~turn_timeout', 20.0)
        self.turn_rate = rospy.get_param('~turn_rate', 20.0)
        self.turn_tf_timeout = rospy.get_param('~turn_tf_timeout', 3.0)
        self.turn_settle = rospy.get_param('~turn_settle', 0.5)

        # the nav stack (amcl + both costmaps) uses base_footprint, not base_link
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

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
        return self.rotate_to(wrap_angle(wp["yaw"] +
                              self.turn_angle(direction)), direction)

    def turn_back(self, wp):
        return self.rotate_to(wrap_angle(wp["yaw"]),
                              self.short_way(wp["yaw"]))

    def turn_angle(self, direction):
        return TURN_LEFT if direction == "left" else TURN_RIGHT

    def short_way(self, target):
        yaw = self.current_yaw()
        if yaw is None:
            return "left"
        return "left" if wrap_angle(target - yaw) >= 0.0 else "right"

    def rotate_to(self, target, direction):
        if self.current_yaw(self.turn_tf_timeout) is None:
            rospy.logwarn("no %s->%s tf, cannot rotate",
                          self.global_frame, self.base_frame)
            return False

        sign = 1.0 if direction == "left" else -1.0
        rate = rospy.Rate(self.turn_rate)
        cmd = Twist()
        start = rospy.Time.now()
        deadline = start + rospy.Duration(self.turn_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            yaw = self.current_yaw()
            if yaw is None:
                rospy.logwarn("lost %s->%s tf while rotating",
                              self.global_frame, self.base_frame)
                self.stop_motion()
                return False
            if abs(wrap_angle(target - yaw)) < self.turn_tolerance:
                self.stop_motion()
                rospy.sleep(self.turn_settle)
                rospy.loginfo("rotated to %.2f rad", target)
                return True
            cmd.angular.z = self.turn_speed * sign
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_motion()
        rospy.sleep(self.turn_settle)
        rospy.logwarn("rotate to %.2f rad timed out after %.0fs",
                      target, self.turn_timeout)
        return False

    def current_yaw(self, timeout=0.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown():
            try:
                trans = self.listener.lookupTransform(
                    self.global_frame, self.base_frame, rospy.Time(0))
                return tf.transformations.euler_from_quaternion(trans[1])[2]
            except Exception:
                if timeout <= 0.0 or rospy.Time.now() >= deadline:
                    return None
                rospy.sleep(0.05)
        return None

    def stop_motion(self):
        self.cmd_pub.publish(Twist())

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

        # resume from what is already on disk: the cycle is expected to be run
        # several times (or in a loop) until every label has enough images
        index = self.counts.get(label, 0)
        while os.path.exists(os.path.join(folder, "%s_%04i.jpg" % (label, index + 1))):
            index += 1
        index += 1

        name = "%s_%04i.jpg" % (label, index)
        path = os.path.join(folder, name)
        try:
            import cv2
            frame = self.bridge.imgmsg_to_cv2(self.frame, 'bgr8')
            cv2.imwrite(path, frame)
            self.shots += 1
            self.counts[label] = index
            rospy.loginfo("saved %s (%d/%d)", path, index,
                          self.targets.get(label, index))
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

        label = stop.get("label", name)
        # in loop mode only top up the labels that are still short, so a long
        # keyword interrupt does not leave one stop with a huge surplus
        want = self.targets.get(label, 1)
        if self.until_done:
            todo = max(0, want - self.counts.get(label, 0))
            todo = min(todo, int(stop.get("shots_per_pass", stop.get("shots", 1))))
        else:
            todo = int(stop.get("shots", 1))

        for _ in range(todo):
            self.save_shot(label)
            rospy.sleep(rospy.get_param('~shot_interval', 0.5))

        # face the waypoint heading again so the next go_to starts from the
        # planned orientation instead of the inspection heading
        if turn in ("left", "right") and not self.turn_back(wp):
            self.stop_motion()
            rospy.logwarn("could not restore heading at %s, holding", name)

    def pending(self):
        # labels that still need images, sorted so the summary is stable
        return {label: self.targets[label] - self.counts.get(label, 0)
                for label in sorted(self.targets)
                if self.counts.get(label, 0) < self.targets[label]}

    def run(self):
        self.client.wait_for_server()
        rospy.loginfo("capture cycle started, %d stops, targets: %s",
                      len(self.stops),
                      ", ".join("%s=%d" % kv for kv in sorted(self.targets.items())))

        try:
            while not rospy.is_shutdown():
                self.cycle += 1
                rospy.loginfo("cycle %d starting (%.0f%% done)",
                              self.cycle, self.progress())
                for stop in self.stops:
                    if rospy.is_shutdown():
                        break
                    self.visit(stop)

                if not self.until_done:
                    break

                missing = self.pending()
                if not missing:
                    rospy.loginfo("all capture targets reached after %d cycles",
                                  self.cycle)
                    break
                rospy.loginfo("cycle %d done, still short: %s",
                              self.cycle,
                              ", ".join("%s(+%d)" % kv for kv in missing.items()))
        except KeyboardInterrupt:
            rospy.loginfo("capture interrupted by KeyboardInterrupt")

        missing = self.pending()
        if missing:
            rospy.logwarn("finished with %d shots, still short: %s",
                          self.shots,
                          ", ".join("%s(+%d)" % kv for kv in missing.items()))
        else:
            rospy.loginfo("capture cycle finished, %d shots, all targets met",
                          self.shots)

    def resume(self):
        """Count the images already in out_dir so a re-run tops up, not refills.

        Filenames are <label>_%04i.jpg, so the highest index tells us how many
        shots this label already has. Without this a fresh process would start
        at zero and write a whole extra batch on top of a finished label.
        """
        for label in self.targets:
            folder = os.path.join(self.out_dir, label)
            index = 0
            while os.path.exists(
                    os.path.join(folder, "%s_%04i.jpg" % (label, index + 1))):
                index += 1
            self.counts[label] = index
        if any(self.counts.values()):
            rospy.loginfo("resuming, found: %s",
                          ", ".join("%s=%d/%d" % (l, self.counts[l], self.targets[l])
                                    for l in sorted(self.targets)
                                    if self.counts[l]))

    def progress(self):
        want = sum(self.targets.values())
        got = sum(min(self.counts.get(label, 0), self.targets[label])
                  for label in self.targets)
        return 100.0 * got / want if want else 100.0


def main():
    rospy.init_node("screenshot_cycle")
    ScreenshotCycle().run()


if __name__ == "__main__":
    main()