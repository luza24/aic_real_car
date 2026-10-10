#!/usr/bin/env python3
import json

import actionlib
import rospy
import tf
import tf.transformations
import yaml
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

from aic26_msgs.msg import TrafficLightState

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


def make_goal(wp):
    q = quaternion_from_euler(0.0, 0.0, wp["yaw"])
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal


class MissionController(object):
    def __init__(self):
        self.vision_states = {}   # light model -> state from vision
        self.truth_states = {}    # light model -> state from sim truth
        self.results = {}         # detection results, merged by target

        self.speech_pub = rospy.Publisher(
            rospy.get_param('~speech_topic', '/speech'), String, queue_size=10)
        self.detect_pub = rospy.Publisher(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, queue_size=10)

        rospy.Subscriber(rospy.get_param('~vision_topic', '/perception/traffic_light'),
                         TrafficLightState, self.on_vision)
        rospy.Subscriber(rospy.get_param('~truth_topic', '/traffic_light/state'),
                         TrafficLightState, self.on_truth)
        rospy.Subscriber(rospy.get_param('~result_topic', '/perception/result'),
                         String, self.on_result)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

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

    def on_vision(self, msg):
        if msg.state and msg.state != 'UNKNOWN':
            self.vision_states[msg.model_name] = msg.state

    def on_truth(self, msg):
        self.truth_states[msg.model_name] = msg.state

    def on_result(self, msg):
        try:
            self.results.update(json.loads(msg.data))
        except ValueError:
            rospy.logwarn("bad detection result: %s", msg.data)

    def light_state(self, model):
        # vision first, ground truth as fallback
        if self.vision_states.get(model) in (GREEN, 'RED', 'YELLOW'):
            return self.vision_states[model]
        return self.truth_states.get(model, 'UNKNOWN')

    def go_to(self, wp):
        self.client.send_goal(make_goal(wp))
        if not self.client.wait_for_result():
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def turn_in_place(self, wp, direction):
        """Rotate in place with cmd_vel instead of a move_base goal.

        Sending a goal to move_base makes the local planner re-plan a path and
        honour the costmap, so the robot can wander off the checkpoint or refuse
        to turn. Here the navigation action server is left alone (costmaps stay
        alive) and the spin is closed-loop on the map->base_footprint yaw.
        """
        target = wrap_angle(wp["yaw"] + self.turn_angle(direction))
        return self.rotate_to(target, direction)

    def turn_back(self, wp):
        """Restore the waypoint heading after an inspection turn.

        Without this the robot keeps the side facing the target and the next
        move_base goal is accepted from a wrong heading, which is what makes it
        drive off and push into a wall instead of following the global plan.
        """
        return self.rotate_to(wrap_angle(wp["yaw"]), self.short_way(wp["yaw"]))

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
        # leave on green; if the light never reports, do not block the mission
        start = rospy.Time.now()
        deadline = start + rospy.Duration(timeout)
        no_data_break = start + rospy.Duration(rospy.get_param('~no_data_wait', 10.0))

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state = self.light_state(model)
            if state == GREEN:
                rospy.loginfo("%s is GREEN, passing", model)
                return True
            if state == 'UNKNOWN' and rospy.Time.now() > no_data_break:
                rospy.logwarn("%s has no data, passing", model)
                return True
            rospy.sleep(0.1)

        rospy.logwarn("%s not green within %.0fs, passing anyway", model, timeout)
        return True

    def speak(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        rospy.loginfo("speak: %s", text)

    def build_message(self, task):
        # people tasks report the running totals, others use the fixed text
        if task.get("target") == "people":
            total = self.results.get("total", 0)
            intruder = self.results.get("intruder", 0)
            if intruder:
                return "Community has %d people, %d non-community people found." \
                       % (total, intruder)
            return "Community has %d people, no non-community people." % total
        return task.get("message", "inspection finished")

    def run_task(self, task, waypoints):
        ttype = task.get("type", "goto")
        wp = waypoints[task["waypoint"]] if "waypoint" in task else None

        if ttype == "report":
            self.speak(task.get("message", "report"))
            return

        if wp is None or not self.go_to(wp):
            rospy.logwarn("failed to reach %s", task.get("name", ttype))
            return

        turn = task.get("turn")
        turned = False
        if turn in ("left", "right"):
            # rotate first: at a stop line the camera has to already face the
            # light / people before we start waiting or detecting
            turned = self.turn_in_place(wp, turn)

        if ttype == "traffic_light":
            self.wait_green(task["light"], task.get("wait", 30.0))

        if ttype == "inspect":
            rospy.sleep(task.get("dwell", 0.0))
            target = task.get("target")
            if target:
                req = String()
                req.data = target
                self.detect_pub.publish(req)
                rospy.sleep(rospy.get_param('~detect_wait', 1.0))
            self.speak(self.build_message(task))

        # face the waypoint heading again so the next go_to starts from the
        # planned orientation instead of the inspection heading
        if turned and not self.turn_back(wp):
            self.stop_motion()
            rospy.logwarn("could not restore heading at %s, holding",
                          task.get("name", ttype))

    def run(self):
        path = rospy.get_param("~tasks_file")
        wp_path = rospy.get_param("~waypoints_file")

        with open(path) as f:
            tasks = yaml.safe_load(f)["tasks"]
        with open(wp_path) as f:
            waypoints = {w["name"]: w for w in yaml.safe_load(f)["waypoints"]}

        self.client.wait_for_server()
        rospy.loginfo("mission started, %d tasks", len(tasks))
        try:
            for task in tasks:
                if rospy.is_shutdown():
                    break
                rospy.loginfo("task: %s", task.get("name", task.get("type")))
                self.run_task(task, waypoints)
        except KeyboardInterrupt:
            self.stop_motion()
            rospy.loginfo("mission interrupted by KeyboardInterrupt")
        rospy.loginfo("mission finished")


def main():
    rospy.init_node("mission_controller")
    MissionController().run()


if __name__ == "__main__":
    main()
