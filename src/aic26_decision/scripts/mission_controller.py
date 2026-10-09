#!/usr/bin/env python3
import json

import actionlib
import rospy
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

from aic26_msgs.msg import TrafficLightState

GREEN = 'GREEN'

TURN_LEFT = 1.570796
TURN_RIGHT = -1.570796


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
        # rotate the robot in place by 90 degrees to face the people on a side
        target = dict(wp)
        target["yaw"] = wp["yaw"] + (TURN_LEFT if direction == "left" else TURN_RIGHT)
        self.client.send_goal(make_goal(target))
        if not self.client.wait_for_result():
            rospy.logwarn("turn %s failed", direction)
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

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

        if ttype == "traffic_light":
            self.wait_green(task["light"], task.get("wait", 30.0))

        if ttype == "inspect":
            turn = task.get("turn")
            if turn in ("left", "right"):
                self.turn_in_place(wp, turn)
            rospy.sleep(task.get("dwell", 0.0))
            target = task.get("target")
            if target:
                req = String()
                req.data = target
                self.detect_pub.publish(req)
                rospy.sleep(rospy.get_param('~detect_wait', 1.0))
            self.speak(self.build_message(task))

    def run(self):
        path = rospy.get_param("~tasks_file")
        wp_path = rospy.get_param("~waypoints_file")

        with open(path) as f:
            tasks = yaml.safe_load(f)["tasks"]
        with open(wp_path) as f:
            waypoints = {w["name"]: w for w in yaml.safe_load(f)["waypoints"]}

        self.client.wait_for_server()
        rospy.loginfo("mission started, %d tasks", len(tasks))
        for task in tasks:
            rospy.loginfo("task: %s", task.get("name", task.get("type")))
            self.run_task(task, waypoints)
        rospy.loginfo("mission finished")


def main():
    rospy.init_node("mission_controller")
    MissionController().run()


if __name__ == "__main__":
    main()
