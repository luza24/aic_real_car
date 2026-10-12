#!/usr/bin/env python3

"""
Showcase cycle.

Walks the inspection route (config/screenshot_cycle.yaml by default) over and
over to demonstrate the whole flow -- navigation, in-place turns, traffic
light waits, the inspection stop with a faked detection result and the spoken
announcement -- without saving anything and without needing any model.

Compared with capture_cycle.py:
    * no screenshots are taken,
    * the loop never ends on its own (Ctrl-C to stop),
    * the sky box time moves on by ~sky_step hours (default 4.5) after every
      pass, so the demo keeps cycling through day / night lighting.

    pub : /speech               (std_msgs/String, announcement)
    pub : /perception/result    (std_msgs/String, json, faked vision output)
    pub : /perception/detect    (std_msgs/String, detection trigger)
    sub : /traffic_light/state  (aic26_msgs/TrafficLightState, sim truth)

The motion / rotation / traffic-light logic comes from mission_common, shared
with the other aic26_decision nodes.
"""

import json
import os
import subprocess
import sys

import rospy
import yaml
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_common import RobotDriver      # noqa: E402

try:
    import rospkg
    HAVE_ROSPKG = True
except ImportError:
    rospkg = None
    HAVE_ROSPKG = False


class DemoCycle(RobotDriver):
    def __init__(self):
        super(DemoCycle, self).__init__(
            light_topic=rospy.get_param('~light_topic', '/traffic_light/state'))

        self.waypoints = self.load_waypoints(rospy.get_param('~waypoints_file'))
        self.stops = self.load_stops(rospy.get_param('~route_file'))
        self.cycle = 0

        # advance the sky box time by this many hours after every pass so the
        # demo keeps showing different lighting (see advance_sky_time)
        self.sky_step = rospy.get_param('~sky_step', 4.5)
        self.day_night_script = self.find_day_night_script()

        self.speech_pub = rospy.Publisher(
            rospy.get_param('~speech_topic', '/speech'), String, queue_size=10)
        self.result_pub = rospy.Publisher(
            rospy.get_param('~result_topic', '/perception/result'),
            String, queue_size=10)
        self.detect_pub = rospy.Publisher(
            rospy.get_param('~detect_topic', '/perception/detect'),
            String, queue_size=10)

    # ---------------------------------------------------------------- input
    def load_waypoints(self, path):
        with open(path) as f:
            return {w["name"]: w for w in yaml.safe_load(f)["waypoints"]}

    def load_stops(self, path):
        with open(path) as f:
            return yaml.safe_load(f)["waypoints"]

    # ---------------------------------------------------------------- output
    def speak(self, text):
        if not text:
            return
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        rospy.loginfo("speak: %s", text)

    def fake_vision(self, stop):
        """Publish the planned result, then announce it like the mission does."""
        result = stop.get("result")
        if not result:
            return
        try:
            payload = json.loads(result)
        except ValueError:
            rospy.logwarn("bad demo result: %s", result)
            return

        target = payload.get("target", "")
        if target:
            trigger = String()
            trigger.data = target
            self.detect_pub.publish(trigger)

        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        rospy.loginfo("fake result: %s", result)

        self.speak(self.build_message(payload))

    def build_message(self, payload):
        target = payload.get("target")
        if target == "people":
            total = payload.get("total", 0)
            intruder = payload.get("intruder", 0)
            if intruder:
                return "Community has %d people, %d non-community people found." \
                       % (total, intruder)
            return "Community has %d people, no non-community people." % total
        if target == "car":
            plate = payload.get("plate", "")
            if plate:
                return "Car here has plate %s." % plate
        return ""

    # ------------------------------------------------------------------ tour
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

        # face the waypoint heading again so the next go_to starts from the
        # planned orientation instead of the inspection heading
        if turn in ("left", "right") and not self.turn_back(wp):
            self.stop_motion()
            rospy.logwarn("could not restore heading at %s, holding", name)

    # ------------------------------------------------------------ sky / state
    def find_day_night_script(self):
        """Locate aic26_gazebo/scripts/day_night.py, or None if not found."""
        explicit = rospy.get_param('~day_night_script', '')
        if explicit:
            return explicit if os.path.isfile(explicit) else None
        if HAVE_ROSPKG:
            try:
                candidate = os.path.join(
                    rospkg.RosPack().get_path('aic26_gazebo'),
                    'scripts', 'day_night.py')
                if os.path.isfile(candidate):
                    return candidate
            except rospkg.ResourceNotFound:
                pass
        return None

    def advance_sky_time(self):
        """Move the sky box time forward by ~sky_step hours via day_night.py."""
        if not self.sky_step:
            return
        if not self.day_night_script:
            rospy.logwarn('day_night.py not found, sky time not advanced')
            return

        nxt = (rospy.get_param('/sky_time', 10.0) + float(self.sky_step)) % 24.0
        try:
            subprocess.call(['python3', self.day_night_script,
                             '_time:=%s' % nxt])
            rospy.loginfo('sky time advanced to %.2fh', nxt)
        except OSError as err:
            rospy.logwarn('could not run day_night.py: %s', err)

    # ------------------------------------------------------------------ run
    def run(self):
        self.wait_ready()
        rospy.loginfo("demo cycle started, %d stops (Ctrl-C to stop)",
                      len(self.stops))

        try:
            while not rospy.is_shutdown():
                self.cycle += 1
                rospy.loginfo("demo cycle %d starting", self.cycle)
                for stop in self.stops:
                    if rospy.is_shutdown():
                        break
                    self.visit(stop)

                # move the sky box forward so the next pass sees new lighting
                self.advance_sky_time()
        except KeyboardInterrupt:
            self.stop_motion()
            rospy.loginfo("demo cycle interrupted by KeyboardInterrupt")
        rospy.loginfo("demo cycle finished")


def main():
    rospy.init_node("demo_cycle")
    DemoCycle().run()


if __name__ == "__main__":
    main()
