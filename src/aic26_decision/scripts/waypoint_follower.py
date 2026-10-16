#!/usr/bin/env python3

"""
Waypoint follower.

Visits every waypoint in config/waypoints.yaml once, in file order, and stops.
Useful to check the map / localisation / planner without running a full
mission.

    (no topics beyond the shared navigation / cmd_vel ones)

The navigation logic comes from mission_common, shared with the other
aic26_decision nodes.
"""

import os
import sys

import rospy
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_common import RobotDriver      # noqa: E402


class WaypointFollower(RobotDriver):
    def __init__(self):
        super(WaypointFollower, self).__init__(light_topic=None)

    def load_waypoints(self, path):
        with open(path) as f:
            return yaml.safe_load(f)["waypoints"]

    def run(self):
        waypoints = self.load_waypoints(rospy.get_param("~waypoints_file"))

        self.wait_ready()
        rospy.loginfo("waypoint follower started, %d waypoints",
                      len(waypoints))

        try:
            for wp in waypoints:
                if rospy.is_shutdown():
                    break
                rospy.loginfo("going to %s", wp["name"])
                if self.go_to(wp):
                    rospy.loginfo("reached %s", wp["name"])
                else:
                    rospy.logwarn("failed to reach %s", wp["name"])
        except KeyboardInterrupt:
            self.stop_motion()
            rospy.loginfo("interrupted by KeyboardInterrupt")
        rospy.loginfo("waypoint follower finished")


def main():
    rospy.init_node("waypoint_follower")
    WaypointFollower().run()


if __name__ == "__main__":
    main()
