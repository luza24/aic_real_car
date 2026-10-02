#!/usr/bin/env python3
import rospy
import actionlib
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def load_waypoints(path):
    with open(path) as f:
        return yaml.safe_load(f)["waypoints"]

def make_goal(wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.orientation.z = wp["yaw"]
    goal.target_pose.pose.orientation.w = 1.0
    return goal

def main():
    rospy.init_node("waypoint_follower")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    waypoints = load_waypoints(rospy.get_param("~waypoints_file"))
    for wp in waypoints:
        rospy.loginfo("Going to %s", wp["name"])
        client.send_goal(make_goal(wp))
        client.wait_for_result()
        rospy.loginfo("Reached %s", wp["name"])

if __name__ == "__main__":
    main()