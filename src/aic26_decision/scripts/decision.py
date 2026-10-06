#!/usr/bin/env python3
import rospy
import actionlib
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def load_waypoints(path):
    with open(path) as f:
        return yaml.safe_load(f)["waypoints"]

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

def main():
    rospy.init_node("waypoint_follower")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    waypoints = load_waypoints(rospy.get_param("~waypoints_file"))
    for wp in waypoints:
        rospy.loginfo("Going to %s", wp["name"])
        client.send_goal(make_goal(wp))
        if client.wait_for_result() and client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached %s", wp["name"])
        else:
            rospy.logwarn("Failed to reach %s", wp["name"])

if __name__ == "__main__":
    main()