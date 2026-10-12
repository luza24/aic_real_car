#!/usr/bin/env python3

"""
Shared building blocks for the aic26_decision nodes.

mission_controller, screenshot_cycle, demo_cycle and waypoint_follower all
drive the same robot over the same map, so the navigation / in-place rotation
/ traffic-light logic lives here instead of being copied into every node.

The one entry point the nodes use is :class:`RobotDriver`, which wraps

    * move_base navigation with a bounded wait,
    * closed-loop in-place rotation on tf (no move_base goal, so the local
      planner cannot wander off the checkpoint),
    * a traffic-light wait that leans on the sim truth when the vision node
      is not up,

and a :meth:`RobotDriver.wait_ready` start-up gate so the nodes tolerate being
launched before (or after) the simulation and the navigation stack.

Only rclpy/rospy, actionlib, tf and the ROS messages are used, so the module
also imports cleanly without a running master (handy for unit tests).
"""

import time

import actionlib
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
    """Bring an angle into (-pi, pi]."""
    while angle > 3.141592653589793:
        angle -= 6.283185307179586
    while angle <= -3.141592653589793:
        angle += 6.283185307179586
    return angle


def make_goal(x, y, yaw, frame='map'):
    """Build a move_base goal at ``(x, y, yaw)`` in ``frame``."""
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal


class RobotDriver(object):
    """Common navigation / rotation / traffic-light helper for the nodes.

    Subclasses publish their own topics and call the ``go_to`` / ``turn_*`` /
    ``wait_green`` helpers. All start-up waits are parameterised so the nodes
    can be launched in any order relative to the rest of the bringup.
    """

    def __init__(self, light_topic=None, on_light=None):
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

        # how long to wait on the various start-up dependencies
        self.ready_timeout = rospy.get_param('~ready_timeout', 60.0)
        self.nav_server_timeout = rospy.get_param('~nav_server_timeout', 60.0)
        self.nav_timeout = rospy.get_param('~nav_timeout', 120.0)

        self.lights = {}          # light model -> last seen state

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        if light_topic and HAVE_LIGHT_MSG:
            rospy.Subscriber(light_topic, TrafficLightState,
                             on_light or self.on_light)
        elif light_topic:
            rospy.logwarn("aic26_msgs/TrafficLightState missing, "
                          "traffic lights are treated as green")

    # ------------------------------------------------------------ callbacks
    def on_light(self, msg):
        self.lights[msg.model_name] = msg.state

    # ------------------------------------------------------------- start-up
    def wait_ready(self, require_nav=True):
        """Block until the launch-order independent dependencies are up.

        Waits for the ``move_base`` action server and for a usable ``map`` ->
        ``base_footprint`` transform, so a node started before gazebo / the
        navigation stack simply idles instead of failing or driving blind.
        """
        if require_nav:
            if not self.client.wait_for_server(
                    rospy.Duration(self.nav_server_timeout)):
                rospy.logwarn("move_base not available after %.0fs, "
                              "continuing without it", self.nav_server_timeout)
                return False
            rospy.loginfo("move_base action server is up")

        if self.wait_for_tf(self.ready_timeout):
            rospy.loginfo("%s -> %s tf is up",
                          self.global_frame, self.base_frame)
        else:
            rospy.logwarn("no %s -> %s tf after %.0fs, continuing",
                          self.global_frame, self.base_frame, self.ready_timeout)
        return True

    def wait_for_tf(self, timeout):
        """Wait up to ``timeout`` s for the map -> base transform."""
        deadline = time.time() + max(0.0, float(timeout))
        while not rospy.is_shutdown():
            if self.current_yaw(0.0) is not None:
                return True
            if time.time() >= deadline:
                return False
            rospy.sleep(0.1)
        return False

    # ----------------------------------------------------------- navigation
    def go_to(self, wp):
        """Navigate to a waypoint dict with ``x``, ``y`` and ``yaw``."""
        self.client.send_goal(make_goal(wp["x"], wp["y"], wp["yaw"],
                                        self.global_frame))
        if not self.client.wait_for_result(rospy.Duration(self.nav_timeout)):
            rospy.logwarn("move_base did not finish in %.0fs, cancelling",
                          self.nav_timeout)
            self.client.cancel_goal()
            self.stop_motion()
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    # ------------------------------------------------------------ rotation
    def turn_in_place(self, wp, direction):
        """Rotate 90 degrees in place to face the target beside the waypoint."""
        target = wrap_angle(wp["yaw"] + self.turn_angle(direction))
        return self.rotate_to(target, direction)

    def turn_back(self, wp):
        """Restore the waypoint heading after an inspection turn."""
        return self.rotate_to(wrap_angle(wp["yaw"]), self.short_way(wp["yaw"]))

    def turn_angle(self, direction):
        return TURN_LEFT if direction == "left" else TURN_RIGHT

    def short_way(self, target):
        yaw = self.current_yaw()
        if yaw is None:
            return "left"
        return "left" if wrap_angle(target - yaw) >= 0.0 else "right"

    def rotate_to(self, target, direction):
        """Closed-loop in-place rotation on the map -> base yaw.

        Uses cmd_vel rather than a move_base goal: a navigation goal makes the
        local planner re-plan and honour the costmap, which can wander off the
        checkpoint or refuse to turn.
        """
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

    # -------------------------------------------------------- traffic light
    def light_state(self, model):
        return self.lights.get(model, 'UNKNOWN')

    def wait_green(self, model, timeout):
        """Wait for ``model`` to be GREEN; never block the run for long.

        Treats "no data" and a timeout the same way so a missing light does
        not stall a mission that has to keep moving.
        """
        if not HAVE_LIGHT_MSG:
            return True
        start = rospy.Time.now()
        deadline = start + rospy.Duration(timeout)
        no_data_break = start + rospy.Duration(
            rospy.get_param('~no_data_wait', 10.0))

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


