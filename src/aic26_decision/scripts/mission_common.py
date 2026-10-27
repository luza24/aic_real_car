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

The module also owns the shared announcement helper: a checkpoint detection
result is formatted into one sentence by :func:`format_result_message` and
published on ``/speech`` (std_msgs/String) by :class:`SpeechAnnouncer`, so
mission_controller and demo_cycle speak the same text in the same language.

Only rospy, actionlib, tf and the ROS messages are used, so the module
also imports cleanly without a running master (handy for unit tests).
"""

import math
import time

import actionlib
import rospy
import rosnode
import tf
import tf.transformations
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

try:
    from aic26_msgs.msg import TrafficLightState
    HAVE_LIGHT_MSG = True
except ImportError:
    TrafficLightState = None
    HAVE_LIGHT_MSG = False

GREEN = 'GREEN'

TURN_LEFT = 1.570796
TURN_RIGHT = -1.570796

LANGUAGES = ('zh', 'en')

# Announcement templates. The Chinese wording is the default; the English
# texts are kept so a run can switch back with ~speech_language.
RESULT_MESSAGES = {
    'zh': {
        'people_intruder': '本区域共检测到 %d 人，其中 %d 人为非社区人员。',
        'people_clear': '本区域共检测到 %d 人，未发现非社区人员。',
        'car_plate': '检测到车辆，车牌号为 %s。',
        'car_no_plate': '检测到车辆，但未能识别车牌。',
    },
    'en': {
        'people_intruder': ('Community has %d people, %d non-community '
                            'people found.'),
        'people_clear': 'Community has %d people, no non-community people.',
        'car_plate': 'Car here has plate %s.',
        'car_no_plate': 'Car here but the plate was not recognised.',
    },
}


def normalize_plate(plate):
    """Strip separators that make the TTS pause inside a plate number."""
    text = str(plate or '')
    for sep in ('·', '•', '.'):
        text = text.replace(sep, '')
    return ''.join(text.split())


def format_result_message(target, payload, language='zh'):
    """Turn one detection payload into one announcement sentence.

    Returns an empty string when there is no usable result, so callers can
    stay silent instead of announcing a stale or made-up value. The payloads
    are the json objects published by the aic26_perception detectors.
    """
    if not isinstance(payload, dict) or not payload:
        return ''
    if language not in RESULT_MESSAGES:
        language = 'zh'
    text = RESULT_MESSAGES[language]

    if target == 'people':
        if 'total' not in payload:
            return ''
        try:
            total = max(0, int(payload.get('total') or 0))
            intruder = max(0, int(payload.get('intruder') or 0))
        except (TypeError, ValueError):
            return ''
        if intruder:
            return text['people_intruder'] % (total, intruder)
        return text['people_clear'] % total

    if target == 'car':
        # total == 0 means no car was seen, not "a car without a plate"
        if 'total' in payload:
            try:
                if int(payload.get('total') or 0) <= 0:
                    return ''
            except (TypeError, ValueError):
                return ''
        plate = normalize_plate(payload.get('plate', ''))
        if plate:
            return text['car_plate'] % plate
        return text['car_no_plate']

    return ''


class SpeechAnnouncer(object):
    """Publish plain-text announcements on one std_msgs/String topic."""

    def __init__(self, topic, language='zh', enabled=True):
        if language not in LANGUAGES:
            rospy.logwarn("unknown speech language %r, using zh", language)
            language = 'zh'
        self.language = language
        self.enabled = bool(enabled) and bool(topic)
        self.pub = None
        if self.enabled:
            self.pub = rospy.Publisher(topic, String, queue_size=10)

    def speak(self, text):
        """Publish ``text``; empty announcements are dropped as well."""
        if not self.enabled or not text:
            return False
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        rospy.loginfo("speak: %s", text)
        return True

    def announce_result(self, target, payload):
        """Announce one detection payload (silent when it has no data)."""
        return self.speak(
            format_result_message(target, payload, self.language))


def wrap_angle(angle):
    """Bring an angle into (-pi, pi]."""
    while angle > 3.141592653589793:
        angle -= 6.283185307179586
    while angle <= -3.141592653589793:
        angle += 6.283185307179586
    return angle


def make_goal(x, y, yaw, frame='map'):
    """Build a move_base goal at ``(x, y, yaw)`` in ``frame``."""
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
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

    def __init__(self, light_topic=None, on_light=None, speech_topic=None,
                 speech_language='zh', speech_enabled=True):
        self.listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(
            rospy.get_param('~cmd_topic', '/cmd_vel'), Twist, queue_size=10)

        self.turn_speed = rospy.get_param('~turn_speed', 0.8)
        self.turn_tolerance = rospy.get_param('~turn_tolerance', 0.05)
        self.turn_min_speed = rospy.get_param('~turn_min_speed', 0.15)
        self.turn_timeout = rospy.get_param('~turn_timeout', 20.0)
        self.turn_rate = rospy.get_param('~turn_rate', 20.0)
        self.turn_tf_timeout = rospy.get_param('~turn_tf_timeout', 3.0)
        self.turn_settle = rospy.get_param('~turn_settle', 0.5)

        # The nav stack (amcl and both costmaps) uses base_footprint, not
        # base_link.
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

        # Optional /speech output. Nodes that do not announce leave this
        # unset, so they keep the same publishers as before.
        self.speech_language = speech_language
        self.announcer = None
        if speech_topic:
            self.announcer = SpeechAnnouncer(
                speech_topic, speech_language, speech_enabled)

    # ------------------------------------------------------------ callbacks
    def on_light(self, msg):
        self.lights[msg.model_name] = msg.state

    # --------------------------------------------------------------- speech
    def speak(self, text):
        """Announce ``text`` when a speech topic was configured."""
        if self.announcer is None:
            return False
        return self.announcer.speak(text)

    def announce_result(self, target, payload):
        """Announce a checkpoint result; silent when there is no payload."""
        if self.announcer is None:
            return False
        return self.announcer.announce_result(target, payload)

    # ------------------------------------------------------------- start-up
    def check_environment(self, timeout=10.0):
        """Refuse to drive when the real and simulated backends are mixed.

        A leftover ``/gazebo`` on the same ROS master as ``/chassis`` makes
        two robots publish TF and consume /cmd_vel; navigation then looks
        broken even though the navigation stack is fine. Wait a few seconds
        for the expected backend node to appear, but fail immediately when
        both are present.
        """
        deadline = time.time() + max(0.0, float(timeout))
        while not rospy.is_shutdown():
            try:
                nodes = set(rosnode.get_node_names())
            except Exception as err:                 # noqa: BLE001
                rospy.logwarn("could not query the ROS node list: %s", err)
                return True
            has_chassis = '/chassis' in nodes
            has_gazebo = '/gazebo' in nodes
            if has_chassis and has_gazebo:
                rospy.logfatal(
                    "both /chassis and /gazebo are running on this ROS "
                    "master; stop the simulation or the hardware stack "
                    "before driving")
                return False
            if has_chassis or has_gazebo:
                return True
            if time.time() >= deadline:
                rospy.logfatal(
                    "neither /chassis nor /gazebo is running; check that "
                    "bringup.launch started the hardware or sim backend")
                return False
            rospy.sleep(0.5)
        return False

    def wait_ready(self, require_nav=True):
        """Block until the dependencies for driving are actually up.

        Waits for the expected backend node, the ``move_base`` action server
        and a usable ``map`` -> ``base_footprint`` transform. Any failure is
        returned to the caller so it can stop instead of reporting a
        successful run that never sent a goal.
        """
        if require_nav:
            if not self.check_environment():
                return False
            if not self.client.wait_for_server(
                    rospy.Duration(self.nav_server_timeout)):
                rospy.logerr("move_base not available after %.0fs; "
                             "not driving", self.nav_server_timeout)
                return False
            rospy.loginfo("move_base action server is up")

        if not self.wait_for_tf(self.ready_timeout):
            rospy.logerr("no %s -> %s tf after %.0fs; not driving",
                         self.global_frame, self.base_frame,
                         self.ready_timeout)
            return False
        rospy.loginfo("%s -> %s tf is up",
                      self.global_frame, self.base_frame)
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
        """Rotate 90 degrees in place to face the target."""
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
            error = wrap_angle(target - yaw)
            if abs(error) < self.turn_tolerance:
                self.stop_motion()
                rospy.sleep(self.turn_settle)
                rospy.loginfo("rotated to %.2f rad", target)
                return True

            # Use the sign of the remaining error, not the originally
            # requested direction: after an overshoot the old fixed-direction
            # command kept rotating away from the target until the timeout.
            # +-pi has no sign to follow, so keep the requested direction.
            if abs(abs(error) - math.pi) < 1e-3:
                direction_sign = sign
            else:
                direction_sign = 1.0 if error >= 0.0 else -1.0

            # Slow down as the target approaches so one 20 Hz command step
            # cannot skip over the tolerance window.
            cmd.angular.z = direction_sign * min(
                self.turn_speed,
                max(self.turn_min_speed, 3.0 * abs(error)))
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

        rospy.logwarn("%s not green within %.0fs, passing anyway", model,
                      timeout)
        return True


