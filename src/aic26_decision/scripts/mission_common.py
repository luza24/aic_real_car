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

Only rospy, actionlib, tf and the ROS messages are used, so the module
also imports cleanly without a running master (handy for unit tests).
"""

import math
import time

import actionlib
import rospy
import tf
import tf.transformations
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

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


def map_pose(mod, odom):
    tx, ty, phi = mod
    ox, oy, theta = odom
    cos_a = math.cos(phi)
    sin_a = math.sin(phi)
    return (cos_a * ox - sin_a * oy + tx,
            sin_a * ox + cos_a * oy + ty,
            wrap_angle(phi + theta))


def map_to_odom(pose, odom):
    x, y, yaw = pose
    ox, oy, theta = odom
    phi = wrap_angle(yaw - theta)
    cos_a = math.cos(phi)
    sin_a = math.sin(phi)
    return (x - (cos_a * ox - sin_a * oy),
            y - (sin_a * ox + cos_a * oy),
            phi)


def frame_correction(data):
    """Return the ``frame_correction`` block of a waypoint file as a tuple."""
    block = data.get('frame_correction') or {}
    return (float(block.get('x', 0.0)), float(block.get('y', 0.0)),
            float(block.get('yaw', 0.0)))


def load_waypoints(path):
    """Load a waypoint file and map it into the map frame.

    ``frame_correction: {x, y, yaw}`` turns the frame the waypoints were
    measured in into the frame of the map the localiser is using:
    ``p_map = R(yaw) * p_venue + (x, y)``. It is measured once with
    ``scripts/frame_calibrate.py``; a zero block leaves the file untouched.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle)
    waypoints = data['waypoints']
    dx, dy, dyaw = frame_correction(data)
    if dx or dy or dyaw:
        cos, sin = math.cos(dyaw), math.sin(dyaw)
        for wp in waypoints:
            x, y = float(wp['x']), float(wp['y'])
            wp['x'] = cos * x - sin * y + dx
            wp['y'] = sin * x + cos * y + dy
            wp['yaw'] = wrap_angle(float(wp['yaw']) + dyaw)
    return waypoints


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

    def __init__(self, light_topic=None, on_light=None):
        self.listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(
            rospy.get_param('~cmd_topic', '/cmd_vel'), Twist, queue_size=10)
        self.initialpose_pub = rospy.Publisher(
            rospy.get_param('~initialpose_topic', '/initialpose'),
            PoseWithCovarianceStamped, queue_size=1)

        self.turn_speed = rospy.get_param('~turn_speed', 0.8)
        self.turn_tolerance = rospy.get_param('~turn_tolerance', 0.05)
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
        # reverse step used when move_base cannot produce a feasible step
        self.back_off_distance = rospy.get_param('~back_off_distance', 0.12)
        self.back_off_speed = rospy.get_param('~back_off_speed', 0.05)
        # start-up gate: refuse to drive when map -> odom is not settled
        self.max_start_offset = float(
            rospy.get_param('~max_start_offset', 0.30))
        self.max_start_yaw = float(
            rospy.get_param('~max_start_yaw', 0.20))
        self.start_gate_timeout = float(
            rospy.get_param('~start_gate_timeout', 30.0))
        self.start_blocked = False
        # pre-position on the start waypoint before the mission drives off
        self.start_align = bool(rospy.get_param('~start_align', True))
        self.start_waypoint = str(rospy.get_param('~start_waypoint', 'start'))
        self.start_align_tolerance = float(
            rospy.get_param('~start_align_tolerance', 0.08))
        self.start_align_yaw = float(
            rospy.get_param('~start_align_yaw', 0.10))
        self.start_align_timeout = float(
            rospy.get_param('~start_align_timeout', 45.0))
        self.start_align_attempts = max(
            1, int(rospy.get_param('~start_align_attempts', 2)))
        self.require_start_align = bool(
            rospy.get_param('~require_start_align', False))
        self.nav_ready = False
        # AMCL confidence gate: a wide particle cloud at start-up makes the
        # estimate swing, and the planner then chases a heading that is wrong
        self.amcl_sigma = None
        self.max_start_sigma = float(
            rospy.get_param('~max_start_sigma', 0.10))
        self.max_runtime_sigma = float(
            rospy.get_param('~max_runtime_sigma', 0.25))
        self.localisation_lost = False
        self._sigma_bad_since = None
        # scan to map consistency: sigma measures particle spread, so it stays
        # small while the pose is 0.2 m off and the robot grinds a pole
        self.scan_ok = None
        self.scan_residual = None
        self.max_scan_residual = float(
            rospy.get_param('~max_scan_residual', 0.18))
        self.scan_timeout = float(rospy.get_param('~scan_bad_timeout', 2.0))
        self._scan_bad_since = None
        # AMCL slides the whole pose while the robot rotates in place (the
        # position is unobservable there), and sigma/residual barely react.
        # The next goal is then planned from the shifted pose, so treat a
        # single large map->odom change as a failure.
        self.odom_jump_limit = float(
            rospy.get_param('~max_odom_jump', 0.10))
        self.odom_yaw_jump_limit = float(
            rospy.get_param('~max_odom_yaw_jump', 0.06))
        self.odom_yaw_accept = float(
            rospy.get_param('~max_odom_yaw_accept', 0.35))
        self.odom_jump_gap = float(
            rospy.get_param('~odom_jump_max_gap', 0.6))
        self.odom_jump_hold = bool(
            rospy.get_param('~odom_jump_hold', True))
        self.odom_jump_recover = bool(
            rospy.get_param('~odom_jump_recover', True))
        self.odom_jump_recover_limit = int(
            rospy.get_param('~odom_jump_recover_limit', 3))
        self.odom_jump_settle = float(
            rospy.get_param('~odom_jump_settle', 0.4))
        self.rotating_speed = float(
            rospy.get_param('~rotating_speed', 0.10))
        self.turn_grace = float(
            rospy.get_param('~odom_jump_turn_grace', 1.0))
        self.odom_jumped = False
        self.odom_jump_recoveries = 0
        self._odom_jump_skip_until = rospy.Time(0)
        self._recover_target = None
        self._last_pose = None
        self._last_mod_yaw = None
        self._last_pose_time = rospy.Time(0)
        # a goal is active but the wheels do not move: collision or stall.
        # Keep the command floor above the chassis dead zone (0.05 m/s only
        # creeps) and the window longer than its ~2 s spin-up, otherwise a
        # normal slow back_off looks stuck.
        self.stuck_cmd = float(rospy.get_param('~stuck_cmd', 0.08))
        self.stuck_distance = float(rospy.get_param('~stuck_distance', 0.005))
        self.stuck_yaw = float(rospy.get_param('~stuck_yaw', 0.02))
        self.stuck_timeout = float(rospy.get_param('~stuck_timeout', 3.0))
        self.motion_blocked = False
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_magnitude = 0.0
        self._cmd_time = rospy.Time(0)
        self._turn_time = rospy.Time(0)
        self._odom_pose = None
        self._odom_reference = None
        self._stuck_since = None
        rospy.Subscriber(rospy.get_param('~scan_ok_topic',
                                         '/localisation/scan_ok'),
                         Bool, self._on_scan_ok, queue_size=1)
        rospy.Subscriber(rospy.get_param('~scan_residual_topic',
                                         '/localisation/scan_residual'),
                         Float32, self._on_scan_residual, queue_size=1)
        rospy.Subscriber(rospy.get_param('~cmd_topic', '/cmd_vel'),
                         Twist, self._on_cmd, queue_size=1)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/odom'),
                         Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(rospy.get_param('~amcl_pose_topic', '/amcl_pose'),
                         PoseWithCovarianceStamped, self._on_amcl,
                         queue_size=5)
        rospy.Timer(rospy.Duration(0.2), self._watch_localisation)

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

    def _on_amcl(self, msg):
        covariance = msg.pose.covariance
        self.amcl_sigma = math.sqrt(max(covariance[0], covariance[7]))

    def _on_scan_ok(self, msg):
        self.scan_ok = msg.data

    def _on_scan_residual(self, msg):
        self.scan_residual = msg.data

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_magnitude = (abs(msg.linear.x) + abs(msg.linear.y) +
                               abs(msg.angular.z))
        self._cmd_time = rospy.Time.now()
        if (abs(msg.angular.z) >= self.rotating_speed and
                abs(msg.linear.x) <= 0.02 and abs(msg.linear.y) <= 0.02):
            self._turn_time = self._cmd_time

    def _on_odom(self, msg):
        pose = msg.pose.pose
        yaw = tf.transformations.euler_from_quaternion(
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w))[2]
        self._odom_pose = (pose.position.x, pose.position.y, yaw)

    def _watch_localisation(self, _event):
        """Stop the robot as soon as the pose stops being trustworthy.

        Three checks, because they fail in different ways: the particle spread
        (amcl sigma), the agreement between scan and map (a confident but wrong
        pose), and a commanded robot that does not move (a collision the pose
        estimate never notices).
        """
        now = rospy.Time.now()
        if self.amcl_sigma is not None and not self.localisation_lost:
            if self.amcl_sigma > self.max_runtime_sigma:
                if self._sigma_bad_since is None:
                    self._sigma_bad_since = now
                    rospy.logwarn("amcl sigma %.2f m above %.2f, watching",
                                  self.amcl_sigma, self.max_runtime_sigma)
                elif (now - self._sigma_bad_since).to_sec() >= 1.0:
                    self._hold("amcl sigma %.2f m for more than 1 s"
                               % self.amcl_sigma)
            else:
                self._sigma_bad_since = None

        if not self.localisation_lost:
            if self.scan_ok is False:
                if self._scan_bad_since is None:
                    self._scan_bad_since = now
                    rospy.logwarn("scan to map residual %.2f m above %.2f, "
                                  "watching", self.scan_residual,
                                  self.max_scan_residual)
                elif (now - self._scan_bad_since).to_sec() >= self.scan_timeout:
                    self._hold("scan to map residual %.2f m for %.1f s"
                               % (self.scan_residual, self.scan_timeout))
            else:
                self._scan_bad_since = None

        self._check_odom_jump(now)
        self._check_stuck(now)

    def _robot_pose(self):
        try:
            trans, rot = self.listener.lookupTransform(
                self.global_frame, 'odom', rospy.Time(0))
            mod = (trans[0], trans[1],
                   tf.transformations.euler_from_quaternion(rot)[2])
            trans, rot = self.listener.lookupTransform(
                'odom', self.base_frame, rospy.Time(0))
            odom = (trans[0], trans[1],
                    tf.transformations.euler_from_quaternion(rot)[2])
        except Exception:
            return None
        return map_pose(mod, odom), mod

    def _check_odom_jump(self, now):
        """Catch a pose that slid in one step (AMCL during in-place turns)."""
        if self.odom_jumped:
            return
        result = self._robot_pose()
        if result is None:
            return
        pose, mod = result
        if now < self._odom_jump_skip_until:
            return
        if self._recover_target is not None:
            target = self._recover_target
            self._recover_target = None
            self._last_pose, self._last_pose_time = pose, now
            self._last_mod_yaw = mod[2]
            if math.hypot(pose[0] - target[0],
                          pose[1] - target[1]) > self.odom_jump_limit:
                self._hold("AMCL kept the slid position after re-seeding")
            return
        previous, previous_time = self._last_pose, self._last_pose_time
        previous_mod_yaw = self._last_mod_yaw
        self._last_pose, self._last_pose_time = pose, now
        self._last_mod_yaw = mod[2]
        if previous is None or previous_mod_yaw is None:
            return
        if (now - previous_time).to_sec() > self.odom_jump_gap:
            return
        moved = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
        turned = abs(wrap_angle(mod[2] - previous_mod_yaw))
        if moved <= self.odom_jump_limit:
            if turned <= self.odom_yaw_jump_limit:
                return
            if turned <= self.odom_yaw_accept:
                rospy.logwarn(
                    "accepted a %.1f deg heading correction without moving "
                    "the position", math.degrees(turned))
                return
        if self._recover_odom_jump(now, pose, previous, moved, turned):
            return
        self.odom_jumped = True
        rospy.logerr(
            "map -> odom jump: the robot pose moved %.2f m / %.1f deg in one "
            "update, every later goal would be off by that much",
            moved, math.degrees(turned))
        if self.odom_jump_hold:
            self._hold("map -> odom jump: robot pose %.2f m / %.1f deg"
                       % (moved, math.degrees(turned)))

    def _recover_odom_jump(self, now, pose, previous, moved, turned):
        if not self.odom_jump_recover:
            return False
        if turned > self.odom_yaw_accept:
            return False
        if not self._rotating_in_place():
            return False
        if self.scan_ok is False:
            return False
        if (self.amcl_sigma is not None and
                self.amcl_sigma > self.max_runtime_sigma):
            return False
        if self.odom_jump_recoveries >= self.odom_jump_recover_limit:
            return False
        target = (previous[0], previous[1], pose[2])
        self._publish_pose(target)
        self._recover_target = target
        self._odom_jump_skip_until = now + rospy.Duration(self.odom_jump_settle)
        self.odom_jump_recoveries += 1
        rospy.logwarn(
            "robot pose moved %.2f m / %.1f deg while turning in place: "
            "keeping the position (%.2f, %.2f), re-seeding AMCL at %.0f deg",
            moved, math.degrees(turned), target[0], target[1],
            math.degrees(target[2]))
        return True

    def _rotating_in_place(self):
        now = rospy.Time.now()
        vx, vy, wz = self._cmd
        if abs(vx) > 0.02 or abs(vy) > 0.02:
            return False
        if ((now - self._cmd_time).to_sec() <= 0.5 and
                abs(wz) >= self.rotating_speed):
            return True
        return (now - self._turn_time).to_sec() <= self.turn_grace

    def _publish_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.global_frame
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, pose[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.0025
        covariance[35] = 0.0025
        msg.pose.covariance = covariance
        self.initialpose_pub.publish(msg)

    def _check_stuck(self, now):
        """A goal is active and the wheels turn, but the robot goes nowhere."""
        if self.motion_blocked or self._odom_pose is None:
            return
        # a planner that stops publishing is not a stuck robot: only trust a
        # command that arrived recently (TEB goes silent instead of sending 0)
        if ((now - self._cmd_time).to_sec() > 0.5 or
                self._cmd_magnitude < self.stuck_cmd):
            self._stuck_since = None
            self._odom_reference = self._odom_pose
            return
        if self._odom_reference is None:
            self._odom_reference = self._odom_pose
            self._stuck_since = now
            return
        moved = math.hypot(self._odom_pose[0] - self._odom_reference[0],
                           self._odom_pose[1] - self._odom_reference[1])
        delta = self._odom_pose[2] - self._odom_reference[2]
        turned = abs(math.atan2(math.sin(delta), math.cos(delta)))
        if moved > self.stuck_distance or turned > self.stuck_yaw:
            self._odom_reference = self._odom_pose
            self._stuck_since = None
            return
        if self._stuck_since is None:
            self._stuck_since = now
            return
        if (now - self._stuck_since).to_sec() >= self.stuck_timeout:
            self._hold("commanded %.2f but odom moved %.3f m in %.1f s"
                       % (self._cmd_magnitude, moved, self.stuck_timeout),
                       blocked=True)

    def _hold(self, reason, blocked=False):
        if self.localisation_lost or self.motion_blocked:
            return
        if blocked:
            self.motion_blocked = True
        else:
            self.localisation_lost = True
        rospy.logerr("%s: cancelling the goal and holding the robot", reason)
        try:
            self.client.cancel_goal()
        except Exception:
            pass
        self.stop_motion()

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
                              "continuing without it",
                              self.nav_server_timeout)
                return False
            rospy.loginfo("move_base action server is up")
            self.nav_ready = True

        if self.wait_for_tf(self.ready_timeout):
            rospy.loginfo("%s -> %s tf is up",
                          self.global_frame, self.base_frame)
        else:
            rospy.logwarn("no %s -> %s tf after %.0fs, continuing",
                          self.global_frame, self.base_frame,
                          self.ready_timeout)
        self.check_start_pose()
        if self.nav_ready and not self.start_blocked:
            self.align_to_start()
        return True

    def align_to_start(self):
        """Drive onto the ``start`` waypoint before the mission begins.

        A hand-placed robot is usually a few centimetres and degrees off. The
        mission is only started once move_base has parked it on the waypoint,
        so every route begins from the same pose. Skipped when the robot is
        already inside the tolerance.
        """
        if not self.start_align:
            return True
        path = rospy.get_param('~waypoints_file', '')
        if not path:
            rospy.logwarn("start align: no ~waypoints_file, skipping")
            return True
        try:
            waypoints = load_waypoints(path)
        except Exception as exc:
            rospy.logwarn("start align: cannot read %s: %s", path, exc)
            return True
        target = None
        for candidate in waypoints:
            if candidate.get('name') == self.start_waypoint:
                target = candidate
                break
        if target is None:
            rospy.logwarn("start align: %s has no '%s' waypoint, skipping",
                          path, self.start_waypoint)
            return True

        pose = self.current_pose(1.0)
        yaw = self.current_yaw(1.0)
        if pose is not None and yaw is not None:
            distance = math.hypot(pose[0] - target['x'], pose[1] - target['y'])
            error = abs(wrap_angle(yaw - target['yaw']))
            if (distance <= self.start_align_tolerance and
                    error <= self.start_align_yaw):
                rospy.loginfo(
                    "start align: already on '%s' (%.3f m / %.1f deg)",
                    self.start_waypoint, distance, math.degrees(error))
                return True
            rospy.loginfo(
                "start align: %.2f m / %.1f deg from '%s', parking first",
                distance, math.degrees(error), self.start_waypoint)

        for attempt in range(self.start_align_attempts):
            self.client.send_goal(make_goal(
                target['x'], target['y'], target['yaw'], self.global_frame))
            if self.client.wait_for_result(
                    rospy.Duration(self.start_align_timeout)):
                if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("start align: on '%s'", self.start_waypoint)
                    return True
            else:
                self.client.cancel_goal()
            self.stop_motion()
            rospy.logwarn("start align: attempt %d failed", attempt + 1)

        if self.require_start_align:
            self.start_blocked = True
            rospy.logerr("start align: could not reach '%s', mission blocked",
                         self.start_waypoint)
        else:
            rospy.logerr("start align: could not reach '%s', continuing",
                         self.start_waypoint)
        return False

    def check_start_pose(self):
        """Block driving while the localiser and the odometry disagree.

        ``map -> odom`` is the correction AMCL had to apply to the wheel
        odometry. A large value means the robot was placed somewhere other
        than the configured start pose (or the map frame does not match the
        waypoint frame); driving then leaves the lane.
        """
        deadline = time.time() + max(0.0, self.start_gate_timeout)
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                    self.global_frame, 'odom', rospy.Time(0))
            except Exception:
                trans = None
            if trans is not None:
                distance = math.hypot(trans[0], trans[1])
                yaw = abs(tf.transformations.euler_from_quaternion(rot)[2])
                settled = (self.amcl_sigma is None or
                           self.amcl_sigma <= self.max_start_sigma)
                if (distance <= self.max_start_offset and
                        yaw <= self.max_start_yaw and settled):
                    rospy.loginfo(
                        "start gate: map -> odom = %.3f m / %.1f deg, "
                        "amcl sigma %.3f m", distance, math.degrees(yaw),
                        self.amcl_sigma if self.amcl_sigma is not None else -1)
                    return True
                rospy.logwarn(
                    "start gate: map -> odom = %.2f m / %.1f deg, "
                    "amcl sigma %.3f m (limits %.2f m / %.1f deg / %.2f m), "
                    "waiting", distance, math.degrees(yaw),
                    self.amcl_sigma if self.amcl_sigma is not None else -1,
                    self.max_start_offset, math.degrees(self.max_start_yaw),
                    self.max_start_sigma)
            if time.time() >= deadline:
                break
            rospy.sleep(0.5)

        self.start_blocked = True
        rospy.logerr(
            "localisation does not match the odometry; the mission will not "
            "drive. Put the robot on the documented start pose and restart, "
            "or correct frame_correction in the waypoint file.")
        return False

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
        """Navigate to a waypoint dict with ``x``, ``y`` and ``yaw``.

        On failure the robot backs off once and retries: TEB can get stuck
        with the footprint inside the obstacle margin, where every trajectory
        is infeasible and no move_base recovery helps.
        """
        if self._send_goal(wp):
            return True
        rospy.logwarn("%s did not succeed, backing off %.2f m and retrying",
                      wp.get("name", "goal"), self.back_off_distance)
        if not self.back_off(self.back_off_distance):
            return False
        return self._send_goal(wp)

    def _send_goal(self, wp):
        """Send one move_base goal and report whether it succeeded."""
        if (self.start_blocked or self.localisation_lost or
                self.motion_blocked):
            rospy.logerr_throttle(
                10.0, "localisation not trusted, not driving to %s",
                wp.get("name", "goal"))
            return False
        self.client.send_goal(make_goal(wp["x"], wp["y"], wp["yaw"],
                                        self.global_frame))
        if not self.client.wait_for_result(rospy.Duration(self.nav_timeout)):
            rospy.logwarn("move_base did not finish in %.0fs, cancelling",
                          self.nav_timeout)
            self.client.cancel_goal()
            self.stop_motion()
            return False
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED

    # ------------------------------------------------------------- recovery
    def current_pose(self, timeout=0.0):
        """Return ``(x, y)`` of the robot in the global frame, or None."""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown():
            try:
                trans = self.listener.lookupTransform(
                    self.global_frame, self.base_frame, rospy.Time(0))
                return trans[0][0], trans[0][1]
            except Exception:
                if timeout <= 0.0 or rospy.Time.now() >= deadline:
                    return None
                rospy.sleep(0.05)
        return None

    def back_off(self, distance):
        """Drive straight backwards by ``distance`` metres on cmd_vel."""
        if (self.start_blocked or self.localisation_lost or
                self.motion_blocked):
            rospy.logerr("localisation not trusted, not backing off")
            return False
        start = self.current_pose(self.turn_tf_timeout)
        if start is None:
            rospy.logwarn("no pose, cannot back off")
            return False

        speed = abs(self.back_off_speed)
        cmd = Twist()
        cmd.linear.x = -speed
        rate = rospy.Rate(20.0)
        deadline = rospy.Time.now() + rospy.Duration(
            abs(distance) / speed * 3.0 + 1.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.cmd_pub.publish(cmd)
            pose = self.current_pose()
            if pose is not None and math.hypot(
                    pose[0] - start[0], pose[1] - start[1]) >= abs(distance):
                break
            rate.sleep()

        self.stop_motion()
        rospy.sleep(0.2)
        rospy.loginfo("backed off %.2f m", distance)
        return True

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
        if (self.start_blocked or self.localisation_lost or
                self.motion_blocked):
            rospy.logerr("localisation not trusted, not rotating")
            return False
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

        rospy.logwarn("%s not green within %.0fs, passing anyway", model,
                      timeout)
        return True


