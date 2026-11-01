#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive to map-frame waypoints without move_base or TEB.

The node replaces the move_base action client used by ``mission_common``
with a small point-to-point controller that publishes ``/cmd_vel``
directly:

    ROTATE -> DRIVE -> ALIGN_YAW -> next waypoint

* Pose feedback comes from the ``map`` -> ``base_footprint`` transform that
  AMCL publishes; wheel odometry is never used for control.
* The chassis stops on its own after ``cmd_timeout`` (0.5 s) without a
  command, so the control loop runs at a fixed rate and always publishes.
* Safety: front-sector lidar stop, TF/scan/AMCL watchdogs, an exclusive
  ``/cmd_vel`` publisher check, a straight-segment check on the map, and a
  zero-velocity burst on shutdown.

This node is deliberately standalone: it does not import ``mission_common``
(whose RobotDriver always creates a move_base action client) and it does
not run the task/detection/speech logic of ``mission_controller``.

Example:

    roslaunch aic26_bringup direct.launch backend:=real \
        names:="traffic_light_1" v_max:=0.10

    # pre-flight only: check waypoint segments against the map, then exit
    roslaunch aic26_bringup direct.launch backend:=real check_only:=true
"""

import math
import os
import time
import traceback

import rospy
import rosgraph
import rosnode
import tf
import yaml
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def wrap_angle(angle):
    """Normalise an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


class DirectWaypointController(object):
    # Phase names used by the per-waypoint state machine.
    ROTATE = 'ROTATE'
    DRIVE_SETTLE = 'DRIVE_SETTLE'
    DRIVE = 'DRIVE'
    ALIGN_SETTLE = 'ALIGN_SETTLE'
    ALIGN_YAW = 'ALIGN_YAW'
    DONE_SETTLE = 'DONE_SETTLE'
    DWELL = 'DWELL'

    def __init__(self):
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.status_topic = rospy.get_param(
            '~status_topic', '/direct_waypoint/status')
        self.control_rate = float(rospy.get_param('~control_rate', 20.0))
        self.status_rate = float(rospy.get_param('~status_rate', 2.0))
        self.laser_yaw_offset = math.radians(
            float(rospy.get_param('~laser_yaw_offset_deg', 0.0)))

        # waypoints and run mode
        self.waypoints_file = rospy.get_param('~waypoints_file', '')
        self.names = self._split_names(rospy.get_param('~names', ''))
        self.start_at = rospy.get_param('~start_at', '')
        self.loop = bool(rospy.get_param('~loop', False))
        self.max_loops = int(rospy.get_param('~max_loops', 0))
        self.continue_on_failure = bool(
            rospy.get_param('~continue_on_failure', False))
        self.honor_dwell = bool(rospy.get_param('~honor_dwell', False))
        self.align_final_yaw = bool(
            rospy.get_param('~align_final_yaw', True))
        self.dry_run = bool(rospy.get_param('~dry_run', False))
        self.dry_run_seconds = float(
            rospy.get_param('~dry_run_seconds', 10.0))
        self.check_only = bool(rospy.get_param('~check_only', False))

        # control gains and limits
        self.v_max = float(rospy.get_param('~v_max', 0.15))
        self.v_min = float(rospy.get_param('~v_min', 0.05))
        self.w_max = float(rospy.get_param('~w_max', 0.45))
        self.turn_w_max = float(rospy.get_param('~turn_w_max', 0.35))
        self.turn_w_min = float(rospy.get_param('~turn_w_min', 0.10))
        self.k_d = float(rospy.get_param('~k_d', 0.8))
        self.k_theta = float(rospy.get_param('~k_theta', 1.0))
        self.k_turn = float(rospy.get_param('~k_turn', 1.2))
        self.stop_distance = float(rospy.get_param('~stop_distance', 0.05))
        self.position_tolerance = float(
            rospy.get_param('~position_tolerance', 0.08))
        self.e_reorient = float(rospy.get_param('~e_reorient', 0.60))
        self.turn_tol = float(rospy.get_param('~turn_tol', 0.06))
        self.yaw_tol = float(rospy.get_param('~yaw_tol', 0.06))
        self.settle_time = float(rospy.get_param('~settle_time', 0.30))
        self.linear_accel = float(rospy.get_param('~linear_accel', 0.5))
        self.angular_accel = float(rospy.get_param('~angular_accel', 1.0))
        self.rotate_skip_distance = float(
            rospy.get_param('~rotate_skip_distance', 0.25))

        # safety
        self.obstacle_half_angle = math.radians(
            float(rospy.get_param('~obstacle_half_angle_deg', 25.0)))
        self.obstacle_stop_distance = float(
            rospy.get_param('~obstacle_stop_distance', 0.25))
        self.obstacle_resume_distance = float(
            rospy.get_param('~obstacle_resume_distance', 0.35))
        self.obstacle_clear_hold = float(
            rospy.get_param('~obstacle_clear_hold', 0.8))
        self.obstacle_wait = float(rospy.get_param('~obstacle_wait', 10.0))
        self.scan_timeout = float(rospy.get_param('~scan_timeout', 0.5))
        self.tf_fail_timeout = float(
            rospy.get_param('~tf_fail_timeout', 0.5))
        self.tf_recovery_wait = float(
            rospy.get_param('~tf_recovery_wait', 2.0))
        self.use_amcl_watchdog = bool(
            rospy.get_param('~use_amcl_watchdog', True))
        self.amcl_pose_timeout = float(
            rospy.get_param('~amcl_pose_timeout', 3.0))
        self.odom_timeout = float(rospy.get_param('~odom_timeout', 0.5))
        self.odom_moving_linear = float(
            rospy.get_param('~odom_moving_linear', 0.02))
        self.odom_moving_angular = float(
            rospy.get_param('~odom_moving_angular', 0.05))
        self.amcl_jump_xy = float(rospy.get_param('~amcl_jump_xy', 0.30))
        self.amcl_jump_yaw = float(rospy.get_param('~amcl_jump_yaw', 0.35))
        self.amcl_cov_xy_max = float(
            rospy.get_param('~amcl_cov_xy_max', 0.30))
        self.amcl_cov_yaw_max = float(
            rospy.get_param('~amcl_cov_yaw_max', 0.10))
        self.amcl_cov_hold = float(rospy.get_param('~amcl_cov_hold', 0.5))
        self.amcl_recovery_wait = float(
            rospy.get_param('~amcl_recovery_wait', 2.0))
        self.exclusive_cmd = bool(
            rospy.get_param('~exclusive_cmd', True))
        self.cmd_check_period = float(
            rospy.get_param('~cmd_check_period', 1.0))
        allowed = rospy.get_param('~allowed_cmd_publishers', [])
        self.allowed_cmd_publishers = set(allowed)
        self.require_backend = rospy.get_param('~require_backend', 'real')

        self.segment_check = bool(
            rospy.get_param('~segment_check', True))
        self.segment_half_width = float(
            rospy.get_param('~segment_half_width', 0.15))
        self.map_occupied_threshold = int(
            rospy.get_param('~map_occupied_threshold', 65))
        self.map_unknown_blocked = bool(
            rospy.get_param('~map_unknown_blocked', True))

        self.zero_burst_sec = float(
            rospy.get_param('~zero_burst_sec', 0.8))
        self.zero_burst_rate = float(
            rospy.get_param('~zero_burst_rate', 20.0))
        self.disable_chassis_on_exit = bool(
            rospy.get_param('~disable_chassis_on_exit', False))
        self.chassis_enable_service = rospy.get_param(
            '~chassis_enable_service', '/chassis/enable')

        # timeouts
        self.ready_timeout = float(rospy.get_param('~ready_timeout', 60.0))
        self.startup_settle = float(
            rospy.get_param('~startup_settle', 2.0))
        self.turn_timeout = float(rospy.get_param('~turn_timeout', 20.0))
        self.drive_timeout = float(rospy.get_param('~drive_timeout', 120.0))
        self.align_timeout = float(rospy.get_param('~align_timeout', 15.0))
        self.stall_window = float(rospy.get_param('~stall_window', 4.0))
        self.stall_distance = float(
            rospy.get_param('~stall_distance', 0.03))

        # runtime state
        self.scan_msg = None
        self.scan_stamp = 0.0
        self.odom_stamp = 0.0
        self.odom_twist = (0.0, 0.0, 0.0)
        self.map_msg = None
        self.amcl_cov = (0.0, 0.0, 0.0)
        self.amcl_stamp = 0.0
        self.amcl_jump_time = 0.0
        self.amcl_cov_bad_since = None
        self._amcl_stalled = False
        self._prev_amcl = None

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self._cmd_conflict = None
        self._tf_fail_since = None
        self._blocked_since = None
        self._block_reason = ''
        self._obstacle_held = False
        self._obstacle_clear_since = None
        self._last_cmd_check = 0.0
        self._last_status = 0.0
        self._shutdown_done = False
        self._status = 'STARTING'

        self.rate = rospy.Rate(self.control_rate)
        self.cmd_pub = rospy.Publisher(
            self.cmd_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan,
                         queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom,
                         queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self._on_amcl, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self._on_map, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------ callbacks
    @staticmethod
    def _split_names(value):
        if not value:
            return []
        return [name.strip() for name in str(value).split(',')
                if name.strip()]

    def _on_scan(self, msg):
        self.scan_msg = msg
        self.scan_stamp = time.time()

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_odom(self, msg):
        self.odom_stamp = time.time()
        twist = msg.twist.twist
        self.odom_twist = (twist.linear.x, twist.linear.y,
                           twist.angular.z)

    def _odom_moving(self, now):
        """True when the wheels report real motion (not just a command)."""
        if self.odom_stamp == 0.0 or now - self.odom_stamp > self.odom_timeout:
            return False
        vx, vy, wz = self.odom_twist
        return (abs(vx) > self.odom_moving_linear or
                abs(vy) > self.odom_moving_linear or
                abs(wz) > self.odom_moving_angular)

    def _on_amcl(self, msg):
        now = time.time()
        pose = msg.pose.pose
        yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w])[2]
        if self._prev_amcl is not None:
            px, py, pyaw, ptime = self._prev_amcl
            if now - ptime <= 2.0:
                jump = math.hypot(pose.position.x - px,
                                  pose.position.y - py)
                dyaw = abs(wrap_angle(yaw - pyaw))
                if jump > self.amcl_jump_xy or dyaw > self.amcl_jump_yaw:
                    rospy.logwarn(
                        'AMCL jumped %.2f m / %.2f rad in one update; '
                        'holding motion', jump, dyaw)
                    self.amcl_jump_time = now
        self._prev_amcl = (pose.position.x, pose.position.y, yaw, now)
        cov = msg.pose.covariance
        self.amcl_cov = (cov[0], cov[7], cov[35])
        self.amcl_stamp = now

    # ------------------------------------------------------------- waypoints
    def _load_waypoints(self, path):
        if not path:
            raise RuntimeError('~waypoints_file is empty')
        if not os.path.exists(path):
            raise RuntimeError('waypoints file not found: %s' % path)
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        correction = data.get('frame_correction') or {}
        cx = float(correction.get('x', 0.0))
        cy = float(correction.get('y', 0.0))
        cyaw = float(correction.get('yaw', 0.0))
        waypoints = []
        for item in data.get('waypoints', []):
            x = float(item['x'])
            y = float(item['y'])
            yaw = float(item.get('yaw', 0.0))
            if cx or cy or cyaw:
                cos_a = math.cos(cyaw)
                sin_a = math.sin(cyaw)
                x, y = (cos_a * x - sin_a * y + cx,
                        sin_a * x + cos_a * y + cy)
                yaw = wrap_angle(yaw + cyaw)
            waypoints.append({
                'name': str(item['name']),
                'x': x,
                'y': y,
                'yaw': yaw,
                'dwell': float(item.get('dwell') or 0.0),
            })
        if not waypoints:
            raise RuntimeError('no waypoints in %s' % path)
        return waypoints

    def _select_waypoints(self, waypoints):
        by_name = {wp['name']: wp for wp in waypoints}
        if self.names:
            missing = [n for n in self.names if n not in by_name]
            if missing:
                raise RuntimeError(
                    'unknown waypoint name(s): %s' % ', '.join(missing))
            return [by_name[name] for name in self.names]
        if self.start_at:
            names = [wp['name'] for wp in waypoints]
            if self.start_at not in names:
                raise RuntimeError(
                    'start_at waypoint not found: %s' % self.start_at)
            return waypoints[names.index(self.start_at):]
        return waypoints

    # ------------------------------------------------------------------ pose
    def _lookup_pose(self):
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                self.global_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None
        yaw = euler_from_quaternion(rotation)[2]
        return translation[0], translation[1], yaw

    # ---------------------------------------------------------------- safety
    def _front_min(self):
        msg = self.scan_msg
        if msg is None:
            return float('inf')
        limit = min(msg.range_max, 6.0)
        best = float('inf')
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and \
                    msg.range_min < distance <= limit:
                bearing = wrap_angle(angle - self.laser_yaw_offset)
                if abs(bearing) <= self.obstacle_half_angle and \
                        distance < best:
                    best = distance
            angle += msg.angle_increment
        return best

    def _safety_state(self, now):
        """Return ('ok'|'blocked'|'fatal', reason)."""
        if self._cmd_conflict:
            return 'fatal', self._cmd_conflict

        # lidar front sector, with hysteresis so a marginal reading does not
        # toggle the robot on and off
        front = self._front_min()
        if self.scan_msg is None or now - self.scan_stamp > self.scan_timeout:
            return 'blocked', 'scan timeout'
        if front < self.obstacle_stop_distance:
            self._obstacle_held = True
            self._obstacle_clear_since = None
            return 'blocked', 'obstacle at %.2f m' % front
        if self._obstacle_held:
            if front < self.obstacle_resume_distance:
                self._obstacle_clear_since = None
                return 'blocked', 'obstacle clearing (%.2f m)' % front
            if self._obstacle_clear_since is None:
                self._obstacle_clear_since = now
            elif now - self._obstacle_clear_since < self.obstacle_clear_hold:
                return 'blocked', 'obstacle clear hold'
            else:
                self._obstacle_held = False
                self._obstacle_clear_since = None

        if not self.use_amcl_watchdog:
            return 'ok', ''

        # AMCL only publishes /amcl_pose when its filter updates, and the
        # filter stops updating while the robot is stationary. Arm the stale
        # check only when odometry shows real motion, and latch it until a
        # fresh pose arrives: clearing it from inside the blocked branch
        # (which stops the robot) would make the watchdog flap every few
        # seconds.
        stale = (self.amcl_stamp > 0.0 and
                 now - self.amcl_stamp > self.amcl_pose_timeout)
        if stale and self._odom_moving(now):
            if not self._amcl_stalled:
                rospy.logwarn(
                    'AMCL pose is %.1f s old while odometry is moving; '
                    'holding until it refreshes', now - self.amcl_stamp)
            self._amcl_stalled = True
        elif self._amcl_stalled and not stale:
            rospy.loginfo('AMCL pose refreshed; resuming')
            self._amcl_stalled = False
        if self._amcl_stalled:
            return 'blocked', 'amcl pose stale'
        if now - self.amcl_jump_time < self.amcl_recovery_wait:
            return 'blocked', 'amcl jump'

        cov_xy = max(self.amcl_cov[0], self.amcl_cov[1])
        cov_yaw = self.amcl_cov[2]
        if cov_xy > self.amcl_cov_xy_max or cov_yaw > self.amcl_cov_yaw_max:
            if self.amcl_cov_bad_since is None:
                self.amcl_cov_bad_since = now
            elif now - self.amcl_cov_bad_since > self.amcl_cov_hold:
                return 'blocked', 'amcl covariance high'
        else:
            self.amcl_cov_bad_since = None
        return 'ok', ''

    def _check_cmd_owners(self):
        """Return the set of nodes currently publishing on the cmd topic."""
        try:
            master = rosgraph.Master(rospy.get_name())
            state = master.getSystemState()
        except Exception as err:  # noqa: BLE001
            rospy.logwarn('could not query ROS master: %s', err)
            return set()
        owners = set()
        for topic, nodes in state[0]:
            if topic == self.cmd_topic:
                owners.update(nodes)
        return owners

    def _verify_cmd_exclusive(self):
        owners = self._check_cmd_owners()
        others = owners - {rospy.get_name()} - self.allowed_cmd_publishers
        if others:
            self._cmd_conflict = (
                'another node publishes %s: %s' %
                (self.cmd_topic, ', '.join(sorted(others))))
            rospy.logerr(self._cmd_conflict)
            return False
        return True

    def _check_backend(self):
        try:
            nodes = set(rosnode.get_node_names())
        except Exception as err:  # noqa: BLE001
            rospy.logwarn('could not query the ROS node list: %s', err)
            return True
        has_chassis = '/chassis' in nodes
        has_gazebo = '/gazebo' in nodes
        if has_chassis and has_gazebo:
            rospy.logfatal('both /chassis and /gazebo are running; '
                           'stop one of them before driving')
            return False
        if self.require_backend == 'real' and has_gazebo:
            rospy.logfatal('require_backend=real but /gazebo is running')
            return False
        if self.require_backend == 'sim' and has_chassis:
            rospy.logfatal('require_backend=sim but /chassis is running')
            return False
        if self.require_backend == 'real' and not has_chassis:
            return True          # may still be starting up
        if self.require_backend == 'sim' and not has_gazebo:
            return True
        return True

    # ------------------------------------------------------------- map check
    def _segment_clear(self, x0, y0, x1, y1):
        """Return (ok, why); the corridor is checked on the static map.

        Distance is measured from each sample point to the edge of the
        nearest occupied/unknown cell rectangle, not by a cell-count radius:
        a cell-count radius overshoots by half a cell and turns borderline
        waypoints near a wall into false positives.
        """
        if not self.segment_check:
            return True, ''
        if self.map_msg is None:
            return True, 'map not available; segment not checked'
        info = self.map_msg.info
        resolution = info.resolution
        if resolution <= 0.0:
            return True, ''
        radius = int(math.ceil(self.segment_half_width / resolution)) + 1
        half_cell = 0.5 * resolution
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-6:
            return True, ''
        step = max(resolution, 0.05)
        samples = max(1, int(length / step))
        # Do not reject a segment because the robot currently sits close to a
        # wall; start checking a little ahead of the current pose.
        skip = min(0.10, length * 0.2)
        for index in range(samples + 1):
            ratio = float(index) / samples
            if ratio * length < skip:
                continue
            x = x0 + (x1 - x0) * ratio
            y = y0 + (y1 - y0) * ratio
            base_x = int(math.floor(
                (x - info.origin.position.x) / resolution))
            base_y = int(math.floor(
                (y - info.origin.position.y) / resolution))
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    mx = base_x + dx
                    my = base_y + dy
                    if not (0 <= mx < info.width and 0 <= my < info.height):
                        return False, 'outside map at (%.2f, %.2f)' % (x, y)
                    value = self.map_msg.data[my * info.width + mx]
                    unknown = value < 0 and self.map_unknown_blocked
                    if not unknown and value < self.map_occupied_threshold:
                        continue
                    center_x = info.origin.position.x + \
                        (mx + 0.5) * resolution
                    center_y = info.origin.position.y + \
                        (my + 0.5) * resolution
                    edge_x = max(0.0, abs(x - center_x) - half_cell)
                    edge_y = max(0.0, abs(y - center_y) - half_cell)
                    if math.hypot(edge_x, edge_y) < self.segment_half_width \
                            - 1e-6:
                        kind = 'unknown' if unknown else 'occupied'
                        return False, '%s map cell near (%.2f, %.2f) ' \
                            '(sample %.2f, %.2f)' % (
                                kind, center_x, center_y, x, y)
        return True, ''

    def _run_check_only(self, waypoints):
        pose = self._lookup_pose()
        if pose is None:
            rospy.logerr('check_only: no %s -> %s transform',
                         self.global_frame, self.base_frame)
            return
        current = {'name': '<current pose>',
                   'x': pose[0], 'y': pose[1]}
        chain = [current] + waypoints
        failures = 0
        for start, goal in zip(chain[:-1], chain[1:]):
            distance = math.hypot(goal['x'] - start['x'],
                                  goal['y'] - start['y'])
            ok, why = self._segment_clear(
                start['x'], start['y'], goal['x'], goal['y'])
            if ok:
                rospy.loginfo('CHECK PASS %-24s -> %-24s %.2f m',
                              start['name'], goal['name'], distance)
            else:
                failures += 1
                rospy.logerr('CHECK FAIL %-24s -> %-24s %.2f m: %s',
                             start['name'], goal['name'], distance, why)
        if failures:
            rospy.logerr('%d segment(s) blocked; add intermediate waypoints '
                         'or move them away from walls', failures)
        else:
            rospy.loginfo('all %d segment(s) clear', len(chain) - 1)

    # -------------------------------------------------------------- commands
    def _send(self, linear, angular):
        msg = Twist()
        if not self.dry_run:
            msg.linear.x = linear
            msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _stop(self):
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self._send(0.0, 0.0)

    def _ramp(self, linear, angular):
        period = 1.0 / self.control_rate
        linear = clamp(linear,
                       self.v_cmd - self.linear_accel * period,
                       self.v_cmd + self.linear_accel * period)
        angular = clamp(angular,
                        self.w_cmd - self.angular_accel * period,
                        self.w_cmd + self.angular_accel * period)
        angular = clamp(angular, -self.w_max, self.w_max)
        self.v_cmd = linear
        self.w_cmd = angular
        self._send(linear, angular)

    def _rotate_command(self, target_yaw, yaw):
        error = wrap_angle(target_yaw - yaw)
        if abs(error) <= self.turn_tol:
            return 0.0, 0.0, error
        speed = clamp(abs(self.k_turn * error),
                      self.turn_w_min, self.turn_w_max)
        return 0.0, math.copysign(speed, error), error

    def _drive_command(self, waypoint, x, y, yaw):
        distance = math.hypot(waypoint['x'] - x, waypoint['y'] - y)
        bearing = math.atan2(waypoint['y'] - y, waypoint['x'] - x)
        error = wrap_angle(bearing - yaw)
        if abs(error) > self.e_reorient:
            speed = clamp(abs(self.k_turn * error),
                          self.turn_w_min, self.turn_w_max)
            return 0.0, math.copysign(speed, error), distance, error
        linear = self.k_d * max(0.0, distance - self.stop_distance)
        if distance > 0.20:
            linear = max(linear, self.v_min)
        linear = min(linear, self.v_max)
        linear *= max(0.35, 1.0 - abs(error) / self.e_reorient)
        angular = clamp(self.k_theta * error, -self.w_max, self.w_max)
        return linear, angular, distance, error

    def _status_update(self, waypoint, phase, distance, error):
        now = time.time()
        self._status = 'waypoint=%s phase=%s distance=%.2f ' \
                       'heading_error=%.2f v=%.2f w=%.2f' % (
                           waypoint['name'], phase, distance, error,
                           self.v_cmd, self.w_cmd)
        if now - self._last_status < 1.0 / max(self.status_rate, 0.1):
            return
        self._last_status = now
        self.status_pub.publish(String(data=self._status))
        if self.dry_run:
            rospy.loginfo('dry-run: %s (would publish v=%.2f w=%.2f)',
                          self._status, self.v_cmd, self.w_cmd)

    # ------------------------------------------------------------------- run
    def _wait_ready(self):
        deadline = time.time() + self.ready_timeout
        while not rospy.is_shutdown():
            if not self._check_backend():
                raise RuntimeError('backend check failed')
            pose = self._lookup_pose()
            scan_ok = (self.scan_msg is not None and
                       time.time() - self.scan_stamp <= self.scan_timeout)
            map_ok = self.map_msg is not None
            if pose is not None and map_ok and (scan_ok or self.check_only):
                break
            if time.time() > deadline:
                raise RuntimeError(
                    'not ready after %.0f s (tf=%s scan=%s map=%s)' % (
                        self.ready_timeout, pose is not None, scan_ok,
                        map_ok))
            self.rate.sleep()

        if self.exclusive_cmd:
            time.sleep(0.2)      # let our publisher register with the master
            if not self._verify_cmd_exclusive():
                raise RuntimeError('another /cmd_vel publisher is running')
        if not self.require_backend == 'any':
            # give the expected backend node a few more seconds to appear
            deadline = time.time() + 5.0
            while not rospy.is_shutdown():
                nodes = set(rosnode.get_node_names())
                expected = '/chassis' if self.require_backend == 'real' \
                    else '/gazebo'
                if expected in nodes:
                    break
                if time.time() > deadline:
                    rospy.logwarn(
                        'expected backend node %s not found; continuing',
                        expected)
                    break
                rospy.sleep(0.2)

        rospy.loginfo('ready: pose, %s, map and lidar are available',
                      self.cmd_topic)
        end = time.time() + self.startup_settle
        while not rospy.is_shutdown() and time.time() < end:
            self._stop()
            self.rate.sleep()

    def _run_dry_run(self, waypoints):
        waypoint = waypoints[0]
        deadline = time.time() + self.dry_run_seconds
        rospy.loginfo('dry-run for %.0f s; no non-zero /cmd_vel is sent',
                      self.dry_run_seconds)
        while not rospy.is_shutdown() and time.time() < deadline:
            state, reason = self._safety_state(time.time())
            pose = self._lookup_pose()
            if pose is None:
                self._stop()
                self.rate.sleep()
                continue
            if state == 'blocked':
                self._stop()
                self._status_update(waypoint, 'BLOCKED', 0.0, 0.0)
                self.rate.sleep()
                continue
            linear, angular, distance, error = self._drive_command(
                waypoint, *pose)
            self.v_cmd = linear
            self.w_cmd = angular
            self._send(linear, angular)
            self._status_update(waypoint, 'DRY_DRIVE', distance, error)
            self.rate.sleep()
        self._stop()

    def _drive_waypoint(self, waypoint):
        self._status = 'waypoint=%s starting' % waypoint['name']
        rospy.loginfo('waypoint %s: (%.2f, %.2f, %.2f rad)',
                      waypoint['name'], waypoint['x'], waypoint['y'],
                      waypoint['yaw'])
        pose = self._lookup_pose()
        if pose is None:
            rospy.logerr('no pose before waypoint %s', waypoint['name'])
            return False
        if self.segment_check and not self.check_only:
            ok, why = self._segment_clear(
                pose[0], pose[1], waypoint['x'], waypoint['y'])
            if not ok:
                rospy.logerr(
                    'straight segment to %s is blocked (%s); add an '
                    'intermediate waypoint', waypoint['name'], why)
                return False

        phase = self.ROTATE
        phase_since = time.time()
        deadline = phase_since
        turn_ticks = 0
        position_ticks = 0
        stall_pose = pose
        stall_since = phase_since
        self._blocked_since = None
        self._block_reason = ''

        while not rospy.is_shutdown():
            now = time.time()
            if self.exclusive_cmd and \
                    now - self._last_cmd_check > self.cmd_check_period:
                self._last_cmd_check = now
                if not self._verify_cmd_exclusive():
                    self._stop()
                    return False

            state, reason = self._safety_state(now)
            if state == 'fatal':
                self._stop()
                rospy.logerr('fatal: %s', reason)
                return False

            pose = self._lookup_pose()
            if pose is None:
                self._stop()
                if self._tf_fail_since is None:
                    self._tf_fail_since = now
                elif now - self._tf_fail_since > self.tf_fail_timeout:
                    if now - self._tf_fail_since > \
                            self.tf_fail_timeout + self.tf_recovery_wait:
                        rospy.logerr(
                            'lost %s -> %s tf for %.1f s; waypoint %s '
                            'failed', self.global_frame, self.base_frame,
                            now - self._tf_fail_since, waypoint['name'])
                        return False
                    self._status_update(waypoint, 'TF_WAIT', 0.0, 0.0)
                self.rate.sleep()
                continue
            self._tf_fail_since = None

            if state == 'blocked':
                self._stop()
                if self._blocked_since is None:
                    self._blocked_since = now
                    self._block_reason = reason
                    rospy.logwarn('holding: %s', reason)
                if now - self._blocked_since > self.obstacle_wait:
                    rospy.logerr('blocked for %.1f s (%s); waypoint %s '
                                 'failed', now - self._blocked_since,
                                 reason, waypoint['name'])
                    return False
                self._status_update(waypoint, 'BLOCKED', 0.0, 0.0)
                self.rate.sleep()
                continue
            if self._blocked_since is not None:
                rospy.loginfo('blocked for %.1f s, resuming waypoint %s '
                              'from ROTATE', now - self._blocked_since,
                              waypoint['name'])
                self._blocked_since = None
                phase = self.ROTATE
                phase_since = now
                turn_ticks = 0
                position_ticks = 0
                stall_pose = pose
                stall_since = now

            x, y, yaw = pose
            distance = math.hypot(waypoint['x'] - x, waypoint['y'] - y)

            if phase == self.ROTATE:
                bearing = math.atan2(waypoint['y'] - y,
                                     waypoint['x'] - x)
                linear, angular, error = self._rotate_command(bearing, yaw)
                if distance <= self.rotate_skip_distance or \
                        abs(error) <= self.turn_tol:
                    turn_ticks += 1
                    self._stop()
                    if turn_ticks >= 3:
                        phase = self.DRIVE_SETTLE
                        phase_since = now
                        deadline = now + self.settle_time
                        turn_ticks = 0
                else:
                    turn_ticks = 0
                    self._ramp(linear, angular)
                if now - phase_since > self.turn_timeout:
                    rospy.logerr('rotate timeout at %s', waypoint['name'])
                    return False
                self._status_update(waypoint, phase, distance, error)

            elif phase == self.DRIVE_SETTLE:
                self._stop()
                if now >= deadline:
                    phase = self.DRIVE
                    phase_since = now
                    stall_pose = pose
                    stall_since = now
                self._status_update(waypoint, phase, distance, 0.0)

            elif phase == self.DRIVE:
                linear, angular, distance, error = self._drive_command(
                    waypoint, x, y, yaw)
                if distance <= self.position_tolerance:
                    position_ticks += 1
                    self._stop()
                    if position_ticks >= 2:
                        phase = self.ALIGN_SETTLE
                        phase_since = now
                        deadline = now + self.settle_time
                        position_ticks = 0
                else:
                    position_ticks = 0
                    self._ramp(linear, angular)
                    moved = math.hypot(x - stall_pose[0],
                                       y - stall_pose[1])
                    if now - stall_since > self.stall_window:
                        if moved < self.stall_distance:
                            rospy.logerr(
                                'no progress for %.1f s at %s; stopping',
                                now - stall_since, waypoint['name'])
                            return False
                        stall_pose = pose
                        stall_since = now
                if now - phase_since > self.drive_timeout:
                    rospy.logerr('drive timeout at %s', waypoint['name'])
                    return False
                self._status_update(waypoint, phase, distance, error)

            elif phase == self.ALIGN_SETTLE:
                self._stop()
                if now >= deadline:
                    if self.align_final_yaw:
                        phase = self.ALIGN_YAW
                        phase_since = now
                        turn_ticks = 0
                    else:
                        phase = self.DONE_SETTLE
                        phase_since = now
                        deadline = now + self.settle_time

            elif phase == self.ALIGN_YAW:
                linear, angular, error = self._rotate_command(
                    waypoint['yaw'], yaw)
                if abs(error) <= self.yaw_tol:
                    turn_ticks += 1
                    self._stop()
                    if turn_ticks >= 3:
                        phase = self.DONE_SETTLE
                        phase_since = now
                        deadline = now + self.settle_time
                        turn_ticks = 0
                else:
                    turn_ticks = 0
                    self._ramp(linear, angular)
                if now - phase_since > self.align_timeout:
                    rospy.logerr('yaw alignment timeout at %s',
                                 waypoint['name'])
                    return False
                self._status_update(waypoint, phase, distance, error)

            elif phase == self.DONE_SETTLE:
                self._stop()
                if now >= deadline:
                    final = self._lookup_pose()
                    if final is not None:
                        position_error = math.hypot(
                            waypoint['x'] - final[0],
                            waypoint['y'] - final[1])
                        yaw_error = abs(wrap_angle(
                            waypoint['yaw'] - final[2]))
                        rospy.loginfo(
                            'reached %s: position error %.3f m, yaw error '
                            '%.3f rad', waypoint['name'], position_error,
                            yaw_error)
                    if self.honor_dwell and waypoint['dwell'] > 0.0:
                        phase = self.DWELL
                        deadline = now + waypoint['dwell']
                    else:
                        return True

            elif phase == self.DWELL:
                self._stop()
                if now >= deadline:
                    return True

            self.rate.sleep()
        return False

    def run(self):
        waypoints = self._select_waypoints(
            self._load_waypoints(self.waypoints_file))
        rospy.loginfo('%d waypoint(s) selected; control rate %.0f Hz',
                      len(waypoints), self.control_rate)
        self._wait_ready()

        if self.check_only:
            self._run_check_only(waypoints)
            return
        if self.dry_run:
            self._run_dry_run(waypoints)
            return

        loops = 0
        while not rospy.is_shutdown():
            for waypoint in waypoints:
                if rospy.is_shutdown():
                    return
                if not self._drive_waypoint(waypoint):
                    if not self.continue_on_failure:
                        rospy.logerr('aborting route at %s',
                                     waypoint['name'])
                        return
            loops += 1
            if not self.loop:
                return
            if self.max_loops and loops >= self.max_loops:
                return

    # -------------------------------------------------------------- shutdown
    def _zero_burst(self):
        period = 1.0 / max(self.zero_burst_rate, 1.0)
        count = int(round(self.zero_burst_sec * self.zero_burst_rate))
        for _ in range(max(1, count)):
            try:
                self._send(0.0, 0.0)
            except Exception:  # noqa: BLE001
                break
            time.sleep(period)
        self.v_cmd = 0.0
        self.w_cmd = 0.0

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        try:
            self._zero_burst()
        except Exception:  # noqa: BLE001
            pass
        if self.disable_chassis_on_exit:
            try:
                from std_srvs.srv import SetBool
                rospy.wait_for_service(
                    self.chassis_enable_service, timeout=1.0)
                rospy.ServiceProxy(
                    self.chassis_enable_service, SetBool)(False)
                rospy.logwarn('chassis disabled on exit')
            except Exception as err:  # noqa: BLE001
                rospy.logwarn('could not disable chassis: %s', err)

    def _on_shutdown(self):
        self.shutdown()


def main():
    rospy.init_node('direct_waypoint_controller')
    controller = DirectWaypointController()
    try:
        controller.run()
    except Exception as err:  # noqa: BLE001
        rospy.logerr('direct controller stopped: %s', err)
        rospy.logerr(traceback.format_exc())
    finally:
        controller.shutdown()
    rospy.signal_shutdown('direct waypoint controller finished')


if __name__ == '__main__':
    main()
