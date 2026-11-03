"""Stop the car and every node that sends it motion commands.

One shot, usable at any time while the stack is running:

    rosrun aic26_utils stop_robot.py

The tool cancels the active move_base goal, streams zero ``/cmd_vel`` for
``--hold`` seconds, disables the real chassis motors through
``/chassis/enable`` and stops the nodes publishing ``/cmd_vel`` or
``/move_base/goal`` (move_base, mission/waypoint/demo/capture cycles,
teleop). The zero stream keeps running while those nodes exit, so no last
velocity command stays latched in the chassis.

Args:
    --cmd-topic   command topic (default /cmd_vel)
    --hold        seconds to keep publishing zeros (default 3.0)
    --rate        zero publish rate in Hz (default 20.0)
    --no-kill     only cancel the goal and publish zeros
    --keep NODE   do not stop NODE (repeatable, e.g. --keep move_base)
    --no-motors   leave /chassis/enable untouched
    --serial      when the ROS master is unreachable, write the zero frames
                  to the STM32 UART with aic26_cobot/emergency_stop.py
    --port, --baud  serial fallback settings (default /dev/carserial, 115200)

After a stop that disabled the motors, re-enable the chassis with
``rosservice call /chassis/enable "data: true"`` or by restarting it.
"""

import argparse
import os
import subprocess
import sys
import time

import rosgraph
import rosnode
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

MOVE_BASE_GOAL = '/move_base/goal'
MOVE_BASE_CANCEL = '/move_base/cancel'
CHASSIS_ENABLE = '/chassis/enable'

# These nodes command the car only between waypoints, so a snapshot of the
# /cmd_vel publishers can miss them. Direct publishers are found dynamically.
KNOWN_SENDERS = ('move_base', 'mission_controller', 'waypoint_follower',
                 'demo_cycle', 'capture_cycle', 'teleop')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Stop the car and the nodes that command it.')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--hold', type=float, default=3.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--no-kill', action='store_true')
    parser.add_argument('--keep', action='append', default=[], metavar='NODE')
    parser.add_argument('--no-motors', action='store_true')
    parser.add_argument('--serial', action='store_true')
    parser.add_argument('--port', default='/dev/carserial')
    parser.add_argument('--baud', type=int, default=115200)
    return parser.parse_args(argv)


def master_pid():
    """Return the master PID, or None when no master answers."""
    try:
        return rosgraph.Master('/stop_robot').getPid()
    except Exception:                                  # noqa: BLE001
        return None


def master_state():
    """Return ``[publishers, subscribers, services]`` or None."""
    try:
        return rosgraph.Master('/stop_robot').getSystemState()
    except Exception:                                  # noqa: BLE001
        return None


def node_names(state):
    """All registered node names, from the master system state."""
    names = set()
    if state is not None:
        for group in state:
            for _, nodes in group:
                names.update(nodes)
    return names


def select_targets(state, cmd_topic, keep, self_name):
    """Nodes to stop: command publishers plus the known senders."""
    wanted = set([cmd_topic, MOVE_BASE_GOAL])
    targets = set()
    if state is not None:
        for topic, publishers in state[0]:
            if topic in wanted:
                targets.update(publishers)
    for name in node_names(state):
        if name.rsplit('/', 1)[-1] in KNOWN_SENDERS:
            targets.add(name)

    spared = set(keep) | set(n.rsplit('/', 1)[-1] for n in keep)
    return set(name for name in targets
               if name != self_name and name.rsplit('/', 1)[-1] not in spared)


def disable_motors():
    """Latch the chassis motors off through /chassis/enable."""
    try:
        rospy.wait_for_service(CHASSIS_ENABLE, timeout=1.0)
    except rospy.ROSException:
        rospy.loginfo('stop: no %s (simulation?), motors left enabled',
                      CHASSIS_ENABLE)
        return False
    try:
        rospy.ServiceProxy(CHASSIS_ENABLE, SetBool)(False)
    except rospy.ServiceException as err:
        rospy.logwarn('stop: %s failed: %s', CHASSIS_ENABLE, err)
        return False
    rospy.loginfo('stop: chassis motors disabled')
    return True


def stop_nodes(targets):
    """Send the shutdown request to ``targets`` (best effort)."""
    try:
        stopped, failed = rosnode.kill_nodes(sorted(targets))
    except Exception as err:                           # noqa: BLE001
        rospy.logwarn('stop: cannot reach the master: %s', err)
        return
    if failed:
        rospy.logwarn('stop: could not stop %s', ', '.join(sorted(failed)))
    rospy.loginfo('stop: stopped %d sender(s)', len(stopped))


def run(args):
    rospy.init_node('stop_robot', anonymous=True)

    zero = Twist()
    cmd_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=10)
    cancel_pub = rospy.Publisher(MOVE_BASE_CANCEL, GoalID, queue_size=1)

    targets = select_targets(master_state(), args.cmd_topic, args.keep,
                             rospy.get_name())
    time.sleep(0.2)                # let the publishers register

    # A zero GoalID cancels every goal accepted by the move_base server.
    for _ in range(3):
        cancel_pub.publish(GoalID())
        cmd_pub.publish(zero)
        time.sleep(0.05)

    if not args.no_motors:
        disable_motors()

    if args.no_kill:
        rospy.loginfo('stop: --no-kill, keep %s',
                      ', '.join(sorted(targets)) or 'no sender')
    elif targets:
        rospy.loginfo('stop: stopping %s', ', '.join(sorted(targets)))
        stop_nodes(targets)
    else:
        rospy.loginfo('stop: no sender node found')

    hold = max(0.5, args.hold)
    period = 1.0 / max(1.0, args.rate)
    deadline = time.time() + hold
    while not rospy.is_shutdown() and time.time() < deadline:
        cmd_pub.publish(zero)
        time.sleep(period)
    for _ in range(5):
        cmd_pub.publish(zero)
        time.sleep(0.05)

    rospy.loginfo('stop: zero %s for %.1fs', args.cmd_topic, hold)
    if not args.no_motors:
        rospy.loginfo('stop: re-enable the chassis with '
                      'rosservice call %s "data: true"', CHASSIS_ENABLE)


def emergency_stop_script():
    """Path of aic26_cobot/emergency_stop.py in the install or source tree."""
    try:
        import rospkg
        return os.path.join(rospkg.RosPack().get_path('aic26_cobot'),
                            'scripts', 'emergency_stop.py')
    except Exception:                                  # noqa: BLE001
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(
            here, os.pardir, os.pardir, 'aic26_cobot', 'scripts',
            'emergency_stop.py'))


def serial_stop(args):
    """Write zero frames straight to the UART when ROS is down."""
    script = emergency_stop_script()
    if not os.path.isfile(script):
        print('cannot find aic26_cobot/emergency_stop.py (looked at %s)'
              % script)
        return 2

    print('ROS master unreachable, writing zero frames to %s' % args.port)
    return subprocess.call([sys.executable, script,
                            '--port', args.port,
                            '--baud', str(args.baud),
                            '--seconds', str(max(0.5, args.hold))])


def main(argv):
    args = parse_args(argv)
    if master_pid() is None:
        if args.serial:
            return serial_stop(args)
        print('ROS master unreachable: start it, or use --serial to send '
              'zero frames to %s directly.' % args.port)
        return 2
    run(args)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main(rospy.myargv(argv=sys.argv)[1:]))
    except rospy.ROSInterruptException:
        pass
