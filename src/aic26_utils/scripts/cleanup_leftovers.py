"""Free the machine from processes left over by an earlier run.

    rosrun aic26_utils cleanup_leftovers.py [--dry-run] [--quiet]
    rosrun aic26_utils cleanup_leftovers.py --kill-cpu 60

Three passes:

1. processes matching ``PATTERNS`` that belong to another ROS master, or
   that are older than the roslaunch this node was started by (a stack from
   a previous run still holding /dev/carserial, /dev/lslidar or the GPU),
2. ROS nodes of the current master with a roslaunch duplicate name
   (``chassis_12345_67890123``),
3. a report of the ten busiest processes, and optionally a kill of every
   unprotected process above ``--kill-cpu`` percent.

Nothing started by the current roslaunch is touched: a process is only
killed when its master port differs, or when it started before the
roslaunch that owns this node.
"""

import argparse
import os
import re
import signal
import socket
import subprocess

PATTERNS = (
    'chassis_node.py', 'lidar_node.py', 'imu_node.py', 'camera_node.py',
    'hardware_status_node.py', 'amcl', 'move_base', 'map_server',
    'localisation_check.py', 'virtual_walls.py', 'robot_state_publisher',
    'camera_view.py', 'capture_cycle.py', 'demo_cycle.py',
    'mission_controller.py', 'waypoint_follower.py', 'stop_robot.py',
    'rviz', 'gzserver', 'gzclient', 'gazebo', 'rqt_', 'image_view',
)

PROTECTED = (
    'cleanup_leftovers.py', 'vscode-server', 'code-server', 'sshd', 'systemd',
    'Xorg', 'dbus', 'rosmaster', 'grep', 'ps -eo',
)

LAUNCHERS = ('roslaunch', 'rosrun')

DUPLICATE = re.compile(r'^(.*)_\d+_\d+$')

LOCAL_HOSTS = ('localhost', '127.0.0.1', 'localhost.localdomain')


def read_cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as handle:
            return handle.read().replace(b'\0', b' ').decode('utf-8', 'replace')
    except (IOError, OSError):
        return ''


def read_environ(pid, key):
    try:
        with open('/proc/%d/environ' % pid, 'rb') as handle:
            for entry in handle.read().split(b'\0'):
                if entry.startswith(key.encode() + b'='):
                    return entry.split(b'=', 1)[1].decode('utf-8', 'replace')
    except (IOError, OSError):
        pass
    return ''


def read_parent(pid):
    try:
        with open('/proc/%d/status' % pid) as handle:
            for line in handle:
                if line.startswith('PPid:'):
                    return int(line.split()[1])
    except (IOError, OSError):
        pass
    return 0


def read_start(pid):
    try:
        with open('/proc/%d/stat' % pid) as handle:
            fields = handle.read().rsplit(')', 1)[1].split()
        return int(fields[19])
    except (IOError, OSError, IndexError):
        return None


def split_uri(uri):
    uri = (uri or '').strip()
    if '://' in uri:
        uri = uri.split('://', 1)[1]
    host, _, port = uri.rstrip('/').partition(':')
    return host.lower(), port


def same_master(uri, master):
    host, port = split_uri(uri)
    ref_host, ref_port = split_uri(master)
    if not port or port != ref_port:
        return False
    if host == ref_host:
        return True
    names = LOCAL_HOSTS + (socket.gethostname().lower(),)
    return host in names and ref_host in names


def launcher_start():
    pid = os.getppid()
    for _ in range(8):
        if pid <= 1:
            return None
        cmdline = read_cmdline(pid)
        if any(name in cmdline for name in LAUNCHERS):
            return read_start(pid)
        pid = read_parent(pid)
    return None


def matches(cmdline, patterns):
    return any(pattern in cmdline for pattern in patterns)


def leftovers(master, started, dry_run):
    killed = []
    current = os.getpid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == current:
            continue
        cmdline = read_cmdline(pid)
        if not cmdline or matches(cmdline, PROTECTED):
            continue
        if not matches(cmdline, PATTERNS):
            continue
        uri = read_environ(pid, 'ROS_MASTER_URI')
        if not uri:
            continue
        if same_master(uri, master):
            begin = read_start(pid)
            if started is None or begin is None or begin >= started:
                continue
        killed.append((pid, cmdline.strip()))
        if not dry_run:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
    return killed


def duplicate_nodes(dry_run):
    try:
        import rosnode
        names = rosnode.get_node_names()
    except Exception:
        return []
    stale = [name for name in names if DUPLICATE.match(name)]
    if stale and not dry_run:
        try:
            rosnode.kill_nodes(stale)
        except Exception:
            pass
    return stale


def kill_cpu(threshold, dry_run):
    killed = []
    if threshold <= 0.0:
        return killed
    output = subprocess.check_output(
        ['ps', '-eo', 'pid,pcpu,args', '--sort=-pcpu']).decode(
            'utf-8', 'replace')
    for line in output.splitlines()[1:]:
        fields = line.split(None, 2)
        if len(fields) < 3:
            continue
        pid, cpu, args = int(fields[0]), float(fields[1]), fields[2]
        if pid == os.getpid() or cpu < threshold:
            continue
        if matches(args, PROTECTED) or matches(args, PATTERNS):
            continue
        killed.append((pid, args.strip()))
        if not dry_run:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
    return killed


def report(limit=10):
    output = subprocess.check_output(
        ['ps', '-eo', 'pid,pcpu,pmem,args', '--sort=-pcpu']).decode(
            'utf-8', 'replace')
    print('busiest processes:')
    for line in output.splitlines()[:limit + 1]:
        print('  ' + line[:160])


def main():
    parser = argparse.ArgumentParser(description='kill leftover aic26 runs')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--kill-cpu', type=float, default=0.0,
                        help='kill unprotected processes above this %% CPU')
    args, _ = parser.parse_known_args()

    master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    started = launcher_start()
    stale = leftovers(master, started, args.dry_run)
    for pid, cmdline in stale:
        print('%s %d: %s' % ('would kill' if args.dry_run else 'killed',
                             pid, cmdline[:120]))
    duplicates = duplicate_nodes(args.dry_run)
    for name in duplicates:
        print('%s duplicate node %s'
              % ('would kill' if args.dry_run else 'killed', name))
    extra = kill_cpu(args.kill_cpu, args.dry_run)
    for pid, cmdline in extra:
        print('%s %d (cpu): %s' % ('would kill' if args.dry_run else 'killed',
                                   pid, cmdline[:120]))

    if not stale and not duplicates and not extra:
        print('cleanup: nothing to do')
    if not args.quiet:
        report()


if __name__ == '__main__':
    main()
