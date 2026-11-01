#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture camera photos into <workspace>/photo/.

The AIC26 camera is the UVC node ``aic26_cobot/camera_node.py`` publishing
``/camera/image_raw``.  If nothing is publishing yet, this script launches
``roslaunch aic26_cobot camera.launch`` itself (starting roscore when it is
not running either), waits for a warm-up frame, saves ``--count`` JPEGs and
then shuts down whatever it started.  A camera that was already running is
left untouched.

Run it through ``capture_photos.sh`` so ROS and the workspace are sourced:

    ./capture_photos.sh             # 20 photos, 0.5 s apart
    ./capture_photos.sh 10          # 10 photos
    ./capture_photos.sh -i 1.0      # 20 photos, 1 s apart
    ./capture_photos.sh --help
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import rosgraph
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

WORKSPACE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TOPIC = '/camera/image_raw'
JPEG_QUALITY = 95


def log(message):
    print('[capture] %s' % message, flush=True)


class ImageBuffer(object):
    """Keeps the newest camera message; conversion happens when saving."""

    def __init__(self, topic):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.msg = None
        self.stamp = 0.0
        self.serial = 0
        self.topic = topic

    def on_image(self, msg):
        with self.lock:
            self.msg = msg
            self.stamp = time.time()
            self.serial += 1

    def latest(self, max_age):
        """Return ``(bgr8 frame, serial)``, or None if stale/missing."""
        with self.lock:
            msg, stamp, serial = self.msg, self.stamp, self.serial
        if msg is None or time.time() - stamp > max_age:
            return None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:  # noqa: BLE001 - keep capture alive
            rospy.logwarn_throttle(2.0, 'cannot convert camera frame: %s', err)
            return None
        return frame, serial


# --------------------------------------------------------------- processes
def master_online(timeout=0.0):
    deadline = time.time() + timeout
    while True:
        try:
            if rosgraph.is_master_online():
                return True
        except Exception:  # noqa: BLE001 - master may be mid-restart
            pass
        if time.time() >= deadline:
            return False
        time.sleep(0.5)


def start_process(cmd, logpath):
    """Start cmd in its own process group and log stdout/stderr to logpath."""
    logfile = open(logpath, 'wb')
    try:
        return subprocess.Popen(
            cmd, stdout=logfile, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True,
            # roslaunch block-buffers stdout when it is a file; keep the log
            # readable while the child is still running.
            env=dict(os.environ, PYTHONUNBUFFERED='1'))
    finally:
        logfile.close()


def stop_process(proc, name):
    """Stop a process group politely (SIGINT) and fall back to SIGKILL."""
    if proc is None or proc.poll() is not None:
        return
    log('stopping %s (pid %d)' % (name, proc.pid))
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        log('%s ignored SIGINT, killing it' % name)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5.0)


def tail(path, lines=20):
    try:
        with open(path, 'rb') as f:
            data = f.read().decode('utf-8', 'replace').splitlines()
        return '\n'.join(data[-lines:])
    except OSError:
        return '(no log at %s)' % path


def topic_has_publisher(topic):
    """True when the master lists at least one publisher on topic.

    Goes through rosgraph.Master directly: rospy has no get_publishers()
    and this runs before rospy.init_node().
    """
    try:
        master = rosgraph.Master('/photo_capture_probe')
        publishers, _subscribers, _services = master.getSystemState()
        return any(name == topic and nodes for name, nodes in publishers)
    except Exception:  # noqa: BLE001 - master may be gone
        return False


def wait_for_publisher(topic, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if topic_has_publisher(topic):
            return True
        time.sleep(0.5)
    return False


# --------------------------------------------------------------------- main
def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Save N photos from the AIC26 camera to <workspace>/photo/.')
    parser.add_argument('count_pos', nargs='?', type=int, default=None,
                        metavar='COUNT', help='number of photos (default 20)')
    parser.add_argument('-n', '--count', type=int, default=None,
                        help='number of photos (default 20)')
    parser.add_argument('-i', '--interval', type=float, default=0.5,
                        help='seconds between photos (default 0.5)')
    parser.add_argument('-o', '--outdir', default='photo',
                        help='output folder, relative to the workspace '
                             '(default photo)')
    parser.add_argument('-t', '--topic', default=DEFAULT_TOPIC,
                        help='image topic (default %s)' % DEFAULT_TOPIC)
    parser.add_argument('--device', default='',
                        help='camera device for camera.launch, e.g. /dev/video2')
    parser.add_argument('--width', type=int, default=0,
                        help='capture width override for camera.launch')
    parser.add_argument('--height', type=int, default=0,
                        help='capture height override for camera.launch')
    parser.add_argument('--fps', type=int, default=0,
                        help='capture fps override for camera.launch')
    parser.add_argument('--warmup', type=float, default=1.5,
                        help='seconds to drop after the first frame so auto '
                             'exposure settles (default 1.5)')
    parser.add_argument('--start-timeout', type=float, default=20.0,
                        help='seconds to wait for roscore/camera '
                             '(default 20)')
    args = parser.parse_args(argv)

    if args.count is None:
        args.count = args.count_pos if args.count_pos is not None else 20
    if args.count <= 0:
        parser.error('count must be positive')
    if args.interval < 0:
        parser.error('interval must be >= 0')
    return args


def main(argv=None):
    args = parse_args(argv)

    outdir = args.outdir
    if not os.path.isabs(outdir):
        outdir = os.path.join(WORKSPACE, outdir)
    os.makedirs(outdir, exist_ok=True)
    log('workspace: %s' % WORKSPACE)
    log('saving %d photos to %s' % (args.count, outdir))

    started_master = None
    started_camera = None
    camera_log = None
    tmpdir = tempfile.mkdtemp(prefix='capture_photos_')
    try:
        if not master_online():
            log('ROS master is not running, starting roscore...')
            logpath = os.path.join(tmpdir, 'roscore.log')
            started_master = start_process(['roscore'], logpath)
            if not master_online(timeout=args.start_timeout):
                log('roscore did not come up:\n%s' % tail(logpath))
                return 1
            log('roscore is up')

        if wait_for_publisher(args.topic, 1.0):
            log('camera already publishing %s' % args.topic)
        else:
            cmd = ['roslaunch', 'aic26_cobot', 'camera.launch']
            for name, value in (('device', args.device),
                                ('width', args.width),
                                ('height', args.height),
                                ('fps', args.fps)):
                if value:
                    cmd.append('%s:=%s' % (name, value))
            log('camera not running, launching: %s' % ' '.join(cmd))
            camera_log = os.path.join(tmpdir, 'camera.log')
            started_camera = start_process(cmd, camera_log)
            if not wait_for_publisher(args.topic, args.start_timeout):
                log('camera did not publish %s within %.0f s:\n%s'
                    % (args.topic, args.start_timeout, tail(camera_log)))
                return 1
            log('camera is up')

        rospy.init_node('photo_capture', anonymous=True, disable_signals=True)
        buffer = ImageBuffer(args.topic)
        rospy.Subscriber(args.topic, Image, buffer.on_image, queue_size=1)

        deadline = time.time() + args.start_timeout
        while buffer.latest(1.0) is None:
            if started_camera is not None and started_camera.poll() is not None:
                log('camera node exited unexpectedly:\n%s'
                    % tail(camera_log or ''))
                return 1
            if time.time() > deadline:
                log('no image received on %s' % args.topic)
                return 1
            time.sleep(0.1)
        log('first frame received, warming up %.1f s' % args.warmup)
        time.sleep(args.warmup)

        session = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        saved = 0
        index = 0
        last_serial = None
        last_wait_log = 0.0
        while saved < args.count:
            got = buffer.latest(max(1.0, args.interval * 2.0))
            if got is None or got[1] == last_serial:
                now = time.time()
                if now - last_wait_log > 2.0:
                    log('waiting for a fresh camera frame...')
                    last_wait_log = now
                if started_camera is not None and \
                        started_camera.poll() is not None:
                    log('camera node exited unexpectedly:\n%s'
                        % tail(camera_log or ''))
                    return 1
                time.sleep(min(0.05, args.interval or 0.05))
                continue

            frame, last_serial = got
            index += 1
            path = os.path.join(
                outdir, 'photo_%s_%02d.jpg' % (session, index))
            ok = cv2.imwrite(
                path, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok or not os.path.getsize(path):
                log('failed to save %s' % path)
                if os.path.exists(path):
                    os.remove(path)
                index -= 1
            else:
                saved += 1
                log('saved %3d/%d  %s' % (saved, args.count, path))
            if saved < args.count:
                time.sleep(args.interval)

        log('done: %d photos in %s' % (saved, outdir))
        return 0
    except KeyboardInterrupt:
        log('interrupted')
        return 1
    finally:
        try:
            rospy.signal_shutdown('capture finished')
        except Exception:  # noqa: BLE001 - rospy may not be initialised
            pass
        stop_process(started_camera, 'camera')
        stop_process(started_master, 'roscore')


if __name__ == '__main__':
    sys.exit(main())
