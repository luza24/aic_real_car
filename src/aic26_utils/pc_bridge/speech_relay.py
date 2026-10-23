"""Temporary car-side relay: /speech -> JSON lines on stdout.

Run on the car (ROS Noetic, Python 3.8). It is copied to /tmp by
speech_bridge.py and never added to the car workspace.

    sub : /speech (std_msgs/String)

Every protocol line is ASCII JSON with a fixed prefix so the Windows side can
ignore rospy's own log lines:

    AIC26_SPEECH {"v": 1, "type": "boot", "pid": 123}
    AIC26_SPEECH {"v": 1, "type": "ready", "topic": "/speech", "node": "..."}
    AIC26_SPEECH {"v": 1, "type": "speech", "seq": 1, "text": "本..."}
    AIC26_SPEECH {"v": 1, "type": "shutdown", "reason": "stdin-eof"}

When stdin is a pipe (the usual case), a watchdog exits the node as soon as
the SSH peer closes it, so a dead bridge cannot leave an orphan ROS node.
"""

import argparse
import json
import os
import sys
import threading

PREFIX = 'AIC26_SPEECH '


def emit(kind, **fields):
    """Write one protocol line; never let a closed pipe kill the node."""
    payload = {'v': 1, 'type': kind}
    payload.update(fields)
    try:
        sys.stdout.write(PREFIX + json.dumps(payload, ensure_ascii=True) + '\n')
        sys.stdout.flush()
    except (IOError, OSError, ValueError):
        pass


class Relay(object):
    def __init__(self, topic):
        self.topic = topic
        self.seq = 0

    def on_speech(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return
        self.seq += 1
        emit('speech', seq=self.seq, text=text)


def stdin_watchdog():
    """Exit when the SSH peer closes stdin (best effort).

    Input is ignored; only EOF (a closed pipe) stops the node.
    """
    try:
        while sys.stdin.buffer.read(1):
            pass
    except Exception:                                   # noqa: BLE001
        pass
    emit('shutdown', reason='stdin-eof')
    try:
        import rospy
        if rospy.core.is_initialized():
            rospy.signal_shutdown('stdin-eof')
            return
    except Exception:                                   # noqa: BLE001
        pass
    os._exit(0)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Relay /speech as JSON lines for the PC bridge')
    parser.add_argument('--topic', default='/speech',
                        help='topic to subscribe (default /speech)')
    args = parser.parse_args(argv)

    emit('boot', pid=os.getpid())

    # Only watch a pipe. On an interactive terminal the watchdog would stop
    # the node as soon as the user presses a key.
    if not sys.stdin.isatty():
        watchdog = threading.Thread(target=stdin_watchdog, name='stdin_watchdog')
        watchdog.daemon = True
        watchdog.start()

    import rospy
    from std_msgs.msg import String

    rospy.init_node('aic26_speech_relay', anonymous=True)
    relay = Relay(args.topic)
    rospy.Subscriber(args.topic, String, relay.on_speech, queue_size=10)
    emit('ready', topic=args.topic, node=rospy.get_name())
    rospy.spin()
    emit('shutdown', reason='ros-spin-ended')
    return 0


if __name__ == '__main__':
    sys.exit(main())
