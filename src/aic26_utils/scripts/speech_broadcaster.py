"""Broadcast /speech text through a Bluetooth speaker with Kokoro TTS.

The ROS node deliberately does not import Kokoro: this workspace runs
Python 3.8 (ROS Noetic) while Kokoro needs Python 3.10-3.12. The node keeps
``kokoro_worker.py`` alive in a separate virtual environment and talks to it
with one JSON object per line on stdin/stdout. A newer announcement always
cancels the one being spoken, so the speaker never lags behind the mission.

    sub : /speech (std_msgs/String, UTF-8 text)

Params:
    ~speech_topic     subscribed topic (default /speech)
    ~enabled          master switch (default true)
    ~tts_python       interpreter of the Kokoro venv
                      (default ~/aic26_kokoro_venv/bin/python)
    ~worker_script    kokoro_worker.py path (default: next to this script)
    ~sink             PulseAudio sink, e.g. bluez_output.XX_XX.XX.1
                      (default '', use the system default output)
    ~voice            Kokoro voice (default zf_xiaoxiao)
    ~lang             Kokoro language code, z = Mandarin (default z)
    ~speed            speech rate multiplier (default 1.0)
    ~threads          torch threads, keeps the N100 free for perception
                      (default 2)
    ~player           audio player command (default paplay)
    ~volume           playback volume in percent (default 100)
    ~restart_delay    pause before respawning a crashed worker (default 5 s)

Usage:
    pactl list short sinks | grep bluez
    roslaunch aic26_utils speech.launch sink:=bluez_output.XX_XX.XX.1

If the interpreter or the worker script is missing the node logs an error
and keeps running, so the mission is never taken down by the speaker.
"""

import json
import os
import subprocess
import threading
import time

import rospy
from std_msgs.msg import String

DEFAULT_PYTHON = '~/aic26_kokoro_venv/bin/python'


class SpeechBroadcaster(object):
    """Feeds /speech messages to the Kokoro worker process."""

    def __init__(self):
        self.topic = rospy.get_param('~speech_topic', '/speech')
        self.enabled = bool(rospy.get_param('~enabled', True))
        self.tts_python = os.path.expanduser(
            rospy.get_param('~tts_python', DEFAULT_PYTHON))
        self.worker_script = os.path.expanduser(rospy.get_param(
            '~worker_script',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'kokoro_worker.py')))
        self.sink = rospy.get_param('~sink', '')
        self.voice = rospy.get_param('~voice', 'zf_xiaoxiao')
        self.lang = rospy.get_param('~lang', 'z')
        self.speed = float(rospy.get_param('~speed', 1.0))
        self.threads = int(rospy.get_param('~threads', 2))
        self.player = rospy.get_param('~player', 'paplay')
        self.volume = int(rospy.get_param('~volume', 100))
        self.restart_delay = float(rospy.get_param('~restart_delay', 5.0))

        self.lock = threading.Lock()
        self.worker = None
        self.reader = None
        self.ready = False
        self.pending = None       # newest text received before ready
        self.next_id = 0
        self.worker_gen = 0
        self.last_spawn = 0.0
        self.shutdown = False

        self.check_setup()
        rospy.on_shutdown(self.stop)
        rospy.Subscriber(self.topic, String, self.on_speech, queue_size=1)

        if self.enabled:
            rospy.loginfo("speech broadcaster: %s -> %s%s",
                          self.topic, self.player,
                          ' (sink %s)' % self.sink if self.sink else '')
            # load the model now, not at the first checkpoint
            self.ensure_worker()
        else:
            rospy.logwarn("speech broadcaster disabled by ~enabled")

    # ------------------------------------------------------------ start-up
    def check_setup(self):
        """Disable the node with a clear error when the venv is missing."""
        if not self.enabled:
            return
        problems = []
        if not os.path.isfile(self.tts_python):
            problems.append("interpreter %s" % self.tts_python)
        if not os.path.isfile(self.worker_script):
            problems.append("worker %s" % self.worker_script)
        if problems:
            self.enabled = False
            rospy.logerr("speech broadcaster: missing %s",
                         ', '.join(problems))
            rospy.logerr("run aic26_utils/scripts/setup_kokoro.sh, then "
                         "pass ~tts_python and ~worker_script")

    def worker_cmd(self):
        cmd = [self.tts_python, self.worker_script,
               '--lang', self.lang,
               '--voice', self.voice,
               '--speed', '%.2f' % self.speed,
               '--threads', str(self.threads),
               '--player', self.player,
               '--volume', str(self.volume)]
        if self.sink:
            cmd += ['--sink', self.sink]
        return cmd

    def ensure_worker(self):
        """Return True when a live worker exists, starting one if allowed."""
        with self.lock:
            if self.worker is not None:
                if self.worker.poll() is None:
                    return True
                rospy.logwarn("speech worker exited (rc=%s)",
                              self.worker.returncode)
                self.worker = None
                self.ready = False
                self.pending = None

            if self.shutdown:
                return False
            if time.time() - self.last_spawn < self.restart_delay:
                return False
            self.last_spawn = time.time()

            # A fresh process must not see ROS's Python 3.8 modules through
            # PYTHONPATH; keep the rest of the environment (PulseAudio,
            # HF_HOME, locale) untouched.
            env = dict(os.environ)
            env.pop('PYTHONPATH', None)
            env.pop('PYTHONHOME', None)
            cmd = self.worker_cmd()
            try:
                self.worker = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, env=env, bufsize=1,
                    universal_newlines=True, encoding='utf-8',
                    errors='replace')
            except OSError as err:
                rospy.logerr("speech broadcaster: cannot start worker: %s",
                             err)
                self.worker = None
                return False

            self.ready = False
            self.worker_gen += 1
            gen = self.worker_gen
            self.reader = threading.Thread(
                target=self.read_worker, args=(self.worker, gen),
                name='speech_worker_reader')
            self.reader.daemon = True
            self.reader.start()

        rospy.loginfo("speech worker started (%s)", ' '.join(cmd))
        return True

    # ------------------------------------------------------------ callbacks
    def on_speech(self, msg):
        text = (msg.data or '').strip()
        if not text or not self.enabled or self.shutdown:
            return
        if not self.ensure_worker():
            rospy.logwarn_throttle(
                5.0, "speech worker not ready, dropping: %s", text)
            return
        self.send(text)

    def read_worker(self, proc, gen):
        """Log the worker's status lines until the process exits."""
        try:
            for line in proc.stdout:
                line = line.strip()
                if line:
                    self.handle_worker_line(line, gen)
        except (IOError, ValueError) as err:
            rospy.logwarn("speech reader stopped: %s", err)
        finally:
            with self.lock:
                if gen == self.worker_gen:
                    self.ready = False
                    self.pending = None
            if not self.shutdown:
                rospy.logwarn("speech worker stream closed (rc=%s)",
                              proc.poll())

    def handle_worker_line(self, line, gen):
        try:
            status = json.loads(line)
        except ValueError:
            rospy.loginfo("[kokoro] %s", line)
            return
        if not isinstance(status, dict):
            rospy.loginfo("[kokoro] %s", line)
            return

        state = status.get('status')
        job = status.get('id')
        if state == 'ready':
            with self.lock:
                if gen != self.worker_gen:
                    return
                self.ready = True
                pending = self.pending
                self.pending = None
            rospy.loginfo("speech worker ready")
            if pending:
                self.send(pending)
        elif state == 'error':
            rospy.logwarn("speech worker%s: %s",
                          ' #%s' % job if job else '',
                          status.get('message', 'error'))
        elif state in ('playing', 'done', 'cancelled'):
            rospy.loginfo("speech %s%s", state, ' #%s' % job if job else '')
        else:
            rospy.loginfo("[kokoro] %s", line)

    def send(self, text):
        """Queue ``text`` on the worker; newest wins while it loads."""
        with self.lock:
            worker = self.worker
            if worker is None or worker.poll() is not None:
                return False
            if not self.ready:
                self.pending = text
                return True
            self.next_id += 1
            job = self.next_id

        payload = json.dumps({'id': job, 'text': text}, ensure_ascii=False)
        try:
            worker.stdin.write(payload + '\n')
            worker.stdin.flush()
        except (IOError, ValueError) as err:
            rospy.logwarn("speech broadcaster: worker write failed: %s", err)
            return False
        rospy.loginfo("speak #%d: %s", job, text)
        return True

    # -------------------------------------------------------------- shutdown
    def stop(self):
        self.shutdown = True
        with self.lock:
            worker = self.worker
            self.worker = None
            self.ready = False
            self.pending = None
        if worker is None:
            return
        try:
            if worker.poll() is None and worker.stdin is not None:
                worker.stdin.write('{"cmd": "shutdown"}\n')
                worker.stdin.flush()
                worker.stdin.close()
        except (IOError, ValueError):
            pass
        try:
            worker.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            worker.terminate()
            try:
                worker.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                worker.kill()
        rospy.loginfo("speech worker stopped")


def main():
    rospy.init_node("speech_broadcaster")
    SpeechBroadcaster()
    rospy.spin()


if __name__ == "__main__":
    main()
