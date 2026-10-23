"""Kokoro TTS worker for speech_broadcaster.py.

This script runs in its own Python 3.10-3.12 virtual environment, because the
ROS Noetic node runs Python 3.8 and Kokoro does not support it. It imports no
ROS package: the parent talks to it with one JSON object per line on stdin
and reads JSON status lines on stdout.

Commands (stdin):
    {"id": 1, "text": "本区域共检测到 12 人。"}
    {"cmd": "shutdown"}

Status (stdout):
    {"status": "ready"}
    {"id": 1, "status": "playing"}
    {"id": 1, "status": "done"}
    {"id": 1, "status": "cancelled"}
    {"id": 1, "status": "error", "message": "..."}

A new command cancels the utterance being synthesized or played, so the
speaker follows the latest announcement. Audio is generated at 24 kHz and
played with paplay through ``--sink`` (a PulseAudio sink, e.g. the Bluetooth
speaker), one temporary wav per text chunk.

Usage:
    # installation check: synthesize one sentence, optionally save it
    ~/aic26_kokoro_venv/bin/python kokoro_worker.py --check \
        --voice zf_xiaoxiao --sink bluez_output.XX_XX.XX.1
    ~/aic26_kokoro_venv/bin/python kokoro_worker.py --check \
        --out /tmp/kokoro_check.wav

    # normally started by speech_broadcaster.py, not by hand
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import tempfile
import threading

SAMPLE_RATE = 24000
CHECK_TEXT = '语音播报测试'


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Kokoro TTS worker (JSON lines on stdin/stdout)')
    parser.add_argument('--lang', default='z',
                        help="Kokoro language code (z = Mandarin)")
    parser.add_argument('--voice', default='zf_xiaoxiao',
                        help='Kokoro voice name')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='speech rate multiplier')
    parser.add_argument('--sink', default='',
                        help='PulseAudio sink name (empty = default output)')
    parser.add_argument('--player', default='paplay',
                        help='audio player command')
    parser.add_argument('--volume', type=int, default=100,
                        help='playback volume in percent (paplay)')
    parser.add_argument('--threads', type=int, default=2,
                        help='torch CPU threads')
    parser.add_argument('--nice', type=int, default=10,
                        help='process nice increment (Linux)')
    parser.add_argument('--sample-rate', type=int, default=SAMPLE_RATE,
                        help='Kokoro output sample rate')
    parser.add_argument('--check', action='store_true',
                        help='synthesize one sentence and exit')
    parser.add_argument('--out', default='',
                        help='with --check, save the wav instead of playing')
    parser.add_argument('--text', default=CHECK_TEXT,
                        help='sentence used by --check')
    return parser.parse_args(argv)


def emit(**fields):
    """Write one JSON status line to the parent."""
    sys.stdout.write(json.dumps(fields, ensure_ascii=False) + '\n')
    sys.stdout.flush()


def log(text):
    """Human-readable progress goes to stderr; the parent logs it as-is."""
    sys.stderr.write(text + '\n')
    sys.stderr.flush()


def player_cmd(args):
    """Build the player command for the configured sink and volume."""
    cmd = shlex.split(args.player)
    if args.sink:
        cmd.append('--device=%s' % args.sink)
    volume = max(0, min(100, int(args.volume)))
    cmd.append('--volume=%d' % int(volume * 65536 / 100.0))
    return cmd


class Worker(object):
    """Read jobs from stdin and play them, newest job wins."""

    def __init__(self, pipeline, args):
        self.pipeline = pipeline
        self.args = args
        self.lock = threading.Lock()
        self.wake = threading.Condition(self.lock)
        self.player_lock = threading.Lock()
        self.cancel = threading.Event()
        self.job = None           # (id, text) waiting to be spoken
        self.player = None        # running paplay process
        self.stopping = False

    # ----------------------------------------------------------------- input
    def read_stdin(self):
        try:
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue
                self.handle_command(line)
                if self.stopping:
                    break
        except (IOError, ValueError) as err:
            log('stdin closed: %s' % err)
        finally:
            self.request_stop()

    def handle_command(self, line):
        try:
            cmd = json.loads(line)
        except ValueError:
            emit(status='error', message='bad json: %s' % line)
            return
        if not isinstance(cmd, dict):
            emit(status='error', message='bad command')
            return

        if cmd.get('cmd') == 'shutdown':
            self.request_stop()
            return

        text = str(cmd.get('text') or '').strip()
        if not text:
            return
        with self.wake:
            self.cancel.set()
            self.job = (cmd.get('id'), text)
            self.wake.notify_all()
        # stop the old wav playout outside the job lock
        self.kill_player()

    def request_stop(self):
        with self.wake:
            self.stopping = True
            self.cancel.set()
            self.job = None
            self.wake.notify_all()
        self.kill_player()

    # ------------------------------------------------------------------ loop
    def run(self):
        reader = threading.Thread(target=self.read_stdin, name='stdin_reader')
        reader.daemon = True
        reader.start()
        emit(status='ready')
        log('kokoro worker ready')

        while True:
            with self.wake:
                while self.job is None and not self.stopping:
                    self.wake.wait(0.5)
                if self.stopping:
                    return
                job_id, text = self.job
                self.job = None
                self.cancel.clear()
            self.speak(job_id, text)

    def speak(self, job_id, text):
        emit(id=job_id, status='playing')
        try:
            for _gs, _ps, audio in self.pipeline(
                    text, voice=self.args.voice, speed=self.args.speed):
                if self.cancel.is_set():
                    emit(id=job_id, status='cancelled')
                    return
                samples = self.to_samples(audio)
                if samples.size == 0:
                    continue
                self.play(samples)
                if self.cancel.is_set():
                    emit(id=job_id, status='cancelled')
                    return
            emit(id=job_id, status='done')
        except Exception as err:                       # noqa: BLE001
            emit(id=job_id, status='error', message=str(err))

    @staticmethod
    def to_samples(audio):
        import numpy as np
        return np.asarray(audio, dtype=np.float32).reshape(-1)

    def play(self, samples):
        """Write one chunk to a temp wav and play it through paplay."""
        import soundfile as sf

        path = None
        try:
            fd, path = tempfile.mkstemp(prefix='aic26_tts_', suffix='.wav')
            os.close(fd)
            sf.write(path, samples, self.args.sample_rate)
            if self.cancel.is_set():
                return

            cmd = player_cmd(self.args) + [path]
            with self.player_lock:
                if self.cancel.is_set():
                    return
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                    universal_newlines=True)
                self.player = proc

            _out, err = proc.communicate()
            with self.player_lock:
                if self.player is proc:
                    self.player = None

            if (proc.returncode not in (0, -15) and
                    not self.cancel.is_set()):
                emit(status='error', message='%s: %s' % (
                    self.args.player, (err or '').strip()))
        finally:
            with self.player_lock:
                self.player = None
            if path:
                try:
                    os.unlink(path)
                except OSError:
                    pass

    def kill_player(self):
        """Terminate the wav being played; called when a new job arrives."""
        with self.player_lock:
            proc = self.player
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except OSError:
                pass


def build_pipeline(args):
    """Load Kokoro once; torch is limited to a few threads on the N100."""
    os.environ.setdefault('OMP_NUM_THREADS', str(max(1, args.threads)))
    import torch
    torch.set_num_threads(max(1, args.threads))
    from kokoro import KPipeline

    log('loading Kokoro (lang=%s, voice=%s, threads=%d)...' % (
        args.lang, args.voice, max(1, args.threads)))
    return KPipeline(lang_code=args.lang)


def run_check(pipeline, args):
    """Synthesize CHECK_TEXT once, for the installation script."""
    import numpy as np
    import soundfile as sf

    text = args.text or CHECK_TEXT
    try:
        chunks = [Worker.to_samples(audio)
                  for _gs, _ps, audio in pipeline(
                      text, voice=args.voice, speed=args.speed)]
        audio = np.concatenate(chunks) if chunks else np.zeros(
            0, dtype=np.float32)
    except Exception as err:                           # noqa: BLE001
        log('kokoro check failed: %s' % err)
        return 2

    if audio.size == 0:
        log('kokoro produced no audio')
        return 2

    if args.out:
        sf.write(args.out, audio, args.sample_rate)
        log('wrote %s' % args.out)
        return 0

    fd, path = tempfile.mkstemp(prefix='aic26_tts_check_', suffix='.wav')
    os.close(fd)
    try:
        sf.write(path, audio, args.sample_rate)
        cmd = player_cmd(args) + [path]
        log('playing: %s' % ' '.join(cmd))
        code = subprocess.call(cmd)
        if code != 0:
            log('player failed (rc=%d); use --out FILE to save instead' %
                code)
        return 0 if code == 0 else 1
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def main():
    if sys.version_info < (3, 10):
        log('kokoro_worker.py needs Python 3.10-3.12; run it with the venv '
            'interpreter (see setup_kokoro.sh)')
        return 2

    if hasattr(sys.stdin, 'reconfigure'):
        sys.stdin.reconfigure(encoding='utf-8')
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')

    args = parse_args()
    try:
        os.nice(max(0, args.nice))
    except (AttributeError, OSError):
        pass

    try:
        pipeline = build_pipeline(args)
    except Exception as err:                           # noqa: BLE001
        emit(status='error', message='kokoro init failed: %s' % err)
        log('hint: run setup_kokoro.sh (kokoro, soundfile, misaki[zh], '
            'espeak-ng)')
        return 2

    if args.check:
        return run_check(pipeline, args)

    Worker(pipeline, args).run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
