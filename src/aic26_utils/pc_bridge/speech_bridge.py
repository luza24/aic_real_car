"""Windows-side bridge: car /speech -> Kokoro TTS -> PC speakers.

The PC has no ROS. This tool copies speech_relay.py to the car's /tmp and
runs it over SSH; the relay prints one JSON line per /speech message. Kokoro
then synthesizes Chinese and sounddevice plays it on the Windows default
output device. A new message immediately stops the current one.

Modes:
    python speech_bridge.py --once "语音播报测试"
    python speech_bridge.py --demo
    python speech_bridge.py --ssh-host vehicle

The first synthesis downloads the Kokoro model. Run this file with the
Python 3.11 conda environment (Kokoro does not support 3.8/3.13+).
"""

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import threading
import time

SAMPLE_RATE = 24000
PREFIX = 'AIC26_SPEECH '
DEMO_TEXTS = [
    '本区域共检测到 12 人，其中 2 人为非社区人员。',
    '本区域共检测到 8 人，未发现非社区人员。',
    '检测到车辆，车牌号为 苏AB8Q62。',
    '检测到车辆，但未能识别车牌。',
]


def log(text):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), text), flush=True)


def resample_linear(samples, src_rate, dst_rate):
    """Small linear resampler used when the output device rejects 24 kHz."""
    import numpy as np
    if src_rate == dst_rate or samples.size == 0:
        return samples
    count = max(1, int(round(samples.size * float(dst_rate) / src_rate)))
    x_new = np.linspace(0.0, samples.size - 1.0, count)
    x_old = np.arange(samples.size)
    return np.interp(x_new, x_old, samples).astype(np.float32)


class KokoroPlayer(object):
    """Loads Kokoro once and plays the newest submitted text."""

    def __init__(self, lang='z', voice='zf_xiaoxiao', speed=1.0,
                 threads=4, device=None):
        self.lang = lang
        self.voice = voice
        self.speed = float(speed)
        self.threads = max(1, int(threads))
        self.device = device
        self.lock = threading.Lock()
        self.wake = threading.Condition(self.lock)
        self.pending = None
        self.generation = 0
        self.busy = False
        self.stopping = False
        self.ready = threading.Event()
        self.failed = threading.Event()
        self.error = None
        self.pipeline = None
        self.thread = None

    # ------------------------------------------------------------- lifecycle
    def start(self):
        self.thread = threading.Thread(target=self._run, name='kokoro',
                                       daemon=True)
        self.thread.start()

    def wait_ready(self):
        while not self.ready.is_set():
            if self.failed.is_set():
                raise RuntimeError(self.error)
            time.sleep(0.1)

    def _run(self):
        try:
            os.environ.setdefault('OMP_NUM_THREADS', str(self.threads))
            import torch
            torch.set_num_threads(self.threads)
            from kokoro import KPipeline
            log('loading Kokoro (lang=%s, voice=%s, threads=%d)...'
                % (self.lang, self.voice, self.threads))
            self.pipeline = KPipeline(lang_code=self.lang)
        except Exception as err:                         # noqa: BLE001
            self.error = str(err)
            self.failed.set()
            return
        self.ready.set()
        log('Kokoro ready')

        while True:
            with self.wake:
                while self.pending is None and not self.stopping:
                    self.wake.wait(0.5)
                if self.stopping:
                    return
                text = self.pending
                self.pending = None
                generation = self.generation
                self.busy = True
            try:
                self._speak(text, generation)
            except Exception as err:                     # noqa: BLE001
                log('speak failed: %s' % err)
            finally:
                with self.wake:
                    self.busy = False

    # ---------------------------------------------------------------- input
    def submit(self, text):
        """Queue text; a newer call cancels the utterance in progress."""
        text = (text or '').strip()
        if not text:
            return
        with self.wake:
            self.pending = text
            self.generation += 1
            self.wake.notify_all()
        self._stop_audio()

    def cancel(self):
        with self.wake:
            self.pending = None
            self.generation += 1
            self.wake.notify_all()
        self._stop_audio()

    def wait_idle(self, timeout=120.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                busy = self.busy or self.pending is not None
            if not busy and not self._stream_active():
                return True
            time.sleep(0.05)
        return False

    def shutdown(self):
        with self.wake:
            self.stopping = True
            self.wake.notify_all()
        self._stop_audio()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    # -------------------------------------------------------------- playback
    def _cancelled(self, generation):
        with self.lock:
            return self.stopping or generation != self.generation

    def _speak(self, text, generation):
        log('speak: %s' % text)
        for _gs, _ps, audio in self.pipeline(
                text, voice=self.voice, speed=self.speed):
            if self._cancelled(generation):
                return
            samples = self._to_samples(audio)
            if samples.size == 0:
                continue
            if not self._play(samples, generation):
                return

    @staticmethod
    def _to_samples(audio):
        import numpy as np
        return np.asarray(audio, dtype=np.float32).reshape(-1).copy()

    def _play(self, samples, generation):
        import sounddevice as sd
        if self._cancelled(generation):
            return False
        samples = self._contiguous(samples)
        try:
            sd.play(samples, samplerate=SAMPLE_RATE, device=self.device,
                    blocking=False)
        except Exception:                                # noqa: BLE001
            # Some output devices refuse 24 kHz; resample to their default.
            info = sd.query_devices(self.device, 'output')
            rate = int(float(info['default_samplerate']))
            log('device rejected 24 kHz, resampling to %d Hz' % rate)
            samples = self._contiguous(
                resample_linear(samples, SAMPLE_RATE, rate))
            sd.play(samples, samplerate=rate, device=self.device,
                    blocking=False)

        while True:
            if self._cancelled(generation):
                sd.stop()
                return False
            if not self._stream_active():
                return True
            time.sleep(0.02)

    @staticmethod
    def _contiguous(samples):
        import numpy as np
        return np.ascontiguousarray(samples, dtype=np.float32)

    @staticmethod
    def _stream_active():
        try:
            import sounddevice as sd
            stream = sd.get_stream()
            return bool(stream is not None and stream.active)
        except Exception:                                # noqa: BLE001
            return False

    def _stop_audio(self):
        try:
            import sounddevice as sd
            sd.stop()
        except Exception:                                # noqa: BLE001
            pass

    # ------------------------------------------------------------ diagnostics
    def save_wav(self, text, path):
        import numpy as np
        import soundfile as sf
        chunks = [self._to_samples(audio)
                  for _gs, _ps, audio in self.pipeline(
                      text, voice=self.voice, speed=self.speed)]
        audio = np.concatenate(chunks) if chunks else np.zeros(
            0, dtype=np.float32)
        sf.write(path, audio, SAMPLE_RATE)
        log('wrote %s' % path)


class RelayClient(object):
    """Keeps the car-side relay alive over SSH."""

    def __init__(self, host, topic, local_relay, remote_path, player,
                 ssh_bin='ssh', scp_bin='scp'):
        self.host = host
        self.topic = topic
        self.local_relay = local_relay
        self.remote_path = remote_path
        self.player = player
        self.ssh_bin = shutil.which(ssh_bin) or ssh_bin
        self.scp_bin = shutil.which(scp_bin) or scp_bin
        self.stop_event = threading.Event()
        self.proc = None
        self.backoff = 2.0

    def push_relay(self):
        if not os.path.isfile(self.local_relay):
            raise IOError('relay script not found: %s' % self.local_relay)
        cmd = [self.scp_bin, '-q',
               '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
               self.local_relay,
               '%s:%s' % (self.host, self.remote_path)]
        subprocess.run(cmd, check=True, timeout=30)

    def start(self):
        remote = (
            'source /opt/ros/noetic/setup.bash && '
            'exec env PYTHONUNBUFFERED=1 PYTHONIOENCODING=utf-8 '
            'python3 -u %s --topic %s'
            % (shlex.quote(self.remote_path), shlex.quote(self.topic)))
        cmd = [self.ssh_bin, '-T',
               '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
               '-o', 'ServerAliveInterval=10',
               '-o', 'ServerAliveCountMax=3',
               self.host, remote]
        return subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, encoding='utf-8',
            errors='replace', bufsize=1)

    def read_stdout(self, proc):
        for line in proc.stdout:
            line = line.rstrip('\r\n')
            if not line:
                continue
            if not line.startswith(PREFIX):
                log('[relay] %s' % line)
                continue
            try:
                msg = json.loads(line[len(PREFIX):])
            except ValueError:
                log('[relay] bad json: %s' % line)
                continue
            kind = msg.get('type')
            if kind == 'speech':
                self.player.submit(msg.get('text', ''))
            elif kind in ('boot', 'ready', 'shutdown'):
                log('[relay] %s' % msg)
                if kind == 'ready':
                    self.backoff = 2.0
            else:
                log('[relay] %s' % msg)

    def read_stderr(self, proc):
        for line in proc.stderr:
            line = line.rstrip('\r\n')
            if line:
                log('[relay stderr] %s' % line)

    def stop(self):
        self.stop_event.set()
        proc = self.proc
        if proc is None:
            return
        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except (IOError, OSError, ValueError):
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()


def run_live(client, player):
    try:
        while not client.stop_event.is_set():
            try:
                client.push_relay()
            except Exception as err:                     # noqa: BLE001
                log('scp failed: %s' % err)
                client.stop_event.wait(client.backoff)
                client.backoff = min(client.backoff * 2.0, 30.0)
                continue

            log('connecting to %s ...' % client.host)
            try:
                proc = client.start()
            except Exception as err:                     # noqa: BLE001
                log('ssh failed: %s' % err)
                client.stop_event.wait(client.backoff)
                client.backoff = min(client.backoff * 2.0, 30.0)
                continue
            client.proc = proc
            readers = [
                threading.Thread(target=client.read_stdout, args=(proc,),
                                 name='relay_stdout'),
                threading.Thread(target=client.read_stderr, args=(proc,),
                                 name='relay_stderr'),
            ]
            for reader in readers:
                reader.daemon = True
                reader.start()
            proc.wait()
            player.cancel()
            if client.stop_event.is_set():
                break
            log('relay exited (rc=%s), reconnecting in %.0fs'
                % (proc.returncode, client.backoff))
            client.stop_event.wait(client.backoff)
            client.backoff = min(client.backoff * 2.0, 30.0)
    except KeyboardInterrupt:
        log('interrupted')
    finally:
        client.stop()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Play the car /speech topic on this PC with Kokoro')
    parser.add_argument('--once', default='',
                        help='speak this text locally, do not connect')
    parser.add_argument('--demo', action='store_true',
                        help='speak the four sample announcements locally')
    parser.add_argument('--ssh-host', default='vehicle',
                        help='SSH host alias of the car (default vehicle)')
    parser.add_argument('--topic', default='/speech',
                        help='ROS topic to relay (default /speech)')
    parser.add_argument('--voice', default='zf_xiaoxiao',
                        help='Kokoro Chinese voice')
    parser.add_argument('--lang', default='z',
                        help='Kokoro language code (z = Mandarin)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='speech rate multiplier')
    parser.add_argument('--threads', type=int, default=4,
                        help='torch CPU threads')
    parser.add_argument('--device', type=int, default=None,
                        help='sounddevice output index (default: system)')
    parser.add_argument('--save-wav', default='',
                        help='with --once, write a wav instead of playing')
    parser.add_argument('--relay-path', default='/tmp/aic26_speech_relay.py',
                        help='relay path on the car')
    parser.add_argument('--local-relay', default='',
                        help='local relay script (default: next to this file)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')

    if args.save_wav and not args.once:
        log('--save-wav needs --once')
        return 2

    local_relay = args.local_relay or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'speech_relay.py')

    player = KokoroPlayer(lang=args.lang, voice=args.voice,
                          speed=args.speed, threads=args.threads,
                          device=args.device)
    player.start()
    try:
        player.wait_ready()
        if args.save_wav:
            player.save_wav(args.once, args.save_wav)
        elif args.demo:
            for text in DEMO_TEXTS:
                player.submit(text)
                if not player.wait_idle():
                    log('demo playback timed out')
                    break
        elif args.once:
            player.submit(args.once)
            if not player.wait_idle():
                log('playback timed out')
                return 1
        else:
            client = RelayClient(args.ssh_host, args.topic, local_relay,
                                 args.relay_path, player)
            run_live(client, player)
    except KeyboardInterrupt:
        log('interrupted')
    except Exception as err:                             # noqa: BLE001
        log('error: %s' % err)
        return 1
    finally:
        player.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
