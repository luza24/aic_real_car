# aic26_speech_pc

Play the car's ROS `/speech` announcements on this Windows PC with Kokoro.

```
car mission_controller --/speech--> /tmp/aic26_speech_relay.py
        (rospy, car side)                  │ stdout: AIC26_SPEECH {...}
                                           ▼
                                  ssh vehicle
                                           ▼
                              speech_bridge.py (this PC)
                              ├── Kokoro KPipeline (Chinese)
                              └── sounddevice -> Windows default audio
```

The PC does not run ROS and the car workspace is not modified. The relay is
copied to `/tmp` on the car each time the bridge starts.

## Requirements

- Windows OpenSSH (`ssh`/`scp`) with the `vehicle` host already configured
  (the same connection VS Code Remote-SSH uses).
- Conda (miniconda is already installed on this PC).
- Network access for the first model download.

Kokoro supports Python 3.10-3.12 only. The system Python 3.8 and the existing
3.9/3.13/3.14 cannot be used.

## Install

```bash
conda create -n kokoro python=3.11 pip -y

# Git Bash activation
source /c/Users/luza/miniconda3/etc/profile.d/conda.sh
conda activate kokoro

python -m pip install --upgrade pip
# CPU-only torch first, otherwise pip may download the large CUDA build
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r C:/Users/luza/Desktop/aic26_speech_pc/requirements.txt
```

Check the interpreter and the audio devices:

```bash
python -c "import sys, torch, kokoro, sounddevice; print(sys.version); print(torch.__version__)"
python -m sounddevice
```

First synthesis downloads the Kokoro model. In mainland China, if Hugging
Face is slow:

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

After the download, offline use works with:

```bash
export HF_HUB_OFFLINE=1
```

## Run

```bash
cd C:/Users/luza/Desktop/aic26_speech_pc

# 1) Local check, no car needed
python speech_bridge.py --once "语音播报测试"
python speech_bridge.py --demo

# 2) Live car /speech
python speech_bridge.py --ssh-host vehicle
```

When the car's ROS master is running, publish a test message from any machine:

```bash
ssh vehicle 'source /opt/ros/noetic/setup.bash && \
  rostopic pub -1 /speech std_msgs/String \
  "data: \"本区域共检测到 12 人，其中 2 人为非社区人员。\""'
```

Useful options:

| Option | Meaning |
| --- | --- |
| `--voice zf_xiaoxiao` | Kokoro Chinese voice |
| `--speed 1.0` | speech rate |
| `--device N` | sounddevice output index |
| `--save-wav out.wav` | with `--once`, save instead of play |
| `--relay-path /tmp/...` | relay location on the car |

## How it works

`speech_bridge.py`:

1. `scp` the relay to the car.
2. Start `ssh -T vehicle "source /opt/ros/noetic/setup.bash && python3 -u /tmp/aic26_speech_relay.py --topic /speech"`.
3. Parse the `AIC26_SPEECH {...}` JSON lines (rospy log lines are ignored).
4. Kokoro synthesizes 24 kHz audio and `sounddevice` plays it on the default
   Windows output; a new message stops the current playback.
5. On SSH/relay failure it reconnects with backoff; the Kokoro model stays
   loaded.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `ModuleNotFoundError: kokoro` | wrong interpreter; use `...\envs\kokoro\python.exe` |
| conda env creation fails | add `-c conda-forge` |
| Torch download is huge | install the CPU wheel first (see above) |
| Model download fails | `export HF_ENDPOINT=https://hf-mirror.com` and retry |
| No sound | `python -m sounddevice`; try `--device N`; close exclusive-mode apps |
| `relay` prints `boot` but no `ready` | the car's ROS master is not running |
| No relay output at all | test `ssh vehicle true`; check `scp`; run the manual relay command |
| Chinese garbled in the console | use Windows Terminal; the bridge reconfigures UTF-8 |
| Double audio | do not run the car's `speech.launch` and this bridge at the same time |
| A stale relay node remains | the relay exits on SSH stdin EOF; check `rosnode list` |

## Note

This tool is only for PC-side testing. The car's production speech path is
`aic26_utils/scripts/speech_broadcaster.py` + `kokoro_worker.py` on the
`feature/speech-announcement` branch, which plays on the car's Bluetooth
speaker.
