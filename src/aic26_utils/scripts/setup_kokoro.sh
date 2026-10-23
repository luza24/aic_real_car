#!/usr/bin/env bash
# One-time setup for the Kokoro speech worker.
#
# Creates a Python 3.10-3.12 virtual environment (ROS Noetic uses 3.8, which
# Kokoro does not support), installs Kokoro and the Mandarin G2P extra, and
# pre-downloads the model by synthesizing one test sentence.
#
#   bash aic26_utils/scripts/setup_kokoro.sh
#   AIC26_KOKORO_VENV=~/tts/venv bash aic26_utils/scripts/setup_kokoro.sh
#
# The first run needs network access (model + voices). After that the cache in
# ~/.cache/huggingface is enough; copy it to the car for offline contests.

set -euo pipefail

VENV="${AIC26_KOKORO_VENV:-$HOME/aic26_kokoro_venv}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKER="$SCRIPT_DIR/kokoro_worker.py"
CHECK_WAV="/tmp/aic26_kokoro_check.wav"

echo "== AIC26 Kokoro TTS setup =="
echo "venv: $VENV"

missing=""
command -v espeak-ng >/dev/null 2>&1 || missing="$missing espeak-ng"
command -v paplay >/dev/null 2>&1 || missing="$missing pulseaudio-utils"
if [ -n "$missing" ]; then
    echo "missing system packages:$missing"
    echo "install with: sudo apt install espeak-ng pulseaudio-utils"
fi

# ---------------------------------------------------------------- interpreter
if [ -x "$VENV/bin/python" ]; then
    echo "reusing $VENV"
else
    PY=""
    for cand in python3.12 python3.11 python3.10; do
        if command -v "$cand" >/dev/null 2>&1; then
            PY="$cand"
            break
        fi
    done
    if [ -n "$PY" ]; then
        echo "creating venv with $PY"
        if ! "$PY" -m venv "$VENV"; then
            echo "could not create the venv; install $PY-venv" >&2
            exit 1
        fi
    elif command -v uv >/dev/null 2>&1; then
        echo "no Python 3.10-3.12 found, creating one with uv"
        uv venv -p 3.12 "$VENV"
    else
        cat <<EOF >&2
No Python 3.10-3.12 was found.
Install uv (one command) and rerun this script:
    curl -LsSf https://astral.sh/uv/install.sh | sh
or install a newer interpreter, e.g.:
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt install python3.10 python3.10-venv
    python3.10 -m venv "$VENV"
EOF
        exit 1
    fi
fi

PYTHON="$VENV/bin/python"
if [ ! -x "$PYTHON" ]; then
    echo "venv python is missing: $PYTHON" >&2
    exit 1
fi

# ------------------------------------------------------------------- packages
echo "installing kokoro, soundfile and misaki[zh]"
echo "(the first install downloads about 2 GB including torch)"
if "$PYTHON" -m pip --version >/dev/null 2>&1; then
    "$PYTHON" -m pip install --upgrade pip
    "$PYTHON" -m pip install "kokoro>=0.9.4" soundfile "misaki[zh]"
elif command -v uv >/dev/null 2>&1; then
    uv pip install --python "$PYTHON" "kokoro>=0.9.4" soundfile "misaki[zh]"
else
    "$PYTHON" -m ensurepip --upgrade
    "$PYTHON" -m pip install "kokoro>=0.9.4" soundfile "misaki[zh]"
fi

# --------------------------------------------------------------------- warm-up
echo "pre-downloading the model and synthesizing a test sentence"
"$PYTHON" "$WORKER" --check --out "$CHECK_WAV"

echo
echo "done."
echo "test wav: $CHECK_WAV"
echo "find the Bluetooth sink:"
echo "    pactl list short sinks | grep bluez"
echo "play the test through it:"
echo "    paplay --device=<sink> $CHECK_WAV"
echo "start the broadcaster next to the running robot:"
echo "    roslaunch aic26_utils speech.launch sink:=<sink>"
