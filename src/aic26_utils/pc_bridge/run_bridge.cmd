@echo off
setlocal
set HF_ENDPOINT=https://hf-mirror.com
"C:\Users\luza\miniconda3\envs\kokoro\python.exe" "%~dp0speech_bridge.py" --ssh-host vehicle %*
echo.
echo Bridge stopped. Press any key to close.
pause >nul
