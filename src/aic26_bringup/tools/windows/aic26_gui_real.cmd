@echo off
setlocal EnableExtensions

rem ==========================================================================
rem  aic26 real-robot GUI launcher
rem  Starts VcXsrv (if needed), then runs the real mission over SSH with X11
rem  forwarding so RViz, camera_view and the detection result panel appear on
rem  this Windows desktop.
rem
rem  backend:=real deliberately does NOT start Gazebo. Use aic26_gui_sim.cmd
rem  for the Gazebo simulation.
rem ==========================================================================

set "VCXSRV=C:\Program Files\VcXsrv\vcxsrv.exe"
set "SSH_EXE=D:\Git\usr\bin\ssh.exe"
if not exist "%SSH_EXE%" set "SSH_EXE=ssh"

set "PATH=C:\Program Files\VcXsrv;%PATH%"
set "HOME=%USERPROFILE%"
set "XAUTHORITY=%USERPROFILE%\.Xauthority"
set "DISPLAY=localhost:0.0"

if not exist "%VCXSRV%" (
    echo [ERROR] VcXsrv not found: "%VCXSRV%"
    echo Install VcXsrv or edit VCXSRV in this file.
    pause
    exit /b 1
)

tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I "vcxsrv.exe" >NUL
if errorlevel 1 (
    echo [1/2] Starting VcXsrv on DISPLAY=:0 ...
    start "" "%VCXSRV%" :0 -multiwindow -clipboard -wgl -ac -listen tcp
    timeout /t 3 /nobreak >NUL
) else (
    echo [1/2] VcXsrv is already running.
)

echo [2/2] Starting aic26 real mission with RViz / camera / result GUI ...
echo       backend:=real does NOT start Gazebo (Gazebo is sim only).
echo       The launcher refuses to run if another aic26 stack is active.
echo       Press Ctrl+C in this window to stop.
echo.

"%SSH_EXE%" -o ConnectTimeout=10 -Y vehicle "bash -lc '~/aic26_ws_speech/src/aic26_bringup/tools/run_gui_real.sh'"
set "RC=%ERRORLEVEL%"

echo.
echo Process finished with exit code %RC%.
pause
endlocal
