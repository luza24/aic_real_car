@echo off
setlocal EnableExtensions

rem ==========================================================================
rem  aic26 simulation GUI launcher
rem  Starts VcXsrv (if needed), then runs the Gazebo simulation over SSH with
rem  X11 forwarding so Gazebo, RViz, camera_view and the detection result
rem  panel appear on this Windows desktop.
rem
rem  This is the mode that starts Gazebo. backend:=real does not.
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

echo [2/2] Starting aic26 Gazebo simulation with Gazebo / RViz / camera / result GUI ...
echo       The simulated car will drive the mission. No real hardware is used.
echo       The launcher refuses to run if another aic26 stack or Gazebo is active.
echo       Press Ctrl+C in this window to stop.
echo.

"%SSH_EXE%" -o ConnectTimeout=10 -Y vehicle "bash -lc '~/aic26_ws_speech/src/aic26_bringup/tools/run_gui_sim.sh'"
set "RC=%ERRORLEVEL%"

echo.
echo Process finished with exit code %RC%.
pause
endlocal
