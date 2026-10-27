@echo off
setlocal EnableExtensions

rem Start the VcXsrv X server used by the automatic DISPLAY injection in
rem bringup.launch. Run this once after a Windows reboot.

set "VCXSRV=C:\Program Files\VcXsrv\vcxsrv.exe"

if not exist "%VCXSRV%" (
    echo [ERROR] VcXsrv not found: "%VCXSRV%"
    pause
    exit /b 1
)

tasklist /FI "IMAGENAME eq vcxsrv.exe" 2>NUL | find /I "vcxsrv.exe" >NUL
if errorlevel 1 (
    echo Starting VcXsrv on DISPLAY=:0 ...
    start "" "%VCXSRV%" :0 -multiwindow -clipboard -wgl -ac -listen tcp
    timeout /t 2 /nobreak >NUL
) else (
    echo VcXsrv is already running.
)

echo.
echo Done. You can now run:
echo   roslaunch aic26_bringup bringup.launch mode:=mission backend:=sim
echo.
pause
endlocal
