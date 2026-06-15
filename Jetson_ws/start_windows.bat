@echo off
setlocal EnableExtensions

cd /d "%~dp0"
chcp 65001 >nul

rem TECHX Vision Windows launcher.
rem Defaults are tuned for local Gemini335L/KFS validation on this laptop.

set "DEFAULT_CONDA_ENV=D:\Anaconda\envs\yolov8"
set "PY_EXE=%DEFAULT_CONDA_ENV%\python.exe"

if not exist "%PY_EXE%" (
    echo [WARN] %PY_EXE% not found.
    echo [WARN] Falling back to python from PATH.
    set "PY_EXE=python"
) else (
    set "PATH=%DEFAULT_CONDA_ENV%;%DEFAULT_CONDA_ENV%\Library\bin;%DEFAULT_CONDA_ENV%\Scripts;%PATH%"
)

rem Avoid Windows GBK crashes in Python status output.
if not defined PYTHONUTF8 set "PYTHONUTF8=1"
if not defined PYTHONIOENCODING set "PYTHONIOENCODING=utf-8"

rem Local Windows validation usually does not own the Jetson field IP
rem 192.168.10.101, so UDP must not be fatal by default.
if not defined TECHX_UDP_OPTIONAL set "TECHX_UDP_OPTIONAL=1"

rem This repository currently ships kfs_v3 weights but not head_v1 weights.
rem Keep all/head/assembly/qr selectable by setting TECHX_STAGE before launching.
if not defined TECHX_STAGE set "TECHX_STAGE=kfs"

rem Jetson-only checks are not useful on Windows.
if not defined TECHX_SKIP_TIME_SYNC set "TECHX_SKIP_TIME_SYNC=1"
if not defined TECHX_SKIP_JETSON_OPT set "TECHX_SKIP_JETSON_OPT=1"

rem Windows UI preview: keep the live view responsive while still detecting often.
if not defined TECHX_INFER_EVERY_N set "TECHX_INFER_EVERY_N=2"
if not defined TECHX_DISPLAY_EVERY_N set "TECHX_DISPLAY_EVERY_N=2"
if not defined TECHX_UI_TICK_MS set "TECHX_UI_TICK_MS=20"
if not defined TECHX_UI_TABLE_REFRESH_SEC set "TECHX_UI_TABLE_REFRESH_SEC=0.20"

echo ========================================
echo   TECHX Vision Windows Launcher
echo ========================================
echo Python: %PY_EXE%
echo Stage : %TECHX_STAGE%
echo UDP optional: %TECHX_UDP_OPTIONAL%
echo Infer every N frames: %TECHX_INFER_EVERY_N%
echo.

echo [1/4] Python
"%PY_EXE%" --version
if errorlevel 1 goto :fail

echo.
echo [2/4] Core dependencies
"%PY_EXE%" -c "import cv2, numpy, ultralytics; print('OK cv2/numpy/ultralytics')"
if errorlevel 1 goto :fail

echo.
echo [3/4] Gemini/Orbbec SDK
"%PY_EXE%" -c "import pyorbbecsdk; print('OK pyorbbecsdk')"
if errorlevel 1 goto :fail

echo.
echo [4/4] Launching TECHX Vision
echo Tip: pass --headless for console-only mode, or set TECHX_STAGE=all/head/kfs/assembly/qr.
echo.
"%PY_EXE%" launch.py --fast-start %*
if errorlevel 1 goto :fail

echo.
echo [OK] TECHX Vision exited normally.
pause
exit /b 0

:fail
echo.
echo [ERROR] Windows launch failed. Check the message above.
pause
exit /b 1
