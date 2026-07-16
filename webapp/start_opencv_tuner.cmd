@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title OpenCV Web Tuner

set "PY_CMD="
where py >nul 2>&1
if not errorlevel 1 set "PY_CMD=py"
if not defined PY_CMD (
    where python >nul 2>&1
    if not errorlevel 1 set "PY_CMD=python"
)

if not defined PY_CMD (
    echo [ERROR] Python was not found.
    echo Install Python 3.10 or newer and enable "Add Python to PATH".
    pause
    exit /b 1
)

if not exist "opencv_web_tuner.py" (
    echo [ERROR] opencv_web_tuner.py is missing from this folder.
    pause
    exit /b 1
)

if not exist "requirements.txt" (
    echo [ERROR] requirements.txt is missing from this folder.
    pause
    exit /b 1
)

if not exist ".venv\Scripts\python.exe" (
    echo [1/3] Creating the Python virtual environment...
    %PY_CMD% -m venv ".venv"
    if errorlevel 1 goto :failed
)

set "REQ_HASH=unknown"
for /f "usebackq delims=" %%H in (`powershell -NoProfile -Command "(Get-FileHash -Algorithm SHA256 'requirements.txt').Hash"`) do set "REQ_HASH=%%H"
set "OLD_HASH=missing"
if exist ".venv\requirements.sha256" set /p OLD_HASH=<".venv\requirements.sha256"

if /I not "%REQ_HASH%"=="%OLD_HASH%" (
    echo [2/3] Installing or updating dependencies...
    ".venv\Scripts\python.exe" -m pip install -r "requirements.txt"
    if errorlevel 1 goto :failed
    >".venv\requirements.sha256" echo %REQ_HASH%
) else (
    echo [2/3] Dependencies are ready.
)

echo [3/3] Starting http://127.0.0.1:5000
start "" powershell -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Seconds 2; Start-Process 'http://127.0.0.1:5000'"
".venv\Scripts\python.exe" "opencv_web_tuner.py"

if errorlevel 1 goto :failed
exit /b 0

:failed
echo.
echo [ERROR] Startup failed. Review the message above.
pause
exit /b 1
