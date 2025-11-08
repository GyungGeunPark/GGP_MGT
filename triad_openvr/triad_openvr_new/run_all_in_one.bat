@echo off
title Triad OpenVR Runner
color 0A

REM Set working directory to script location
cd /d "%~dp0"

echo ================================================================================
echo                           TRIAD OPENVR ALL-IN-ONE RUNNER
echo ================================================================================
echo.

REM Check if virtual environment exists
if not exist "smart1" (
    echo [SETUP] Virtual environment 'smart1' not found. Creating...
    echo.

    REM Create virtual environment
    python -m venv smart1
    if errorlevel 1 (
        echo [ERROR] Failed to create virtual environment!
        pause
        exit /b 1
    )

    REM Install dependencies using absolute path
    echo [SETUP] Installing dependencies...
    "%~dp0smart1\Scripts\python.exe" -m pip install --upgrade pip
    "%~dp0smart1\Scripts\python.exe" -m pip install -r requirements.txt

    echo.
    echo [SETUP] Installation complete!
    echo.
) else (
    echo [INFO] Virtual environment 'smart1' found.
)

echo.
echo ================================================================================
echo                              PYTHON ENVIRONMENT INFO
echo ================================================================================
"%~dp0smart1\Scripts\python.exe" --version
echo Virtual Environment: smart1
echo Working Directory: %CD%
echo.

echo ================================================================================
echo                              RUNNING TRIAD_OPENVR.PY
echo ================================================================================
echo.

REM Run the main script using virtual environment python
"%~dp0smart1\Scripts\python.exe" "%~dp0triad_openvr\triad_openvr_tcp.py"

echo.
echo ================================================================================
echo                                 EXECUTION COMPLETE
echo ================================================================================
echo.
pause