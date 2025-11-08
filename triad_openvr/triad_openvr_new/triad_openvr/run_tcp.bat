@echo off
echo ========================================
echo Triad OpenVR TCP Communication
echo ========================================
echo.

REM smart1 가상환경 활성화
echo Activating smart1 virtual environment...
call conda activate smart1
if %errorlevel% neq 0 (
    echo ERROR: Failed to activate smart1 environment
    echo Please ensure that smart1 environment exists
    echo You can create it with: conda create -n smart1 python=3.8
    pause
    exit /b 1
)
echo Virtual environment 'smart1' activated successfully!
echo.

REM Python이 설치되어 있는지 확인
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed in smart1 environment
    echo Please install Python in the smart1 environment
    pause
    exit /b 1
)

REM 필요한 패키지가 설치되어 있는지 확인
echo Checking required packages in smart1 environment...
python -c "import openvr" 2>nul
if %errorlevel% neq 0 (
    echo.
    echo WARNING: openvr module not found in smart1 environment!
    echo Installing required packages...
    pip install openvr numpy
    if %errorlevel% neq 0 (
        echo ERROR: Failed to install required packages
        echo Please run: pip install openvr numpy
        pause
        exit /b 1
    )
)

REM 기본 설정으로 실행 (로컬호스트, 포트 8051, 250Hz)
echo.
echo Starting with default settings...
echo   Host: 127.0.0.1
echo   Port: 8051
echo   Frequency: 250 Hz
echo.
echo Press Ctrl+C to stop
echo ----------------------------------------
python triad_openvr_tcp.py

pause