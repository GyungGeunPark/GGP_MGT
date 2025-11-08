@echo off
echo ========================================
echo Building Triad OpenVR TCP Executable
echo ========================================
echo.

REM Python이 설치되어 있는지 확인
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7 or higher
    pause
    exit /b 1
)

REM 빌드 스크립트 실행
python build_exe.py

pause