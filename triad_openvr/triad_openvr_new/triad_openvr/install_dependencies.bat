@echo off
echo ========================================
echo Installing Dependencies for Triad OpenVR
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

echo Installing required packages...
echo.

REM pip 업그레이드
echo Upgrading pip...
python -m pip install --upgrade pip

echo.
echo Installing packages from requirements.txt...
pip install -r requirements.txt

echo.
echo ========================================
echo Verifying installation...
echo ========================================
python -c "import openvr; print('✓ openvr version:', openvr.__version__)"
python -c "import numpy; print('✓ numpy version:', numpy.__version__)"

echo.
echo Installation complete!
pause