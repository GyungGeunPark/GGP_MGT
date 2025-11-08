"""
PyInstaller를 사용하여 triad_openvr_tcp.py를 실행 파일로 빌드하는 스크립트
"""

import os
import sys
import subprocess
import shutil

def check_pyinstaller():
    """PyInstaller 설치 확인"""
    try:
        import PyInstaller
        print(f"✓ PyInstaller {PyInstaller.__version__} found")
        return True
    except ImportError:
        print("✗ PyInstaller not found")
        return False

def install_pyinstaller():
    """PyInstaller 설치"""
    print("Installing PyInstaller...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])
        print("✓ PyInstaller installed successfully")
        return True
    except subprocess.CalledProcessError:
        print("✗ Failed to install PyInstaller")
        return False

def check_requirements():
    """필요한 패키지 확인"""
    required = ['openvr', 'numpy']
    missing = []
    
    for package in required:
        try:
            __import__(package)
            print(f"✓ {package} found")
        except ImportError:
            print(f"✗ {package} not found")
            missing.append(package)
    
    return missing

def install_requirements(packages):
    """필요한 패키지 설치"""
    for package in packages:
        print(f"Installing {package}...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            print(f"✓ {package} installed successfully")
        except subprocess.CalledProcessError:
            print(f"✗ Failed to install {package}")
            return False
    return True

def build_executable():
    """실행 파일 빌드"""
    print("\nBuilding executable...")
    
    # PyInstaller 명령어 구성
    cmd = [
        "pyinstaller",
        "--onefile",  # 단일 실행 파일로 생성
        "--name", "triad_openvr_tcp",  # 실행 파일 이름
        "--icon", "NONE",  # 아이콘 (없으면 기본값)
        "--console",  # 콘솔 창 표시
        "--clean",  # 이전 빌드 정리
        "--noconfirm",  # 덮어쓰기 확인 없음
        "--add-data", "triad_openvr.py;.",  # triad_openvr.py 포함
        "--add-data", "tcp_emitter.py;.",  # tcp_emitter.py 포함
        "--hidden-import", "openvr",  # openvr 명시적 포함
        "--hidden-import", "numpy",  # numpy 명시적 포함
        "triad_openvr_tcp.py"  # 메인 스크립트
    ]
    
    try:
        subprocess.check_call(cmd)
        print("✓ Build completed successfully")
        
        # 실행 파일 위치 확인
        exe_path = os.path.join("dist", "triad_openvr_tcp.exe")
        if os.path.exists(exe_path):
            print(f"\n✓ Executable created: {os.path.abspath(exe_path)}")
            
            # 실행 파일을 현재 디렉토리로 복사
            current_exe = "triad_openvr_tcp.exe"
            shutil.copy2(exe_path, current_exe)
            print(f"✓ Executable copied to: {os.path.abspath(current_exe)}")
            
            return True
        else:
            print("✗ Executable not found")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"✗ Build failed: {e}")
        return False

def create_spec_file():
    """PyInstaller spec 파일 생성 (고급 설정용)"""
    spec_content = """
# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

a = Analysis(
    ['triad_openvr_tcp.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('triad_openvr.py', '.'),
        ('tcp_emitter.py', '.')
    ],
    hiddenimports=['openvr', 'numpy'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='triad_openvr_tcp',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
"""
    
    with open("triad_openvr_tcp.spec", "w") as f:
        f.write(spec_content)
    print("✓ Spec file created: triad_openvr_tcp.spec")

def main():
    print("="*50)
    print("Triad OpenVR TCP - Executable Builder")
    print("="*50)
    print()
    
    # PyInstaller 확인 및 설치
    if not check_pyinstaller():
        response = input("PyInstaller is required. Install it? (y/n): ")
        if response.lower() == 'y':
            if not install_pyinstaller():
                print("\nFailed to install PyInstaller. Exiting...")
                return 1
        else:
            print("\nPyInstaller is required to build executable. Exiting...")
            return 1
    
    print()
    
    # 필요한 패키지 확인 및 설치
    missing = check_requirements()
    if missing:
        print(f"\nMissing packages: {', '.join(missing)}")
        response = input("Install missing packages? (y/n): ")
        if response.lower() == 'y':
            if not install_requirements(missing):
                print("\nFailed to install required packages. Exiting...")
                return 1
        else:
            print("\nRequired packages are missing. Build may fail.")
    
    print()
    
    # spec 파일 생성 (옵션)
    response = input("Create custom spec file for advanced configuration? (y/n): ")
    if response.lower() == 'y':
        create_spec_file()
    
    print()
    
    # 실행 파일 빌드
    if build_executable():
        print("\n" + "="*50)
        print("BUILD SUCCESSFUL!")
        print("="*50)
        print("\nYou can now run the executable:")
        print("  triad_openvr_tcp.exe")
        print("\nOptions:")
        print("  triad_openvr_tcp.exe --help")
        print("  triad_openvr_tcp.exe -f 100")
        print("  triad_openvr_tcp.exe -H 192.168.1.100 -p 9000")
        return 0
    else:
        print("\n" + "="*50)
        print("BUILD FAILED!")
        print("="*50)
        return 1

if __name__ == "__main__":
    sys.exit(main())