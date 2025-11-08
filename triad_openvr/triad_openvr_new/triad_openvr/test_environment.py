#!/usr/bin/env python
"""
환경 테스트 스크립트
필요한 모든 패키지가 올바르게 설치되었는지 확인합니다.
"""

import sys
import importlib

def test_import(module_name):
    """모듈 임포트 테스트"""
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, '__version__', 'unknown')
        print(f"✓ {module_name} (version: {version})")
        return True
    except ImportError as e:
        print(f"✗ {module_name} - {e}")
        return False

def test_local_modules():
    """로컬 모듈 테스트"""
    try:
        import triad_openvr
        print("✓ triad_openvr.py")
    except ImportError as e:
        print(f"✗ triad_openvr.py - {e}")
        return False
    
    try:
        import tcp_emitter
        print("✓ tcp_emitter.py")
    except ImportError as e:
        print(f"✗ tcp_emitter.py - {e}")
        return False
    
    return True

def test_openvr_init():
    """OpenVR 초기화 테스트"""
    try:
        import openvr
        # VR 시스템 초기화 시도 (실제 VR 없어도 테스트 가능)
        try:
            vr = openvr.init(openvr.VRApplication_Other)
            openvr.shutdown()
            print("✓ OpenVR initialization successful")
            return True
        except openvr.OpenVRError as e:
            if "Init_VRClientDLLNotFound" in str(e):
                print("⚠ OpenVR: SteamVR not installed (expected if no VR system)")
            elif "Init_HmdNotFound" in str(e):
                print("⚠ OpenVR: No HMD found (expected if no VR headset connected)")
            else:
                print(f"⚠ OpenVR initialization: {e}")
            return True  # OpenVR는 설치됨, VR 하드웨어만 없음
    except Exception as e:
        print(f"✗ OpenVR test failed: {e}")
        return False

def main():
    print("="*50)
    print("Triad OpenVR TCP - Environment Test")
    print("="*50)
    print()
    
    print("Testing Python version...")
    print(f"✓ Python {sys.version}")
    print()
    
    print("Testing required packages...")
    all_ok = True
    
    # 필수 패키지 테스트
    if not test_import('openvr'):
        all_ok = False
    if not test_import('numpy'):
        all_ok = False
    
    # 선택적 패키지 테스트
    test_import('struct')  # 기본 라이브러리
    test_import('socket')  # 기본 라이브러리
    test_import('threading')  # 기본 라이브러리
    print()
    
    print("Testing local modules...")
    if not test_local_modules():
        all_ok = False
    print()
    
    print("Testing OpenVR...")
    test_openvr_init()
    print()
    
    print("="*50)
    if all_ok:
        print("✓ All required packages are installed!")
        print("You can now run: python triad_openvr_tcp.py")
    else:
        print("✗ Some packages are missing!")
        print("Run: pip install -r requirements.txt")
    print("="*50)
    
    return 0 if all_ok else 1

if __name__ == "__main__":
    sys.exit(main())