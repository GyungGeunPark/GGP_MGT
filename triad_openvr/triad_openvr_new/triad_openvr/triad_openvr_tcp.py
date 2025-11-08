#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Triad OpenVR TCP 통신 메인 실행 파일
Unity와 TCP 통신으로 VR 트래커 데이터를 전송합니다.
"""

import sys
import os
import argparse
import time

# Windows 콘솔 인코딩 문제 해결
if sys.platform == 'win32':
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass

# 현재 디렉토리를 Python 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tcp_emitter import TCPEmitter
import triad_openvr

def print_banner():
    """프로그램 배너 출력"""
    print("="*60)
    print("  Triad OpenVR TCP Communication System")
    print("  VR Tracker -> TCP -> Unity")
    print("="*60)
    print()

def test_vr_connection():
    """VR 연결 테스트"""
    print("Testing VR connection...")
    try:
        v = triad_openvr.triad_openvr()
        v.print_discovered_objects()
        
        # tracker_1 확인
        if "tracker_1" in v.devices:
            print("✓ Tracker_1 found and ready")
            data = v.devices["tracker_1"].get_pose_quaternion()
            if data:
                print(f"✓ Initial position: [{data[0]:.3f}, {data[1]:.3f}, {data[2]:.3f}]")
            return True
        else:
            print("✗ Tracker_1 not found. Available devices:")
            for device_name in v.devices.keys():
                print(f"  - {device_name}")
            return False
    except Exception as e:
        print(f"✗ VR connection failed: {e}")
        return False

def main():
    """메인 실행 함수"""
    parser = argparse.ArgumentParser(
        description='Triad OpenVR TCP Communication System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Default: 250Hz, localhost:8051
  %(prog)s -f 100            # 100Hz update rate
  %(prog)s -h 192.168.1.100  # Custom host
  %(prog)s -p 9000           # Custom port
  %(prog)s --test            # Test mode only
        """
    )
    
    parser.add_argument('-f', '--frequency', 
                       type=float, 
                       default=250,
                       help='Update frequency in Hz (default: 250)')
    
    parser.add_argument('-H', '--host', 
                       type=str,
                       default='127.0.0.1',
                       help='TCP server host/IP (default: 127.0.0.1)')
    
    parser.add_argument('-p', '--port',
                       type=int,
                       default=8051,
                       help='TCP server port (default: 8051)')
    
    parser.add_argument('--test',
                       action='store_true',
                       help='Test VR connection only')
    
    parser.add_argument('-v', '--verbose',
                       action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # 배너 출력
    print_banner()
    
    # VR 연결 테스트
    if not test_vr_connection():
        print("\n⚠ Warning: VR connection test failed!")
        if not args.test:
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                print("Exiting...")
                return 1
    
    # 테스트 모드면 여기서 종료
    if args.test:
        print("\nTest completed.")
        return 0
    
    # TCP 통신 시작
    print(f"\nStarting TCP communication:")
    print(f"  Host: {args.host}")
    print(f"  Port: {args.port}")
    print(f"  Frequency: {args.frequency} Hz")
    print(f"  Interval: {1/args.frequency:.6f} seconds")
    print("\nPress Ctrl+C to stop\n")
    print("-"*60)
    
    try:
        # TCP Emitter 생성 및 실행
        emitter = TCPEmitter(host=args.host, port=args.port)
        emitter.run(interval=1/args.frequency)
    except KeyboardInterrupt:
        print("\n\nShutdown requested by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        return 1
    
    print("\nProgram terminated successfully.")
    return 0

if __name__ == "__main__":
    sys.exit(main())