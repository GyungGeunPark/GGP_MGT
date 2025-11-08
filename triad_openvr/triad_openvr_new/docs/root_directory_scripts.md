# 루트 디렉토리 스크립트 문서

## 개요
루트 디렉토리에는 `triad_openvr` 하위 폴더의 스크립트들과 동일한 스크립트들이 복사본으로 존재합니다. 또한 메인 `triad_openvr.py` 파일에는 UDP 통신 기능이 추가되어 있습니다.

## 파일 구조
```
triad_openvr/ (루트)
├── triad_openvr.py (UDP 기능 추가 버전)
├── controller_test.py (복사본)
├── tracker_test.py (복사본)
├── udp_emitter.py (복사본)
├── test_setup.py (환경 검증 스크립트)
└── triad_openvr/ (하위 폴더)
    ├── triad_openvr.py (원본 코어 모듈)
    ├── controller_test.py
    ├── tracker_test.py
    └── udp_emitter.py
```

## 1. triad_openvr.py (루트 버전 - 확장판)

### 차이점
루트 디렉토리의 `triad_openvr.py`는 하위 폴더의 원본 버전에 UDP 통신 기능이 추가된 버전입니다.

### 추가된 기능

#### UDP 설정
```python
# --- 추가된 코드 ---
# UDP 설정
UDP_IP = "127.0.0.1"  # Unity가 실행되는 컴퓨터의 IP
UDP_PORT = 8051      # C# 스크립트의 포트와 일치해야 함

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP 소켓 생성
```

#### main() 함수
독립 실행 가능한 메인 함수가 추가되어 트래커 데이터를 자동으로 UDP로 전송합니다.

**주요 특징:**
1. **자동 트래커 탐지**: 연결된 트래커를 자동으로 찾아 선택
2. **실시간 이벤트 폴링**: 장치 연결/해제 감지
3. **90Hz 전송**: VIVE 장치의 기본 주파수로 데이터 전송
4. **한글 메시지**: 한국어 사용자를 위한 친화적인 메시지

**동작 흐름:**
```python
def main():
    v = triad_openvr()
    v.print_discovered_objects()
    
    # 트래커 자동 탐지
    tracked_device_name = None
    for device_name in v.devices:
        if v.devices[device_name].device_class == "Tracker":
            tracked_device_name = device_name
            break
    
    # UDP 전송 루프
    while True:
        v.poll_vr_events()  # 이벤트 폴링
        pose_quaternion = v.devices[tracked_device_name].get_pose_quaternion()
        
        if pose_quaternion is not None:
            message = struct.pack('7d', *pose_quaternion)
            sock.sendto(message, (UDP_IP, UDP_PORT))
            update_text(f"Sent: {pose_quaternion[0]:.4f}, ...")
        
        time.sleep(1/90)  # 90Hz
```

### 사용 방법

#### 라이브러리로 사용
```python
import triad_openvr

v = triad_openvr.triad_openvr()
# 기존 API와 동일하게 사용
```

#### 독립 실행 (UDP 전송)
```bash
python triad_openvr.py
```

### 출력 예시
```
Found 1 Tracker
  tracker_1 (LHR-12345678, VIVE Tracker 3.0)

트래커 (tracker_1)의 데이터를 전송합니다...
Sent: 0.5234, 1.2345, 0.8765, 0.9876, 0.1234, 0.2345, 0.3456
```

## 2. controller_test.py, tracker_test.py, udp_emitter.py (복사본)

이 파일들은 `triad_openvr/` 하위 폴더의 파일들과 완전히 동일한 복사본입니다.

### 존재 이유
1. **편의성**: 루트에서 직접 실행 가능
2. **호환성**: 이전 버전과의 호환성 유지
3. **테스트**: 빠른 테스트 실행

### 주의사항
- 하위 폴더 버전을 수정할 경우 루트 버전도 동기화 필요
- import 경로 차이로 인한 잠재적 충돌 가능성

## 3. test_setup.py (루트 전용)

환경 설정을 검증하는 스크립트로 루트 디렉토리에만 존재합니다.

### 주요 기능
- Python 패키지 의존성 확인
- OpenVR 연결 테스트
- triad_openvr 모듈 import 테스트

(자세한 내용은 test_scripts.md 참조)

## 파일 선택 가이드

### 언제 루트 버전을 사용할까?

1. **triad_openvr.py (루트)**
   - UDP 전송 기능이 필요한 경우
   - Unity/Unreal과 통합하는 경우
   - 독립 실행 프로그램이 필요한 경우

2. **테스트 스크립트 (루트)**
   - 빠른 테스트가 필요한 경우
   - 프로젝트 루트에서 작업하는 경우

### 언제 하위 폴더 버전을 사용할까?

1. **triad_openvr.py (하위 폴더)**
   - 순수한 라이브러리 기능만 필요한 경우
   - 다른 프로젝트에 모듈로 포함시킬 경우
   - 최소한의 의존성이 필요한 경우

2. **테스트 스크립트 (하위 폴더)**
   - 모듈과 함께 패키징하는 경우
   - 명확한 파일 구조가 필요한 경우

## Import 방법

### 루트에서 import
```python
# 루트 버전 사용
import triad_openvr

# 하위 폴더 버전 명시적 사용
from triad_openvr import triad_openvr
```

### 다른 디렉토리에서 import
```python
import sys
sys.path.append('/path/to/triad_openvr')
import triad_openvr
```

## 버전 관리 권장사항

1. **단일 소스 원칙**: 가능하면 하나의 버전만 유지
2. **심볼릭 링크 사용**: 복사본 대신 심볼릭 링크 고려
3. **명확한 네이밍**: 버전 차이가 있다면 파일명에 표시
4. **문서화**: 각 버전의 용도와 차이점 명시

## 마이그레이션 가이드

### 하위 폴더 버전으로 통합
```python
# 기존 코드
import triad_openvr  # 루트 버전

# 변경 후
from triad_openvr import triad_openvr  # 하위 폴더 버전
```

### UDP 기능 분리
UDP 기능을 별도 모듈로 분리하는 것을 고려:
```python
# udp_handler.py
import triad_openvr.triad_openvr as vr
import socket
import struct

class UDPHandler:
    def __init__(self, ip="127.0.0.1", port=8051):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (ip, port)
    
    def send_pose(self, pose_data):
        message = struct.pack('7d', *pose_data)
        self.sock.sendto(message, self.address)
```

## 성능 고려사항

### 메모리 사용
- 중복 파일로 인한 추가 디스크 공간 사용
- 실행 시에는 하나의 버전만 메모리에 로드

### Import 충돌
- Python의 import 캐싱으로 인한 잠재적 문제
- 명시적 import 경로 사용 권장

### 유지보수
- 두 버전 간 동기화 필요
- 버그 수정 시 양쪽 모두 업데이트 필요