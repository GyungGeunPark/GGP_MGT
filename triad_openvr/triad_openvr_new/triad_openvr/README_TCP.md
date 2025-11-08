# Triad OpenVR TCP Communication System

UDP에서 TCP로 변환된 Triad OpenVR 통신 시스템입니다.

## 📋 변경 사항

### Python 측 변경
- **UDP → TCP 변환**: `udp_emitter.py` → `tcp_emitter.py`
- **재연결 로직 추가**: 연결이 끊어져도 자동으로 재연결 시도
- **데이터 무결성 보장**: TCP 프로토콜로 패킷 손실 방지
- **메인 실행 파일**: `triad_openvr_tcp.py` 추가

### Unity C# 측 변경
- **UDP → TCP 변환**: `udp_tracked_object.cs` → `tcp_tracked_object.cs`
- **TCP 서버 구현**: Unity가 TCP 서버로 동작, Python이 클라이언트로 연결
- **연결 상태 관리**: 연결 상태 모니터링 및 재연결 처리
- **데이터 스트림 처리**: TCP 스트림 기반 데이터 수신

## 🚀 사용 방법

### 1. Python 환경 설정

필요한 패키지 설치:
```bash
pip install openvr
pip install numpy
```

### 2. Unity 설정

1. Unity 에디터에서 `tcp_tracked_object.cs` 스크립트를 GameObject에 추가
2. Inspector에서 설정:
   - **Port**: 8051 (기본값)
   - **Auto Calibrate**: true (자동 캘리브레이션)
   - **Smooth**: true (부드러운 움직임)
   - **Flip Z Position**: true (OpenVR → Unity 좌표계 변환)

### 3. 실행 순서

#### 방법 1: Python 스크립트 직접 실행
```bash
# 기본 설정으로 실행 (250Hz, localhost:8051)
python triad_openvr_tcp.py

# 사용자 정의 설정
python triad_openvr_tcp.py -f 100 -H 192.168.1.100 -p 9000

# 도움말 보기
python triad_openvr_tcp.py --help
```

#### 방법 2: 배치 파일 사용
```bash
# Windows에서 간단 실행
run_tcp.bat
```

#### 방법 3: 실행 파일 사용 (EXE)
```bash
# 실행 파일 빌드
python build_exe.py

# 생성된 실행 파일 실행
triad_openvr_tcp.exe
```

## 📡 통신 프로토콜

### 데이터 형식
- **크기 헤더**: 4바이트 (uint32) - 데이터 크기
- **데이터 본문**: N바이트 - 7개의 double 값
  - [0-2]: X, Y, Z 위치
  - [3-6]: W, X, Y, Z 쿼터니언 회전

### TCP 통신 흐름
1. Unity가 TCP 서버 시작 (포트 8051)
2. Python 클라이언트가 서버에 연결
3. Python이 VR 트래커 데이터를 수집
4. 데이터를 TCP로 Unity에 전송
5. Unity가 데이터를 받아 GameObject 업데이트

## 🔧 주요 기능

### Python (tcp_emitter.py)
- **자동 재연결**: 연결이 끊어지면 3초마다 재연결 시도
- **타임아웃 처리**: 5초 타임아웃으로 블로킹 방지
- **실시간 상태 표시**: 전송 중인 데이터 실시간 출력
- **에러 핸들링**: 연결 오류 시 안전한 처리

### Unity (tcp_tracked_object.cs)
- **비동기 연결 대기**: 블로킹 없이 클라이언트 연결 대기
- **스트림 기반 수신**: TCP 스트림으로 안정적인 데이터 수신
- **캘리브레이션**: C키로 수동 캘리브레이션 가능
- **스무딩 & 속도 제한**: 부드러운 움직임과 속도 제한 옵션

## 📊 성능 최적화

### 권장 설정
- **주파수**: 100-250Hz (네트워크 상태에 따라 조절)
- **스무딩**: 0.1-0.3 (부드러운 움직임)
- **속도 제한**: 활성화 (급격한 움직임 방지)

### 네트워크 설정
- **로컬 통신**: 127.0.0.1 사용 (최소 지연)
- **네트워크 통신**: 방화벽에서 포트 8051 허용 필요

## 🐛 문제 해결

### Python 실행 오류
```bash
# openvr 모듈 없음
pip install openvr

# VR 시스템 찾을 수 없음
# → SteamVR이 실행 중인지 확인
# → 트래커가 연결되어 있는지 확인
```

### Unity 연결 오류
- 포트가 이미 사용 중인지 확인
- 방화벽 설정 확인
- Python 스크립트가 올바른 IP/포트로 연결하는지 확인

### 데이터 수신 안 됨
- tracker_1이 VR 시스템에서 인식되는지 확인
- Python 콘솔에서 데이터 전송 로그 확인
- Unity 콘솔에서 연결 상태 로그 확인

## 📝 명령줄 옵션

```
usage: triad_openvr_tcp.py [-h] [-f FREQUENCY] [-H HOST] [-p PORT] [--test] [-v]

옵션:
  -h, --help            도움말 표시
  -f, --frequency       업데이트 주파수 (Hz, 기본값: 250)
  -H, --host           TCP 서버 호스트/IP (기본값: 127.0.0.1)
  -p, --port           TCP 서버 포트 (기본값: 8051)
  --test               VR 연결만 테스트
  -v, --verbose        상세 출력 활성화
```

## 📦 파일 구조

```
triad_openvr/
├── triad_openvr.py          # 기존 OpenVR 라이브러리
├── tcp_emitter.py           # TCP 클라이언트 (Python)
├── triad_openvr_tcp.py      # 메인 실행 파일
├── tcp_tracked_object.cs    # TCP 서버 (Unity)
├── run_tcp.bat              # Windows 실행 배치 파일
├── build_exe.py             # 실행 파일 빌드 스크립트
└── README_TCP.md            # 이 문서
```

## ⚡ UDP vs TCP 비교

| 특성 | UDP (이전) | TCP (현재) |
|-----|-----------|-----------|
| 연결 | 비연결형 | 연결형 |
| 신뢰성 | 패킷 손실 가능 | 패킷 손실 없음 |
| 순서 보장 | 보장 안 됨 | 순서 보장 |
| 지연 시간 | 낮음 | 약간 높음 |
| 재연결 | 수동 처리 필요 | 자동 재연결 |

## 📄 라이선스

원본 Triad OpenVR 라이선스를 따릅니다.