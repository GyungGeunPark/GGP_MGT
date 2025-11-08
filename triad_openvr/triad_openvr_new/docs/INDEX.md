# Triad OpenVR 프로젝트 문서 인덱스

## 프로젝트 개요
Triad OpenVR은 OpenVR API를 Python에서 쉽게 사용할 수 있도록 만든 라이브러리입니다. VR 추적 장치(트래커, 컨트롤러, HMD)의 위치와 회전 데이터를 실시간으로 읽고, 네트워크를 통해 다른 애플리케이션과 공유할 수 있습니다.

## 주요 특징
- 🎮 **다양한 VR 장치 지원**: HTC Vive, Valve Index, Oculus (SteamVR 모드)
- 📊 **실시간 추적**: 최대 250Hz 샘플링 속도
- 🔄 **다양한 데이터 형식**: 오일러 각도, 쿼터니언, 변환 행렬
- 🌐 **네트워크 통신**: UDP를 통한 실시간 데이터 전송
- 🎯 **Unity/Unreal 통합**: 게임 엔진과의 쉬운 연동

## 문서 구조

### 📚 [핵심 모듈 문서](./triad_openvr_core.md)
`triad_openvr.py` - 메인 라이브러리 모듈
- **클래스**: `triad_openvr`, `vr_tracked_device`, `vr_tracking_reference`, `pose_sample_buffer`
- **기능**: 장치 관리, 포즈 추출, 컨트롤러 입력, 햅틱 피드백
- **데이터 변환**: 오일러 각도, 쿼터니언 변환 함수

### 🧪 [테스트 스크립트 문서](./test_scripts.md)
테스트 및 검증 스크립트들
- **test_setup.py**: 환경 설정 검증, 의존성 확인
- **controller_test.py**: 컨트롤러 추적 테스트
- **tracker_test.py**: 트래커 추적 테스트

### 🌐 [UDP Emitter 문서](./udp_emitter.md)
`udp_emitter.py` - 네트워크 데이터 전송
- **프로토콜**: UDP 소켓 통신
- **데이터 형식**: 7개의 double 값 (위치 + 쿼터니언)
- **통합 예시**: Unity, Python 수신 코드

### 📁 [루트 디렉토리 스크립트 문서](./root_directory_scripts.md)
루트 디렉토리의 확장 버전 파일들
- **triad_openvr.py (확장판)**: UDP 전송 기능이 통합된 버전
- **복사본 파일들**: 편의를 위한 스크립트 복사본

## 빠른 시작 가이드

### 1. 환경 설정
```bash
# 환경 테스트
python test_setup.py

# 필요한 패키지 설치
pip install pyopenvr numpy matplotlib
```

### 2. 기본 사용법
```python
import triad_openvr

# 시스템 초기화
v = triad_openvr.triad_openvr()
v.print_discovered_objects()

# 트래커 데이터 읽기
pose = v.devices["tracker_1"].get_pose_euler()
if pose:
    x, y, z, yaw, pitch, roll = pose
    print(f"Position: ({x}, {y}, {z})")
```

### 3. UDP 데이터 전송
```bash
# 트래커 데이터를 UDP로 전송 (90Hz)
python triad_openvr.py

# 또는 사용자 정의 속도
python udp_emitter.py 120
```

## 파일 구조 다이어그램

```
triad_openvr/
│
├── 📄 triad_openvr.py (UDP 기능 포함)
├── 📄 controller_test.py
├── 📄 tracker_test.py
├── 📄 udp_emitter.py
├── 📄 test_setup.py
│
├── 📁 triad_openvr/
│   ├── 📄 triad_openvr.py (원본 코어)
│   ├── 📄 controller_test.py
│   ├── 📄 tracker_test.py
│   └── 📄 udp_emitter.py
│
└── 📁 docs/
    ├── 📄 INDEX.md (현재 문서)
    ├── 📄 triad_openvr_core.md
    ├── 📄 test_scripts.md
    ├── 📄 udp_emitter.md
    └── 📄 root_directory_scripts.md
```

## 주요 클래스 및 함수 요약

### 클래스 계층 구조
```
triad_openvr (메인 시스템 관리)
    ├── vr_tracked_device (일반 추적 장치)
    │   ├── get_pose_euler()
    │   ├── get_pose_quaternion()
    │   ├── get_controller_inputs()
    │   └── trigger_haptic_pulse()
    │
    └── vr_tracking_reference (베이스 스테이션)
        └── get_mode()
```

### 핵심 함수
| 함수명 | 용도 | 반환값 |
|--------|------|--------|
| `convert_to_euler()` | 행렬 → 오일러 각도 | [x, y, z, yaw, pitch, roll] |
| `convert_to_quaternion()` | 행렬 → 쿼터니언 | [x, y, z, w, rx, ry, rz] |
| `get_pose()` | 모든 장치 포즈 | 포즈 배열 |
| `poll_vr_events()` | 이벤트 모니터링 | 없음 |

## 일반적인 사용 시나리오

### 1. VR 추적 시스템
- 풀바디 모션 캡처
- 가상 카메라 추적
- 객체 위치 추적

### 2. 게임 개발
- Unity/Unreal Engine 통합
- 실시간 캐릭터 애니메이션
- VR 상호작용 시스템

### 3. 연구 및 개발
- 움직임 분석
- 공간 인터페이스 연구
- VR/AR 프로토타이핑

### 4. 산업 응용
- 가상 훈련 시스템
- 원격 조작 인터페이스
- 3D 시각화 도구

## 성능 지표

| 항목 | 값 | 비고 |
|------|-----|------|
| 최대 샘플링 속도 | 250Hz | 하드웨어 의존 |
| 권장 샘플링 속도 | 90Hz | VIVE 기본값 |
| UDP 패킷 크기 | 56 bytes | 7 doubles |
| 네트워크 대역폭 | ~20 KB/s | 250Hz 기준 |
| CPU 사용률 | 1-2% | 250Hz 기준 |
| 지연시간 | <1ms | 로컬 네트워크 |

## 문제 해결 가이드

### 자주 발생하는 문제
1. **"No module named 'openvr'"**
   - 해결: `pip install pyopenvr`

2. **SteamVR 연결 실패**
   - SteamVR 실행 확인
   - HMD 연결 상태 확인
   - USB/DisplayPort 케이블 확인

3. **트래킹 손실**
   - 베이스 스테이션 위치 확인
   - 반사 표면 제거
   - 조명 간섭 확인

4. **UDP 데이터 수신 안됨**
   - 방화벽 설정 확인
   - IP/포트 설정 확인
   - 네트워크 연결 상태 확인

## 개발 로드맵

### 현재 기능
- ✅ 기본 추적 기능
- ✅ UDP 네트워크 전송
- ✅ 컨트롤러 입력 처리
- ✅ 다양한 데이터 형식 지원

### 계획된 기능
- ⏳ TCP 통신 지원
- ⏳ 웹소켓 인터페이스
- ⏳ 데이터 로깅 시스템
- ⏳ GUI 모니터링 도구
- ⏳ 자동 캘리브레이션

## 라이선스 및 기여

이 프로젝트는 오픈소스 프로젝트입니다. 기여를 환영합니다!

### 기여 방법
1. 이슈 리포트
2. 풀 리퀘스트
3. 문서 개선
4. 예제 코드 추가

## 참고 자료

### 공식 문서
- [OpenVR API Documentation](https://github.com/ValveSoftware/openvr/wiki/API-Documentation)
- [SteamVR Developer Resources](https://developer.valvesoftware.com/wiki/SteamVR)

### 관련 프로젝트
- [PyOpenVR](https://github.com/cmbruns/pyopenvr)
- [OpenVR SDK](https://github.com/ValveSoftware/openvr)

### 커뮤니티
- [SteamVR Community](https://steamcommunity.com/app/250820/discussions/)
- [Reddit r/SteamVR](https://www.reddit.com/r/SteamVR/)

---

## 문서 버전 정보
- **작성일**: 2024
- **버전**: 1.0
- **작성자**: AI Assistant
- **최종 수정**: 현재

## 연락처
프로젝트 관련 문의사항이 있으시면 GitHub 이슈를 통해 연락 주시기 바랍니다.