# triad_openvr.py - 핵심 모듈 문서

## 개요
`triad_openvr.py`는 OpenVR 기반 VR 추적 장치(트래커, 컨트롤러, HMD 등)와 상호작용하기 위한 Python 라이브러리입니다. 이 모듈은 OpenVR API를 래핑하여 더 사용하기 쉬운 인터페이스를 제공합니다.

## 주요 기능
- VR 장치 자동 탐지 및 관리
- 포즈 데이터 추출 (위치, 회전)
- 다양한 회전 표현 방식 지원 (오일러 각도, 쿼터니언)
- 컨트롤러 입력 처리
- UDP 네트워크 통신
- 실시간 장치 이벤트 모니터링

## 유틸리티 함수

### `update_text(txt)`
현재 줄을 덮어쓰는 방식으로 텍스트를 출력합니다.
- **매개변수**: `txt` - 출력할 텍스트
- **용도**: 실시간 데이터를 같은 줄에 계속 업데이트할 때 사용

### `convert_to_euler(pose_mat)`
3x4 변환 행렬을 위치(x,y,z)와 오일러 각도(yaw, pitch, roll)로 변환합니다.
- **매개변수**: `pose_mat` - 3x4 변환 행렬
- **반환값**: `[x, y, z, yaw, pitch, roll]` 리스트 (각도는 도 단위)
- **참고**: 오일러 각도는 짐벌 락(Gimbal Lock) 문제가 있을 수 있음

### `convert_to_quaternion(pose_mat)`
3x4 변환 행렬을 위치(x,y,z)와 쿼터니언(w,x,y,z)으로 변환합니다.
- **매개변수**: `pose_mat` - 3x4 변환 행렬
- **반환값**: `[x, y, z, r_w, r_x, r_y, r_z]` 리스트
- **특징**: Issue #2 수정 - `abs()` 함수로 음수 제곱근 방지

### `get_pose(vr_obj)`
모든 추적 장치의 현재 포즈를 가져옵니다.
- **매개변수**: `vr_obj` - OpenVR 시스템 객체
- **반환값**: 모든 장치의 포즈 배열

## 클래스

### `pose_sample_buffer`
포즈 데이터를 시간에 따라 기록하고 분석하기 위한 버퍼 클래스입니다.

#### 속성
- `time`: 타임스탬프 리스트
- `x, y, z`: 위치 좌표 리스트
- `yaw, pitch, roll`: 오일러 각도 리스트
- `r_w, r_x, r_y, r_z`: 쿼터니언 성분 리스트

#### 메서드
- `append(pose_mat, t)`: 새로운 포즈 샘플을 버퍼에 추가

### `vr_tracked_device`
VR 추적 장치를 나타내는 기본 클래스입니다.

#### 속성
- `device_class`: 장치 유형 ("Controller", "HMD", "Tracker" 등)
- `index`: OpenVR 장치 인덱스
- `vr`: OpenVR 시스템 참조

#### 주요 메서드

##### 장치 정보
- `get_serial()`: 장치 시리얼 번호 반환 (캐싱됨)
- `get_model()`: 장치 모델명 반환
- `get_battery_percent()`: 배터리 잔량 퍼센트 반환
- `is_charging()`: 충전 중 여부 반환

##### 포즈 데이터
- `get_pose_euler(pose=None)`: 오일러 각도 형식의 포즈 반환
  - 반환값: `[x, y, z, yaw, pitch, roll]` 또는 `None`
- `get_pose_quaternion(pose=None)`: 쿼터니언 형식의 포즈 반환
  - 반환값: `[x, y, z, w, x, y, z]` 또는 `None`
- `get_pose_matrix(pose=None)`: 3x4 변환 행렬 반환
- `get_velocity(pose=None)`: 선속도 벡터 반환
- `get_angular_velocity(pose=None)`: 각속도 벡터 반환

##### 샘플링
- `sample(num_samples, sample_rate)`: 지정된 속도로 포즈 데이터 샘플링
  - `num_samples`: 샘플 개수
  - `sample_rate`: 초당 샘플 수 (Hz)
  - 반환값: `pose_sample_buffer` 객체

##### 컨트롤러 전용
- `controller_state_to_dict(pControllerState)`: 컨트롤러 상태를 딕셔너리로 변환
  - 트리거 값, 트랙패드 좌표, 버튼 상태 등 포함
- `get_controller_inputs()`: 현재 컨트롤러 입력 상태 반환
- `trigger_haptic_pulse(duration_micros, axis_id)`: 햅틱 진동 트리거

### `vr_tracking_reference`
베이스 스테이션(라이트하우스) 추적 참조점을 나타내는 클래스입니다.

#### 추가 메서드
- `get_mode()`: 추적 모드 반환 (예: "A", "B", "C")
- `sample()`: 경고 메시지 출력 (베이스 스테이션은 움직이지 않음)

### `triad_openvr`
OpenVR 시스템 전체를 관리하는 메인 클래스입니다.

#### 초기화
```python
def __init__(self, configfile_path=None)
```
- `configfile_path`: 선택적 JSON 설정 파일 경로
- 설정 파일이 있으면 시리얼 번호로 장치를 매핑
- 없으면 자동으로 장치 탐지 및 번호 부여

#### 속성
- `vr`: OpenVR 시스템 객체
- `vrsystem`: VRSystem 인터페이스
- `object_names`: 장치 유형별 이름 딕셔너리
- `devices`: 장치 이름을 키로 하는 장치 객체 딕셔너리
- `device_index_map`: 인덱스를 장치 이름에 매핑

#### 주요 메서드

##### 시스템 관리
- `get_pose()`: 모든 장치의 현재 포즈 반환
- `poll_vr_events()`: VR 이벤트 폴링 (장치 연결/해제 감지)
- `print_discovered_objects()`: 발견된 모든 장치 정보 출력

##### 장치 관리
- `add_tracked_device(tracked_device_index)`: 새 장치 추가
  - 장치 유형에 따라 자동으로 이름 생성 (예: "controller_1", "tracker_1")
- `remove_tracked_device(tracked_device_index)`: 장치 제거
- `rename_device(old_name, new_name)`: 장치 이름 변경

## UDP 통신 기능 (추가 코드)

### 설정
```python
UDP_IP = "127.0.0.1"  # Unity 등 수신 측 IP
UDP_PORT = 8051       # 수신 포트
```

### `main()` 함수
트래커의 쿼터니언 데이터를 UDP로 전송하는 메인 루프입니다.

#### 동작 과정
1. OpenVR 시스템 초기화
2. 트래커 장치 검색
3. 90Hz 주기로 포즈 데이터 전송
4. 데이터 형식: 7개의 double 값 (x, y, z, w, x, y, z)

#### 특징
- struct.pack('7d', ...)로 바이너리 패킹
- 실시간 이벤트 폴링으로 장치 변경 감지
- KeyboardInterrupt 처리로 안전한 종료

## 설정 파일 형식

JSON 설정 파일 예시:
```json
{
  "devices": [
    {
      "name": "tracker1",
      "type": "Tracker",
      "serial": "LHR-XXXXXXXX"
    },
    {
      "name": "left_controller",
      "type": "Controller",
      "serial": "LHR-YYYYYYYY"
    }
  ]
}
```

## 사용 예시

### 기본 사용
```python
import triad_openvr

# 시스템 초기화
v = triad_openvr.triad_openvr()
v.print_discovered_objects()

# 트래커 데이터 읽기
if "tracker_1" in v.devices:
    pose = v.devices["tracker_1"].get_pose_euler()
    if pose:
        x, y, z, yaw, pitch, roll = pose
        print(f"Position: ({x:.3f}, {y:.3f}, {z:.3f})")
```

### 컨트롤러 입력 처리
```python
controller = v.devices["controller_1"]
inputs = controller.get_controller_inputs()

if inputs["trigger"] > 0.5:
    print("Trigger pressed!")
    controller.trigger_haptic_pulse(1000)  # 1ms 진동
```

### 실시간 모니터링
```python
while True:
    v.poll_vr_events()  # 장치 변경 감지
    
    for device_name in v.devices:
        device = v.devices[device_name]
        pose = device.get_pose_quaternion()
        if pose:
            print(f"{device_name}: {pose}")
    
    time.sleep(0.01)  # 100Hz
```

## 주의사항

1. **SteamVR 필요**: 이 라이브러리는 SteamVR이 실행 중이어야 작동합니다.
2. **좌표계**: OpenVR은 Y축이 위쪽인 우수 좌표계를 사용합니다.
3. **단위**: 위치는 미터 단위, 각도는 도(degree) 단위입니다.
4. **성능**: 높은 샘플링 속도(>250Hz)는 시스템 부하를 일으킬 수 있습니다.
5. **오류 처리**: 포즈가 유효하지 않을 때 None을 반환하므로 항상 확인 필요

## 버전 정보
- OpenVR API 래퍼
- Python 3.x 호환
- 주요 지원 장치: HTC Vive, Valve Index, Oculus (SteamVR 모드)