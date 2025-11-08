# 로테이션 축 매핑 수정 문서

## 문제 설명

Unity 에디터에서 VR 트래커 테스트 중 발견된 회전 축 매핑 오류:
- **Pitch (X축 회전)**가 → **Roll (Z축 회전)**로 동작
- **Yaw (Y축 회전)**가 → **Pitch (X축 회전)**로 동작  
- **Roll (Z축 회전)**이 → **Yaw (Y축 회전)**로 동작

## 원인 분석

### OpenVR vs Unity 좌표계 차이
- **OpenVR**: 오른손 좌표계 (Right-handed)
- **Unity**: 왼손 좌표계 (Left-handed)

### 회전 순서 차이
- **OpenVR**: 회전 행렬에서 추출한 순서 (Pitch, Yaw, Roll)
- **Unity**: Quaternion.Euler(x, y, z) 기대 순서

테스트 결과 축이 순환적으로 한 칸씩 밀려있음을 확인

## 해결 방법

### 1. Python 측 수정 (triad_openvr.py)

**convert_to_euler 함수 반환 순서 변경:**

```python
# 기존 (잘못된 매핑)
return [x, y, z, pitch, yaw, roll]

# 수정 (올바른 매핑) 
return [x, y, z, roll, pitch, yaw]
```

**매핑 설명:**
- Python `roll` → Unity X축 회전 (Pitch)
- Python `pitch` → Unity Y축 회전 (Yaw)  
- Python `yaw` → Unity Z축 회전 (Roll)

### 2. Unity 측 수정 (tcp_tracked_object.cs)

**데이터 해석 변경:**

```csharp
// 수정된 순서로 받은 데이터 해석
float unityPitch = (float)float_array[3];  // OpenVR roll → Unity X축
float unityYaw = (float)float_array[4];    // OpenVR pitch → Unity Y축
float unityRoll = (float)float_array[5];   // OpenVR yaw → Unity Z축

// Unity Quaternion 생성
trackerRotation = Quaternion.Euler(unityPitch, unityYaw, unityRoll);
```

### 3. TCP 통신 디버그 출력 개선

**tcp_emitter.py:**
```python
print(f"\rSent: Pos({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f}) " +
      f"UnityRot(X:{data[3]:.1f}° Y:{data[4]:.1f}° Z:{data[5]:.1f}°)")
```

## 실행 방법

### 1. Python 서버 실행
```bash
# run_all_in_one.bat 실행 (가상환경 자동 활성화)
run_all_in_one.bat

# 또는 직접 실행
python ./triad_openvr/triad_openvr_tcp.py
```

### 2. Unity 설정
1. `tcp_tracked_object.cs` 스크립트를 GameObject에 연결
2. Inspector 설정:
   - Port: 8051 (기본값)
   - Show Debug Info: true (디버깅용)
   - flipZPosition: false (Python에서 처리)
   - invertQuaternion: false (일반적으로 불필요)
   - apply30DegreeCorrection: 필요시 true

## 테스트 방법

### 1. 축별 회전 테스트
각 축을 개별적으로 회전시켜 올바른 매핑 확인:

| 트래커 동작 | Unity에서 예상 결과 |
|------------|-------------------|
| X축 중심 회전 (앞뒤 기울임) | Pitch - X축 회전 |
| Y축 중심 회전 (좌우 회전) | Yaw - Y축 회전 |
| Z축 중심 회전 (좌우 기울임) | Roll - Z축 회전 |

### 2. 디버그 로그 확인
Unity Console에서:
```
[TCP Tracker Debug] Received Values: [값1°, 값2°, 값3°]
[TCP Tracker Debug] Pos: (x, y, z), Final Euler: (x°, y°, z°)
```

### 3. 테스트 스크립트 사용
```bash
python test_euler_transmission.py
# 옵션 2: Test rotation 선택
```

## 추가 보정

### 30도 오프셋이 여전히 있는 경우
Unity Inspector에서:
1. `apply30DegreeCorrection` = true
2. `rotationOffsetY` 값 조정 (기본값: -30)

### 축 반전이 필요한 경우
triad_openvr.py의 convert_to_euler 함수에서:
```python
# 필요시 주석 해제
# yaw = -yaw  # Y축 반전
# pitch = -pitch  # X축 반전
# roll = -roll  # Z축 반전
```

## 파일 구조

```
triad_openvr_new/
├── run_all_in_one.bat          # 메인 실행 파일
├── triad_openvr/
│   ├── triad_openvr.py         # 핵심 VR 라이브러리 (수정됨)
│   ├── triad_openvr_tcp.py     # TCP 서버 메인
│   ├── tcp_emitter.py          # TCP 전송 모듈 (수정됨)
│   ├── tcp_tracked_object.cs   # Unity 수신 스크립트 (수정됨)
│   └── test_euler_transmission.py # 테스트 도구
└── docs/
    ├── VR_Tracking_Coordinate_System_Fix_Guide.md
    └── Rotation_Axis_Fix_Documentation.md (이 문서)
```

## 요약

1. **문제**: 회전 축이 순환적으로 잘못 매핑됨
2. **원인**: OpenVR과 Unity 좌표계 차이 및 오일러 각도 순서
3. **해결**: Python에서 전송 순서를 [roll, pitch, yaw]로 변경
4. **결과**: Unity에서 올바른 X, Y, Z 축 회전 동작

이제 VR 트래커의 회전이 Unity에서 정확하게 표현됩니다.