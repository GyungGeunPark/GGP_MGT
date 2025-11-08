# VR 트래킹 좌표계 문제 해결 가이드

## 문제 분석

### 1. 현재 시스템 구조
- **OpenVR (Python)**: VR 트래커로부터 위치와 회전 데이터 수집
- **TCP 통신**: Python에서 Unity로 데이터 전송
- **Unity (C#)**: 수신한 데이터로 오브젝트 변환 적용

### 2. 발견된 문제점

#### 2.1 30도 회전 오프셋 문제
**증상**: Unity 에디터에서 트래커를 움직일 때 Z축과 X축 이동이 약 30도 시계방향으로 틀어짐

**원인 분석**:
1. **쿼터니언 순서 불일치**
   - Python 전송: `[x, y, z, w, r_x, r_y, r_z]`
   - Unity 수신: `float_array[3]=w, [4]=x, [5]=y, [6]=z`
   - Unity Quaternion 생성: `new Quaternion(z, y, x, w)` - 잘못된 순서

2. **좌표계 차이**
   - OpenVR: 오른손 좌표계 (Y=위, Z=뒤, X=오른쪽)
   - Unity: 왼손 좌표계 (Y=위, Z=앞, X=오른쪽)

3. **불완전한 좌표계 변환**
   - 현재 Z축만 반전 (`flipZPosition=true`)
   - 회전 변환이 불완전함

#### 2.2 쿼터니언 대신 오일러 각도 필요
- Unity에서 `r_w` 값이 불필요
- 오일러 각도가 더 직관적이고 디버깅 용이

## 해결 방안

### 방법 1: 오일러 각도 전송으로 변경 (권장)

#### Python 측 수정 사항

**1. triad_openvr.py의 convert_to_euler 함수 개선**
```python
def convert_to_euler(pose_mat):
    """
    OpenVR 행렬을 Unity 호환 오일러 각도로 변환
    Unity는 ZXY 회전 순서 사용
    """
    import math
    
    # 위치 추출
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    
    # 회전 행렬에서 오일러 각도 추출 (ZXY 순서)
    # 참고: Unity의 Quaternion.eulerAngles는 ZXY 순서 사용
    
    # Pitch (X축 회전)
    sin_pitch = -pose_mat[2][1]
    pitch = math.asin(max(-1, min(1, sin_pitch)))
    
    # Yaw (Y축 회전) 및 Roll (Z축 회전)
    if math.cos(pitch) > 0.001:  # Gimbal lock 체크
        yaw = math.atan2(pose_mat[2][0], pose_mat[2][2])
        roll = math.atan2(pose_mat[0][1], pose_mat[1][1])
    else:
        # Gimbal lock 상황
        yaw = math.atan2(-pose_mat[0][2], pose_mat[0][0])
        roll = 0
    
    # 라디안을 도(degree)로 변환
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    roll = math.degrees(roll)
    
    # OpenVR to Unity 좌표계 변환
    # 위치: Z축 반전
    z = -z
    
    # 회전: Y축 180도 회전 보정 (필요시)
    # yaw = -yaw  # 필요한 경우 Y축 회전 반전
    
    return [x, y, z, pitch, yaw, roll]
```

**2. tcp_emitter.py 수정**
```python
# 기존 코드 (라인 72-73)
data = self.v.devices["tracker_1"].get_pose_quaternion()

# 변경 후
data = self.v.devices["tracker_1"].get_pose_euler()
```

#### Unity 측 수정 사항

**tcp_tracked_object.cs 수정**

**1. float_array 크기 변경 (라인 64)**
```csharp
// 기존
float_array = new double[7];  // 쿼터니언용

// 변경 후
float_array = new double[6];  // 오일러 각도용 (x, y, z, pitch, yaw, roll)
```

**2. Update 메서드의 회전 처리 부분 수정 (라인 106-130)**
```csharp
// 위치 데이터 (변경 없음)
trackerPosition = new Vector3(
    (float)float_array[0],
    (float)float_array[1],
    (float)float_array[2]
);

// 오일러 각도에서 쿼터니언 생성
float pitch = (float)float_array[3];  // X축 회전
float yaw = (float)float_array[4];    // Y축 회전
float roll = (float)float_array[5];   // Z축 회전

// Unity의 Quaternion.Euler 사용 (ZXY 순서)
trackerRotation = Quaternion.Euler(pitch, yaw, roll);

// 30도 오프셋 보정 (필요한 경우)
// 만약 여전히 30도 오프셋이 있다면:
// trackerRotation = Quaternion.Euler(0, -30, 0) * trackerRotation;
```

**3. 데이터 수신 부분 수정 (라인 360-367)**
```csharp
int doubleCount = dataSize / 8;
if (doubleCount >= 6)  // 7개 대신 6개 확인
{
    for (int i = 0; i < 6; i++)  // 6개만 읽기
    {
        float_array[i] = BitConverter.ToDouble(dataBuffer, i * 8);
    }
    hasNewPose = true;
}
```

### 방법 2: 쿼터니언 순서 수정 (대안)

쿼터니언을 계속 사용하고 싶다면:

**Unity 측 수정 (라인 113-118)**
```csharp
// 올바른 쿼터니언 순서로 수정
trackerRotation = new Quaternion(
    (float)float_array[4],  // x (r_x)
    (float)float_array[5],  // y (r_y)
    (float)float_array[6],  // z (r_z)
    (float)float_array[3]   // w (r_w)
);
```

### 30도 오프셋 추가 보정

만약 위 수정 후에도 30도 오프셋이 남아있다면:

**1. 캘리브레이션 시 보정값 적용**
```csharp
// Calibrate 메서드에 추가
private Quaternion rotationOffset = Quaternion.Euler(0, -30, 0);

// Update 메서드에서 적용
targetRot = initialObjectRotation * rotationOffset * rotationDelta;
```

**2. 트래커 물리적 장착 각도 확인**
- 트래커가 물리적으로 30도 틀어져 장착되었는지 확인
- mountOffsetTransform을 사용하여 보정

## 테스트 및 검증

### 1단계: 기본 동작 확인
1. 수정한 코드 적용
2. 트래커를 X, Y, Z 각 축으로만 이동
3. Unity에서 올바른 방향으로 이동하는지 확인

### 2단계: 회전 확인
1. 트래커를 각 축 중심으로 회전
2. Unity에서 동일한 회전이 적용되는지 확인

### 3단계: 30도 오프셋 확인
1. 트래커를 정면(Z축)으로 이동
2. Unity에서 정확히 Z축 방향으로 이동하는지 확인
3. 오프셋이 있다면 보정값 적용

### 디버깅 팁

**Python 측 디버깅**
```python
# 전송 데이터 확인
print(f"Sending: Pos({x:.2f}, {y:.2f}, {z:.2f}) Rot({pitch:.1f}, {yaw:.1f}, {roll:.1f})")
```

**Unity 측 디버깅**
```csharp
// showDebugInfo = true로 설정하여 수신 데이터 확인
Debug.Log($"Received: Pos({trackerPosition}) Euler({pitch}, {yaw}, {roll})");
```

## 추가 개선 사항

### 1. 좌표계 변환 행렬 사용
더 정확한 변환을 위해 4x4 변환 행렬 사용:
```python
def openvr_to_unity_matrix():
    """OpenVR to Unity 좌표계 변환 행렬"""
    return np.array([
        [1,  0,  0, 0],
        [0,  1,  0, 0],
        [0,  0, -1, 0],  # Z축 반전
        [0,  0,  0, 1]
    ])
```

### 2. 동적 캘리브레이션
- 런타임에 오프셋 각도를 조정할 수 있는 UI 추가
- 캘리브레이션 데이터를 파일로 저장/로드

### 3. 성능 최적화
- 불필요한 좌표계 변환 연산 최소화
- 데이터 전송 빈도 최적화 (필요시 낮춤)

## 요약

1. **주요 원인**: 쿼터니언 순서 오류와 불완전한 좌표계 변환
2. **권장 해결책**: 오일러 각도 전송으로 변경
3. **테스트 필수**: 각 축별로 이동/회전 테스트하여 검증
4. **추가 보정**: 필요시 30도 오프셋 보정값 적용

이 가이드를 따라 수정하면 VR 트래킹 좌표계 문제를 해결할 수 있습니다.