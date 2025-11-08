# UDP Tracked Object (C#) - 개선된 Unity VR 트래커 통합 문서

## 개요
`udp_tracked_object.cs`는 `udp_receiver.cs`의 개선된 버전으로, VR 트래커 데이터를 Unity에서 수신하여 GameObject를 제어하는 고급 기능을 제공합니다. 캘리브레이션, 스무딩, 속도 제한, 좌표계 변환 등 프로덕션 레벨의 기능들이 구현되어 있습니다.

## 주요 개선사항
- 🔒 **스레드 안전성**: lock을 통한 동기화
- 🎯 **캘리브레이션**: 수동/자동 위치 보정
- 🎨 **스무딩**: Lerp/Slerp를 통한 부드러운 움직임
- ⚡ **속도 제한**: 비정상적인 움직임 방지
- 🔄 **좌표계 변환**: OpenVR → Unity 변환
- 🔧 **오프셋 지원**: 트래커 장착 위치 보정
- 📊 **디버그 모드**: 실시간 데이터 모니터링

## 클래스 구조

```csharp
public class udp_tracked_object : MonoBehaviour
```

Unity MonoBehaviour를 상속받아 Inspector에서 설정 가능한 다양한 옵션을 제공합니다.

## 핵심 필드 분석

### 네트워크 관련
```csharp
Thread receiveThread;           // UDP 수신 스레드
UdpClient client;              // UDP 클라이언트
private double[] float_array;  // 수신 데이터 배열 (7개)
public int port = 8051;        // UDP 포트 (Inspector 노출)
```

### 동기화 및 상태 관리
```csharp
private object lockObject = new object();  // 스레드 동기화 객체
private bool isRunning = true;            // 스레드 실행 플래그
private bool isInitialized = false;       // 초기화 상태
private bool hasNewPose = false;          // 새 데이터 수신 플래그
```

### 캘리브레이션 설정
```csharp
[Header("캘리브레이션")]
public KeyCode calibrateKey = KeyCode.C;  // 캘리브레이션 키
public bool autoCalibrate = false;        // 자동 캘리브레이션
private bool isCalibrated = false;        // 캘리브레이션 완료 상태
```

**캘리브레이션 시스템:**
- 현재 트래커와 오브젝트 위치를 기준점으로 설정
- 이후 모든 움직임은 이 기준점으로부터의 상대 변화로 적용

### 오프셋 설정
```csharp
[Header("오프셋(트래커 장착 보정)")]
public Transform mountOffsetTCP;  // 트래커 장착 오프셋
```

**용도**: 트래커가 추적 대상의 중심이 아닌 곳에 부착된 경우 위치/회전 보정

### 스무딩 및 속도 제한
```csharp
[Header("스무딩/속도제한")]
public bool smooth = true;
[Range(0f, 1f)] public float posLerp = 0.1f;    // 위치 보간 계수
[Range(0f, 1f)] public float rotLerp = 0.1f;    // 회전 보간 계수
public bool limitSpeed = true;
public float maxPosSpeed = 1.0f;                // 최대 이동 속도 (m/s)
public float maxRotSpeed = 180f;                // 최대 회전 속도 (deg/s)
```

### 좌표계 변환
```csharp
[Header("좌표 변환(OpenVR → Unity)")]
public bool flipZPosition = true;       // Z축 반전
public bool invertQuaternion = false;   // 쿼터니언 반전
```

## 메서드 상세 분석

### `Start()`
```csharp
void Start() {
    float_array = new double[7];
    initialObjectPosition = this.transform.position;
    initialObjectRotation = transform.rotation;
    
    receiveThread = new Thread(new ThreadStart(ReceiveData));
    receiveThread.IsBackground = true;
    receiveThread.Start();
    
    Debug.Log("UDP 서버 시작: 포트 " + port);
    lastRotTarget = transform.rotation;
}
```

**초기화 과정:**
1. 데이터 배열 할당
2. 오브젝트의 초기 위치/회전 저장
3. UDP 수신 스레드 시작
4. 마지막 회전 목표값 초기화

### `Update()` - 메인 업데이트 루프

```csharp
void Update() {
    // 1. 캘리브레이션 키 체크
    if (Input.GetKeyDown(calibrateKey)) {
        Calibrate();
    }
    
    // 2. 데이터 동기화 및 읽기
    Vector3 trackerPosition;
    Quaternion trackerRotation;
    
    lock (lockObject) {
        if (!hasNewPose) return;
        
        // 위치 데이터 추출
        trackerPosition = new Vector3(
            (float)float_array[0],
            (float)float_array[1],
            (float)float_array[2]
        );
        
        // 쿼터니언 추출 (올바른 순서)
        trackerRotation = new Quaternion(
            (float)float_array[4],  // x
            (float)float_array[5],  // y
            (float)float_array[6],  // z
            (float)float_array[3]   // w
        );
        
        // 3. 좌표계 변환
        if (flipZPosition) {
            trackerPosition.z = -trackerPosition.z;
        }
        
        if (invertQuaternion) {
            trackerRotation = Quaternion.Inverse(trackerRotation);
        }
        
        // 4. 초기 캘리브레이션
        if (!isInitialized || (autoCalibrate && !isCalibrated)) {
            initialTrackerPosition = trackerPosition;
            initialTrackerRotation = trackerRotation;
            isInitialized = true;
            isCalibrated = true;
        }
    }
    
    // 5. 상대 변환 계산
    Vector3 positionDelta = trackerPosition - initialTrackerPosition;
    Quaternion rotationDelta = trackerRotation * Quaternion.Inverse(initialTrackerRotation);
    
    // 6. 목표 위치/회전 설정
    Vector3 targetPos = initialObjectPosition + positionDelta;
    Quaternion targetRot = initialObjectRotation * rotationDelta;
    
    // 7. 오프셋 적용
    if (mountOffsetTCP != null) {
        targetPos += targetRot * mountOffsetTCP.localPosition;
        targetRot = targetRot * mountOffsetTCP.localRotation;
    }
    
    // 8. 속도 제한
    if (limitSpeed) {
        // 위치 속도 제한
        Vector3 dp = targetPos - transform.position;
        float maxStep = maxPosSpeed * Time.deltaTime;
        if (dp.magnitude > maxStep && dp.magnitude > 0.001f) {
            targetPos = transform.position + dp.normalized * maxStep;
        }
        
        // 회전 속도 제한
        float angle;
        Vector3 axis;
        Quaternion deltaRot = targetRot * Quaternion.Inverse(lastRotTarget);
        deltaRot.ToAngleAxis(out angle, out axis);
        
        if (angle > 180f) angle = 360f - angle;
        
        float maxAngStep = maxRotSpeed * Time.deltaTime;
        if (angle > maxAngStep && angle > 0.01f) {
            targetRot = Quaternion.Slerp(lastRotTarget, targetRot, maxAngStep / angle);
        }
    }
    
    // 9. 스무딩 적용
    if (smooth) {
        transform.position = Vector3.Lerp(transform.position, targetPos, posLerp);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, rotLerp);
    } else {
        transform.position = targetPos;
        transform.rotation = targetRot;
    }
    
    lastRotTarget = targetRot;
}
```

### `Calibrate()` - 수동 캘리브레이션
```csharp
void Calibrate() {
    lock (lockObject) {
        if (hasNewPose) {
            // 현재 트래커 위치를 새로운 기준점으로 설정
            // 현재 오브젝트 위치도 새로운 기준점으로 설정
            initialTrackerPosition = trackerPosition;
            initialTrackerRotation = trackerRotation;
            initialObjectPosition = transform.position;
            initialObjectRotation = transform.rotation;
            isCalibrated = true;
            
            Debug.Log("[UDP Tracker] 수동 캘리브레이션 완료");
        }
    }
}
```

### `ReceiveData()` - UDP 수신 스레드
```csharp
private void ReceiveData() {
    try {
        client = new UdpClient(port);
        Debug.Log("Starting Server on port " + port);
        
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
        while (isRunning) {
            byte[] data = client.Receive(ref anyIP);
            
            lock (lockObject) {
                int doubleCount = data.Length / 8;
                if (doubleCount >= 7) {
                    for (int i = 0; i < 7; i++)
                        float_array[i] = BitConverter.ToDouble(data, i * 8);
                    
                    hasNewPose = true;
                }
            }
        }
    }
    catch (SocketException e) {
        if (isRunning) Debug.LogError("UDP 수신 오류: " + e);
    }
    catch (Exception err) {
        Debug.LogError("UDP 수신 오류: " + err);
    }
    finally {
        client?.Close();
        Debug.Log("UDP 서버가 종료되었습니다.");
    }
}
```

**개선사항:**
- 데이터 크기 검증
- 스레드 안전한 종료
- 예외 처리 세분화
- null 조건 연산자 사용

### `OnApplicationQuit()`
```csharp
void OnApplicationQuit() {
    isRunning = false;
    client?.Close();
}
```

안전한 종료를 위해 `Thread.Abort()` 대신 플래그 사용

## 좌표계 변환 이해

### OpenVR vs Unity 좌표계
```
OpenVR:                Unity:
  Y (up)                 Y (up)
  |                      |
  |___X (right)          |___X (right)
 /                      /
Z (backward)           Z (forward)
```

Z축 방향이 반대이므로 `flipZPosition`으로 변환합니다.

## 캘리브레이션 시스템

### 작동 원리
1. **초기 기준점 설정**: 트래커와 오브젝트의 현재 위치를 기준점으로 저장
2. **상대 변환 계산**: 이후 모든 움직임을 기준점으로부터의 변화로 계산
3. **오브젝트 업데이트**: 상대 변환을 오브젝트에 적용

### 사용 시나리오
- **자동 캘리브레이션**: 시작 시 자동으로 현재 위치를 기준점으로 설정
- **수동 캘리브레이션**: 사용자가 원하는 시점에 C키로 재설정

## 스무딩 시스템

### Lerp vs Slerp
- **Vector3.Lerp**: 위치의 선형 보간
- **Quaternion.Slerp**: 회전의 구면 선형 보간

### 매개변수 조정
- `posLerp = 0.1`: 낮을수록 부드럽지만 지연 증가
- `rotLerp = 0.1`: 높을수록 반응이 빠르지만 떨림 증가

## 속도 제한 시스템

### 위치 속도 제한
```csharp
Vector3 dp = targetPos - transform.position;
float maxStep = maxPosSpeed * Time.deltaTime;
if (dp.magnitude > maxStep) {
    targetPos = transform.position + dp.normalized * maxStep;
}
```

### 회전 속도 제한
```csharp
deltaRot.ToAngleAxis(out angle, out axis);
if (angle > maxAngStep) {
    targetRot = Quaternion.Slerp(lastRotTarget, targetRot, maxAngStep / angle);
}
```

## Unity Inspector 설정 가이드

### 권장 설정값

#### 일반 사용
```
Port: 8051
Auto Calibrate: true
Smooth: true
Pos Lerp: 0.15
Rot Lerp: 0.2
Limit Speed: true
Max Pos Speed: 2.0
Max Rot Speed: 360
Flip Z Position: true
```

#### 정밀 추적
```
Smooth: false
Limit Speed: false
Show Debug Info: true
```

#### VR 게임
```
Smooth: true
Pos Lerp: 0.3
Rot Lerp: 0.4
Max Pos Speed: 5.0
```

## 디버깅 기능

### 디버그 출력
```csharp
if (showDebugInfo && frameCounter++ % 10 == 0) {
    Debug.Log($"[UDP Tracker Debug] Pos: {trackerPosition:F3}, " +
              $"Rot: {trackerRotation.eulerAngles:F1}");
    Debug.Log($"[UDP Tracker Debug] Delta Pos: {positionDelta:F3}, " +
              $"Target: {targetPos:F3}");
}
```

10프레임마다 현재 상태를 출력하여 성능 영향 최소화

## 코드 내 주석 분석

개발자가 남긴 주석들:
```csharp
// 1. doubleCount가 7미만인경우???? 정의 안됨
// 2. doubleCount가 7인 경우 제대로 돌아감.. But 7 이상인 경우
// 3. 왜 7???(<=== magic number) x, y, z, rx, ry, rz는 총 6개임...
// 4. ToDouble의 float에 제대로 들어감?
// 5. bool isFirst 플래그로 초기값 저장 필요
// 6. 필요에 의해 isFirst를 false로 Set 하여 Calibration 필요
```

**답변:**
1. 7개 미만일 경우 무시 (현재 구현됨)
2. 7개는 위치(3) + 쿼터니언(4) = 7개 값
3. 쿼터니언은 w,x,y,z 4개 성분 필요
4. double에서 float 캐스팅은 정밀도 손실 있지만 동작
5. isCalibrated 플래그로 구현됨
6. Calibrate() 메서드로 구현됨

## 성능 최적화 제안

### 1. 오브젝트 풀링
```csharp
private Queue<PoseData> poseBuffer = new Queue<PoseData>();
// 여러 프레임 버퍼링으로 네트워크 지터 완화
```

### 2. 예측 알고리즘
```csharp
Vector3 velocity = (currentPos - lastPos) / Time.deltaTime;
Vector3 predictedPos = currentPos + velocity * latency;
```

### 3. 적응형 스무딩
```csharp
float adaptiveLerp = Mathf.Lerp(0.05f, 0.3f, 
    deltaPosition.magnitude / maxExpectedDelta);
```

## 일반적인 문제 해결

### 1. 오브젝트가 움직이지 않음
- UDP 포트 확인 (기본 8051)
- 방화벽 설정 확인
- Python 송신 측 실행 확인

### 2. 잘못된 회전
- 쿼터니언 순서 확인
- invertQuaternion 옵션 토글

### 3. 위치 오프셋
- 캘리브레이션 실행 (C키)
- mountOffsetTCP 확인

### 4. 떨림 현상
- smooth 활성화
- posLerp/rotLerp 값 조정
- 샘플링 속도 확인

## 확장 기능 제안

### 1. 멀티 트래커 지원
```csharp
public int trackerID = 0;
// 패킷에 ID 포함하여 여러 트래커 구분
```

### 2. 녹화/재생
```csharp
public class PoseRecorder : MonoBehaviour {
    List<PoseFrame> recording;
    // 포즈 데이터 녹화 및 재생
}
```

### 3. 네트워크 상태 모니터링
```csharp
public float packetLossRate;
public float averageLatency;
// 네트워크 품질 지표 추적
```

## 결론

`udp_tracked_object.cs`는 프로덕션 레벨의 VR 트래커 통합을 위한 완성도 높은 구현입니다:

### 강점
✅ 스레드 안전한 구현
✅ 유연한 캘리브레이션 시스템
✅ 다양한 스무딩 옵션
✅ 속도 제한으로 안정성 확보
✅ Unity Inspector 통합
✅ 디버깅 기능 내장

### 개선 가능 영역
⚠️ 네트워크 지연 보상
⚠️ 패킷 손실 처리
⚠️ 멀티 트래커 지원
⚠️ 성능 프로파일링

이 스크립트는 대부분의 VR 트래킹 시나리오에서 즉시 사용 가능한 솔루션을 제공합니다.