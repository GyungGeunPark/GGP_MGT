# UDP Emitter 문서

## 개요
`udp_emitter.py`는 VR 트래커의 포즈 데이터를 UDP 네트워크 프로토콜을 통해 실시간으로 전송하는 스크립트입니다. 주로 Unity, Unreal Engine 등의 게임 엔진이나 다른 애플리케이션과 VR 추적 데이터를 공유할 때 사용됩니다.

## 주요 기능
- 트래커의 쿼터니언 포즈 데이터 추출
- UDP 소켓을 통한 실시간 데이터 전송
- 바이너리 형식으로 효율적인 데이터 패킹
- 250Hz 기본 샘플링 속도 지원

## 네트워크 설정

### 기본 설정
```python
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('10.0.1.48', 8051)
```

- **프로토콜**: UDP (SOCK_DGRAM)
- **대상 IP**: 10.0.1.48 (수신 측 컴퓨터)
- **포트**: 8051
- **특징**: 연결 없는 프로토콜로 낮은 지연시간

## 데이터 형식

### 전송 데이터 구조
```python
data = v.devices["tracker_1"].get_pose_quaternion()
# 반환값: [x, y, z, w, rx, ry, rz]
```

**데이터 구성 (7개의 double 값):**
1. `x`: X축 위치 (미터)
2. `y`: Y축 위치 (미터)
3. `z`: Z축 위치 (미터)
4. `w`: 쿼터니언 W 성분 (실수부)
5. `rx`: 쿼터니언 X 성분
6. `ry`: 쿼터니언 Y 성분
7. `rz`: 쿼터니언 Z 성분

### 바이너리 패킹
```python
sent = sock.sendto(struct.pack('d'*len(data), *data), server_address)
```

- **형식**: IEEE 754 double precision (64비트)
- **패킹 문자열**: 'd' * 7 = 'ddddddd'
- **총 크기**: 56 바이트 (8 바이트 × 7)
- **바이트 순서**: 시스템 기본 (보통 Little Endian)

## 동작 흐름

### 1. 초기화 단계
```python
v = triad_openvr.triad_openvr()
v.print_discovered_objects()
```
- OpenVR 시스템 초기화
- 연결된 VR 장치 탐색 및 출력

### 2. 샘플링 속도 설정
```python
if len(sys.argv) == 1:
    interval = 1/250  # 기본 250Hz
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])  # 사용자 정의
```

### 3. 메인 전송 루프
```python
while(True):
    start = time.time()
    txt = ""
    data = v.devices["tracker_1"].get_pose_quaternion()
    sent = sock.sendto(struct.pack('d'*len(data), *data), server_address)
    print("\r" + txt, end="")
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)
```

**루프 동작:**
1. 시작 시간 기록
2. 트래커 포즈 데이터 읽기
3. UDP 패킷 전송
4. 남은 시간만큼 대기 (정확한 타이밍 유지)

## 사용 방법

### 기본 실행 (250Hz)
```bash
python udp_emitter.py
```

### 사용자 정의 속도 (90Hz)
```bash
python udp_emitter.py 90
```

## 수신 측 구현 예시

### Python 수신 코드
```python
import socket
import struct

UDP_IP = "10.0.1.48"
UDP_PORT = 8051

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(56)  # 7 doubles = 56 bytes
    pose = struct.unpack('7d', data)
    x, y, z, w, rx, ry, rz = pose
    print(f"Position: ({x:.3f}, {y:.3f}, {z:.3f})")
    print(f"Rotation: ({w:.3f}, {rx:.3f}, {ry:.3f}, {rz:.3f})")
```

### Unity C# 수신 코드
```csharp
using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

public class UDPReceiver : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint endPoint;
    
    void Start()
    {
        udpClient = new UdpClient(8051);
        endPoint = new IPEndPoint(IPAddress.Any, 8051);
        udpClient.BeginReceive(ReceiveCallback, null);
    }
    
    void ReceiveCallback(IAsyncResult result)
    {
        byte[] data = udpClient.EndReceive(result, ref endPoint);
        
        if (data.Length == 56)  // 7 doubles
        {
            double x = BitConverter.ToDouble(data, 0);
            double y = BitConverter.ToDouble(data, 8);
            double z = BitConverter.ToDouble(data, 16);
            double w = BitConverter.ToDouble(data, 24);
            double rx = BitConverter.ToDouble(data, 32);
            double ry = BitConverter.ToDouble(data, 40);
            double rz = BitConverter.ToDouble(data, 48);
            
            // Unity 좌표계로 변환 (필요시)
            Vector3 position = new Vector3((float)x, (float)y, (float)-z);
            Quaternion rotation = new Quaternion((float)rx, (float)ry, (float)-rz, (float)w);
            
            // 메인 스레드에서 처리하도록 큐에 추가
            UpdatePose(position, rotation);
        }
        
        udpClient.BeginReceive(ReceiveCallback, null);
    }
}
```

## 성능 최적화

### 네트워크 최적화
1. **버퍼 크기 조정**
```python
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
```

2. **Non-blocking 모드**
```python
sock.setblocking(False)
```

3. **멀티캐스트 지원**
```python
# 멀티캐스트 그룹 설정
multicast_group = ('224.1.1.1', 8051)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
```

### 데이터 압축
더 작은 데이터 크기가 필요한 경우:
```python
# float32 사용 (정밀도 감소, 크기 절반)
sent = sock.sendto(struct.pack('7f', *[float(x) for x in data]), server_address)
```

## 문제 해결

### 일반적인 문제

1. **"Address already in use" 오류**
   - 원인: 포트가 이미 사용 중
   - 해결: 다른 포트 사용 또는 기존 프로세스 종료

2. **데이터가 수신되지 않음**
   - 방화벽 설정 확인
   - IP 주소와 포트 번호 확인
   - 네트워크 연결 상태 확인

3. **데이터 손실**
   - UDP는 신뢰성이 없는 프로토콜
   - 중요한 데이터는 시퀀스 번호 추가 고려
   ```python
   sequence = 0
   data_with_seq = [sequence] + data
   sent = sock.sendto(struct.pack('i7d', *data_with_seq), server_address)
   sequence += 1
   ```

4. **지연시간 문제**
   - 샘플링 속도 조정
   - 네트워크 대역폭 확인
   - 로컬 네트워크 사용 권장

## 확장 기능

### 여러 트래커 동시 전송
```python
# 트래커 ID를 포함한 패킷 구조
tracker_id = 1
data_with_id = [tracker_id] + data
sent = sock.sendto(struct.pack('i7d', *data_with_id), server_address)
```

### 타임스탬프 추가
```python
import time
timestamp = time.time()
data_with_time = [timestamp] + data
sent = sock.sendto(struct.pack('d8d', *data_with_time), server_address)
```

### 데이터 검증을 위한 체크섬
```python
import hashlib

def calculate_checksum(data):
    data_bytes = struct.pack('7d', *data)
    return hashlib.md5(data_bytes).digest()[:4]  # 4바이트 체크섬

checksum = calculate_checksum(data)
packet = struct.pack('7d', *data) + checksum
sent = sock.sendto(packet, server_address)
```

## 프로토콜 사양

### 패킷 구조
```
[위치 X][위치 Y][위치 Z][쿼터니언 W][쿼터니언 X][쿼터니언 Y][쿼터니언 Z]
 8 bytes  8 bytes  8 bytes  8 bytes    8 bytes    8 bytes    8 bytes
```

### 좌표계
- **OpenVR 좌표계**: Y축이 위, 우수 좌표계
- **Unity 변환**: Z축 반전 필요
- **Unreal 변환**: 스케일 조정 필요 (cm 단위)

## 보안 고려사항

1. **네트워크 격리**: VPN 또는 로컬 네트워크 사용 권장
2. **데이터 암호화**: 민감한 데이터의 경우 암호화 고려
3. **접근 제어**: IP 화이트리스트 구현
4. **데이터 검증**: 수신 데이터의 유효성 검사

## 성능 메트릭

### 대역폭 사용량
- **패킷 크기**: 56 바이트
- **250Hz 전송 시**: 14 KB/s
- **오버헤드 포함**: 약 20 KB/s

### 지연시간
- **로컬 네트워크**: < 1ms
- **Wi-Fi**: 2-5ms
- **인터넷**: 네트워크 상태에 따라 가변

### CPU 사용률
- **250Hz**: 약 1-2%
- **90Hz**: < 1%
- **최적화 팁**: 별도 스레드에서 전송 처리