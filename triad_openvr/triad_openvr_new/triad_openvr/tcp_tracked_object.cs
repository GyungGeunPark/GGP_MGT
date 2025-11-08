using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;

public class tcp_tracked_object : MonoBehaviour
{
    private Thread receiveThread;
    private TcpListener tcpListener;
    private TcpClient connectedClient;
    private NetworkStream stream;
    private double[] float_array;
    public int port = 8051;
    
    [Header("30도 오프셋 보정")]
    public bool apply30DegreeCorrection = false;
    public float rotationOffsetY = -30f;  // Y축 회전 오프셋 (도)

    private object lockObject = new object();
    private bool isRunning = true;
    private bool isInitialized = false;
    private bool isConnected = false;

    [Header("캘리브레이션")]
    public KeyCode calibrateKey = KeyCode.C;
    public bool autoCalibrate = true;
    private bool isCalibrated = false;
    
    [Header("오프셋(트래커 장착 보정)")]
    public Transform mountOffsetTransform;  // TCP에서 Transform으로 변경
    
    [Header("스무딩/속도제한")]
    public bool smooth = true;
    [Range(0f, 1f)] public float posLerp = 0.1f;
    [Range(0f, 1f)] public float rotLerp = 0.1f;
    public bool limitSpeed = true;
    public float maxPosSpeed = 1.0f;   
    public float maxRotSpeed = 180f;   

    [Header("좌표 변환(OpenVR → Unity)")]
    public bool flipZPosition = false;
    public bool invertQuaternion = false;  // Deprecated - use axis-specific inversion

    [Header("축별 회전 반전 설정")]
    public bool invertW = false;  // X축 회전 반전
    public bool invertX = true;      // Y축 회전 반전 (OpenVR→Unity 변환시 일반적으로 필요)
    public bool invertY = false;    // Z축 회전 반전 
    public bool invertZ = false;    // W축 회전 반전

    private Vector3 initialObjectPosition;
    private Quaternion initialObjectRotation;
    private Vector3 initialTrackerPosition;
    private Quaternion initialTrackerRotation;

    private Vector3 recvPos;
    private Quaternion recvRot;
    private bool hasNewPose = false;

    private Quaternion lastRotTarget;
    
    [Header("디버그")]
    public bool showDebugInfo = false;
    private int frameCounter = 0;
    
    [Header("연결 상태")]
    public bool showConnectionStatus = true;
    private float reconnectInterval = 3.0f;
    private float lastReconnectAttempt = 0f;

    void Start()
    {
        float_array = new double[7];  // Euler angles: x, y, z, rw, rx, ry, rz 
        initialObjectPosition = transform.position;
        initialObjectRotation = transform.rotation;
        lastRotTarget = transform.rotation;

        StartTCPServer();
    }

    void StartTCPServer()
    {
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log($"[TCP Tracker] TCP 서버 시작: 포트 {port}");
    }

    void Update()
    {
        // 연결 상태 확인 및 재연결 시도
        if (!isConnected && Time.time - lastReconnectAttempt > reconnectInterval)
        {
            lastReconnectAttempt = Time.time;
            if (showConnectionStatus)
            {
                Debug.Log("[TCP Tracker] 클라이언트 연결 대기 중...");
            }
        }

        // 캘리브레이션 키 체크
        if (Input.GetKeyDown(calibrateKey))
        {
            Calibrate();
        }

        Vector3 trackerPosition;
        Quaternion trackerRotation;

        lock (lockObject)
        {
            if (!hasNewPose) return;

            // OpenVR에서 받은 위치 데이터 (이미 Z축 반전이 적용됨)
            trackerPosition = new Vector3(
                (float)float_array[0],
                (float)float_array[1],
                (float)float_array[2]
            );

            // OpenVR에서 받은 Euler 각도 데이터 (pitch, yaw, roll)
            trackerRotation = new Quaternion((float)float_array[6], (float)float_array[5], (float)float_array[4], (float)float_array[3]);
            //float pitch = (float)float_array[3];  // X축 회전
            //float yaw = (float)float_array[4];    // Y축 회전
            //float roll = (float)float_array[5];   // Z축 회전

            //// 축별 선택적 반전 적용 (OpenVR → Unity 좌표계 변환)
            //if (invertPitch) pitch = -pitch;
            //if (invertYaw) yaw = -yaw;
            //if (invertRoll) roll = -roll;

            // Unity의 Quaternion.Euler 사용
            //trackerRotation = Quaternion.Euler(pitch, yaw, roll);
            
            //// 30도 오프셋 보정 (필요한 경우)
            //if (apply30DegreeCorrection)
            //{
            //    trackerRotation = Quaternion.Euler(0, rotationOffsetY, 0) * trackerRotation;
            //}

            // 좌표계 변환은 이미 Python에서 처리됨
            // flipZPosition과 invertQuaternion은 더 이상 필요하지 않지만,
            // 호환성을 위해 남겨둠 (필요시 추가 조정 가능)
            if (flipZPosition) 
            {
                // Python에서 이미 Z 반전 처리했으므로 일반적으로 false로 설정
                trackerPosition.z = -trackerPosition.z;
            }
            
            // invertQuaternion은 deprecated - 축별 반전 사용
            // 호환성을 위해 남겨둠 (사용 비권장)
            if (invertQuaternion)
            {
                Debug.LogWarning("[TCP Tracker] invertQuaternion is deprecated. Use axis-specific inversion instead.");
                trackerRotation = Quaternion.Inverse(trackerRotation);
            }

            if (invertX) trackerRotation.x *= -1;
            if (invertY) trackerRotation.y *= -1;
            if (invertZ) trackerRotation.z *= -1;
            if (invertW) trackerRotation.w *= -1;

            // 초기화 또는 자동 캘리브레이션
            if (!isInitialized || (autoCalibrate && !isCalibrated))
            {
                initialTrackerPosition = trackerPosition;
                initialTrackerRotation = trackerRotation;
                isInitialized = true;
                isCalibrated = true;
                Debug.Log("[TCP Tracker] 초기 위치 캘리브레이션 완료");
            }
        }

        // 상대 위치/회전 계산
        Vector3 positionDelta = trackerPosition - initialTrackerPosition;
        Quaternion rotationDelta = trackerRotation * Quaternion.Inverse(initialTrackerRotation);

        // 목표 위치와 회전 설정
        Vector3 targetPos = initialObjectPosition + positionDelta;
        Quaternion targetRot = initialObjectRotation * rotationDelta;
        
        // 디버그 정보 출력 (Euler 각도 포함)
        if (showDebugInfo && frameCounter++ % 10 == 0)
        {
            Debug.Log($"[TCP Tracker Debug] Received Q: W={float_array[3]:F1}° X={float_array[4]:F1}° Y={float_array[5]:F1}° Z={float_array[6]:F1}");
            Debug.Log($"[TCP Tracker Debug] Pos: {trackerPosition:F3}, Rot: {trackerRotation.eulerAngles:F1}");
            Debug.Log($"[TCP Tracker Debug] Delta Pos: {positionDelta:F3}, Target: {targetPos:F3}");
            Debug.Log($"[TCP Tracker Debug] Connected: {isConnected}, 30° Correction: {apply30DegreeCorrection}");
        }

        // 오프셋 적용
        if (mountOffsetTransform != null)
        {
            targetPos += targetRot * mountOffsetTransform.localPosition;
            targetRot = targetRot * mountOffsetTransform.localRotation;
        }

        // 속도 제한
        if (limitSpeed)
        {
            Vector3 dp = targetPos - transform.position;
            float maxStep = maxPosSpeed * Time.deltaTime;
            if (dp.magnitude > maxStep && dp.magnitude > 0.001f)
            {
                targetPos = transform.position + dp.normalized * maxStep;
            }

            float angle;
            Vector3 axis;
            Quaternion deltaRot = targetRot * Quaternion.Inverse(lastRotTarget);
            deltaRot.ToAngleAxis(out angle, out axis);
            
            if (angle > 180f) angle = 360f - angle;
            
            float maxAngStep = maxRotSpeed * Time.deltaTime;
            if (angle > maxAngStep && angle > 0.01f)
            {
                targetRot = Quaternion.Slerp(lastRotTarget, targetRot, maxAngStep / angle);
            }
        }

        // 스무딩 적용
        if (smooth)
        {
            transform.position = Vector3.Lerp(transform.position, targetPos, posLerp);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, rotLerp);
        }
        else
        {
            transform.position = targetPos;
            transform.rotation = targetRot;
        }

        lastRotTarget = targetRot;
    }

    void Calibrate()
    {
        lock (lockObject)
        {
            if (hasNewPose)
            {
                Vector3 trackerPosition = new Vector3(
                    (float)float_array[0],
                    (float)float_array[1],
                    (float)float_array[2]
                );

                Quaternion trackerRotation = new Quaternion(
                    (float)float_array[6], 
                    (float)float_array[5], 
                    (float)float_array[4], 
                    (float)float_array[3]);

                Debug.Log($"Received Q {trackerRotation}");
                //// Euler angles to Quaternion
                //float pitch = (float)float_array[3];
                //float yaw = (float)float_array[4];
                //float roll = (float)float_array[5];

                // 축별 선택적 반전 적용

                //Quaternion trackerRotation = Quaternion.Euler(pitch, yaw, roll);

                //if (apply30DegreeCorrection)
                //{
                //    trackerRotation = Quaternion.Euler(0, rotationOffsetY, 0) * trackerRotation;
                //}

                if (flipZPosition)
                {
                    trackerPosition.z = -trackerPosition.z;
                }

                // invertQuaternion은 deprecated
                if (invertQuaternion)
                {
                    Debug.LogWarning("[TCP Tracker] invertQuaternion is deprecated. Use axis-specific inversion instead.");
                    trackerRotation = Quaternion.Inverse(trackerRotation);
                }

                if (invertX) trackerRotation.x *= -1;
                if (invertY) trackerRotation.y *= -1;
                if (invertZ) trackerRotation.z *= -1;
                if (invertW) trackerRotation.w *= -1;

                initialTrackerPosition = trackerPosition;
                initialTrackerRotation = trackerRotation;
                initialObjectPosition = transform.position;
                initialObjectRotation = transform.rotation;
                isCalibrated = true;
                
                Debug.Log("[TCP Tracker] 수동 캘리브레이션 완료 - 현재 위치를 기준점으로 설정");
            }
        }
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        CleanupTCP();
    }

    void OnDestroy()
    {
        isRunning = false;
        CleanupTCP();
    }

    private void CleanupTCP()
    {
        try
        {
            if (stream != null)
            {
                stream.Close();
                stream = null;
            }
            if (connectedClient != null)
            {
                connectedClient.Close();
                connectedClient = null;
            }
            if (tcpListener != null)
            {
                tcpListener.Stop();
                tcpListener = null;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[TCP Tracker] Cleanup error: {e.Message}");
        }
    }

    private void ReceiveData()
    {
        try
        {
            // TCP 리스너 생성 및 시작
            tcpListener = new TcpListener(IPAddress.Any, port);
            tcpListener.Start();
            Debug.Log($"[TCP Tracker] TCP 서버 시작됨 - 포트: {port}");

            byte[] sizeBuffer = new byte[4];
            byte[] dataBuffer = new byte[1024];

            while (isRunning)
            {
                try
                {
                    // 클라이언트 연결 대기 (타임아웃 설정)
                    if (!isConnected)
                    {
                        Debug.Log("[TCP Tracker] 클라이언트 연결 대기 중...");
                        
                        // 비동기 연결 대기
                        IAsyncResult result = tcpListener.BeginAcceptTcpClient(null, null);
                        bool success = result.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(5));
                        
                        if (success)
                        {
                            connectedClient = tcpListener.EndAcceptTcpClient(result);
                            stream = connectedClient.GetStream();
                            stream.ReadTimeout = 5000;  // 5초 읽기 타임아웃
                            isConnected = true;
                            Debug.Log("[TCP Tracker] 클라이언트 연결됨!");
                        }
                        else
                        {
                            continue;  // 타임아웃 발생, 다시 대기
                        }
                    }

                    // 연결된 상태에서 데이터 수신
                    if (isConnected && stream != null)
                    {
                        // 먼저 데이터 크기 읽기 (4바이트)
                        int bytesRead = 0;
                        while (bytesRead < 4)
                        {
                            int read = stream.Read(sizeBuffer, bytesRead, 4 - bytesRead);
                            if (read == 0)
                            {
                                throw new Exception("Connection closed by remote host");
                            }
                            bytesRead += read;
                        }

                        int dataSize = BitConverter.ToInt32(sizeBuffer, 0);
                        
                        // 버퍼 크기 조정
                        if (dataBuffer.Length < dataSize)
                        {
                            dataBuffer = new byte[dataSize];
                        }

                        // 실제 데이터 읽기
                        bytesRead = 0;
                        while (bytesRead < dataSize)
                        {
                            int read = stream.Read(dataBuffer, bytesRead, dataSize - bytesRead);
                            if (read == 0)
                            {
                                throw new Exception("Connection closed while reading data");
                            }
                            bytesRead += read;
                        }

                        // 데이터 파싱
                        lock (lockObject)
                        {
                            int doubleCount = dataSize / 8;
                            if (doubleCount >= 7)  // 6개 값: x, y, z, pitch, yaw, roll
                            {
                                for (int i = 0; i < 7; i++)
                                {
                                    float_array[i] = BitConverter.ToDouble(dataBuffer, i * 8);
                                }
                                hasNewPose = true;
                                
                                // 디버그: 수신 데이터 확인
                                if (showDebugInfo && frameCounter % 60 == 0)  // 1초마다 출력 (60fps 기준)
                                {
                                    Debug.Log($"[TCP] Received Data - Pos: ({float_array[0]:F2}, {float_array[1]:F2}, {float_array[2]:F2}) " +
                                              $"Quaternion: ({float_array[3]:W}°, {float_array[4]:X}°, {float_array[5]:Y}°, {float_array[6]:Z})");
                                }
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    if (isRunning)
                    {
                        Debug.LogWarning($"[TCP Tracker] 연결 오류: {e.Message}");
                        isConnected = false;
                        
                        // 연결 정리
                        if (stream != null)
                        {
                            stream.Close();
                            stream = null;
                        }
                        if (connectedClient != null)
                        {
                            connectedClient.Close();
                            connectedClient = null;
                        }
                        
                        // 잠시 대기 후 재시도
                        Thread.Sleep(1000);
                    }
                }
            }
        }
        catch (SocketException e)
        {
            if (isRunning) 
                Debug.LogError($"[TCP Tracker] 소켓 오류: {e.Message}");
        }
        catch (Exception e)
        {
            if (isRunning)
                Debug.LogError($"[TCP Tracker] 예외 발생: {e.Message}");
        }
        finally
        {
            CleanupTCP();
            Debug.Log("[TCP Tracker] TCP 서버가 종료되었습니다.");
        }
    }
}