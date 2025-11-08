using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class udp_tracked_object : MonoBehaviour
{
    Thread receiveThread;
    UdpClient client;
    private double[] float_array;
    public int port = 8051;

    private object lockObject = new object();
    private bool isRunning = true;
    private bool isInitialized = true;

    [Header("캘리브레이션")]
    public KeyCode calibrateKey = KeyCode.C; // C 키 누를때 플래그(bool isFirst = false)만 지정하여 UDP 값을 init pos, rot로 저장
    public bool autoCalibrate = true;
    private bool isCalibrated = false;
    
    [Header("오프셋(트래커 장착 보정)")]
    public Transform mountOffsetTCP;  // Transform은 positon, rotation(Vector3), scale까지 있음
                                      // 변수명에서 TCP 제거 요망...
                                      // Vector3 positionOffset, Vector3 rotationOffset으로 나누길 권장(쓸데없는 Scale까지 왜 지정?)

    [Header("스무딩/속도제한")]
    public bool smooth = true;
    [Range(0f, 1f)] public float posLerp = 0.1f;
    [Range(0f, 1f)] public float rotLerp = 0.1f;
    public bool limitSpeed = true;
    public float maxPosSpeed = 1.0f;   
    public float maxRotSpeed = 180f;   

    [Header("좌표 변환(OpenVR → Unity)")]
    public bool flipZPosition = true;
    public bool invertQuaternion = true; 

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

    void Start()
    {
        float_array = new double[7]; 
        initialObjectPosition = this.transform.position;
        initialObjectRotation = transform.rotation;

        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("UDP 서버 시작: 포트 " + port);

        lastRotTarget = transform.rotation;
    }

    void Update()
    {
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

            // OpenVR에서 받은 위치 데이터
            trackerPosition = new Vector3(
                (float)float_array[0],
                (float)float_array[1],
                (float)float_array[2]
            );

            // OpenVR에서 받은 쿼터니언 데이터 (w, x, y, z 순서)
            trackerRotation = new Quaternion(
                (float)float_array[6],  // z
                (float)float_array[5],  // y
                (float)float_array[4],  // x
                (float)float_array[3]   // w
            );

            // OpenVR -> Unity 좌표계 변환
            // OpenVR: X=right, Y=up, Z=backward
            // Unity: X=right, Y=up, Z=forward
            if (flipZPosition) 
            {
                trackerPosition.z = -trackerPosition.z;
            }
            
            // 쿼터니언 반전 옵션 (필요시)
            if (invertQuaternion) 
            {
                trackerRotation = Quaternion.Inverse(trackerRotation);
            }

            // 초기화 또는 자동 캘리브레이션
            if (!isInitialized || (autoCalibrate && !isCalibrated))
            {
                initialTrackerPosition = trackerPosition;
                initialTrackerRotation = trackerRotation;
                isInitialized = true;
                isCalibrated = true;
                Debug.Log("[UDP Tracker] 초기 위치 캘리브레이션 완료");
            }
        }

        // 상대 위치/회전 계산
        Vector3 positionDelta = trackerPosition - initialTrackerPosition;
        Quaternion rotationDelta = trackerRotation * Quaternion.Inverse(initialTrackerRotation);

        // 목표 위치와 회전 설정
        Vector3 targetPos = initialObjectPosition + positionDelta;
        Quaternion targetRot = initialObjectRotation * rotationDelta;
        
        // 디버그 정보 출력 (10프레임마다)
        if (showDebugInfo && frameCounter++ % 10 == 0)
        {
            Debug.Log($"[UDP Tracker Debug] Pos: {trackerPosition:F3}, Rot: {trackerRotation.eulerAngles:F1}");
            Debug.Log($"[UDP Tracker Debug] Delta Pos: {positionDelta:F3}, Target: {targetPos:F3}");
        }

        // 오프셋 적용 (있는 경우)
        if (mountOffsetTCP != null)
        {
            targetPos += targetRot * mountOffsetTCP.localPosition;
            targetRot = targetRot * mountOffsetTCP.localRotation;
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
            
            // 각도를 0-180도 범위로 정규화
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
                    (float)float_array[3]
                );

                if (flipZPosition)
                {
                    trackerPosition.z = -trackerPosition.z;
                }

                if (invertQuaternion)
                {
                    trackerRotation = Quaternion.Inverse(trackerRotation);
                }

                initialTrackerPosition = trackerPosition;
                initialTrackerRotation = trackerRotation;
                initialObjectPosition = transform.position;
                initialObjectRotation = transform.rotation;
                isCalibrated = true;
                
                Debug.Log("[UDP Tracker] 수동 캘리브레이션 완료 - 현재 위치를 기준점으로 설정");
            }
        }
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        client?.Close();
    }

    private void ReceiveData()
    {
        try
        {
            client = new UdpClient(port);
            Debug.Log("Starting Server on port " + port);

            IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
            while (isRunning)
            {
                byte[] data = client.Receive(ref anyIP);
                // 1. doubleCount가 7미만인경우???? 정의 안됨
                // 2. doubleCount가 7인 경우 제대로 돌아감.. But 7 이상인 경우
                // 3. 왜 7???(<=== magic number) x, y, z, rx, ry, rz는 총 6개임... <= 검증 필요
                // 4. ToDouble의 float에 제대로 들어감?
                // 5. bool isFirst 플래그로 초기값 저장 필요
                // 6. 필요에 의해 isFirst를 false로 Set 하여 Calibration 필요
                lock (lockObject)
                {
                    int doubleCount = data.Length / 8;
                    if (doubleCount >= 7)
                    {
                        for (int i = 0; i < 7; i++)
                            float_array[i] = BitConverter.ToDouble(data, i * 8);

                        hasNewPose = true;
                    }
                }
            }
        }
        catch (SocketException e)
        {
            if (isRunning) Debug.LogError("UDP 수신 오류: " + e);
        }
        catch (Exception err)
        {
            Debug.LogError("UDP 수신 오류: " + err);
        }
        finally
        {
            client?.Close();
            Debug.Log("UDP 서버가 종료되었습니다.");
        }
    }
}
