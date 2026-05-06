#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

// 스피네이커 관련 헤더
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory> // [최적화] std::shared_ptr 사용을 위해 추가

// 소켓 통신 관련
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <netinet/tcp.h>

// ROS2 메시지 타입
#include "std_msgs/msg/string.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

class CameraTcpStreamerNode : public rclcpp::Node
{
public:
    CameraTcpStreamerNode(const std::string& camera_name, int server_port = 9100)
        : Node("camera_node_" + camera_name)
    {
        // ... (생성자의 다른 부분은 이전과 동일) ...
        RCLCPP_INFO(this->get_logger(), "CameraTcpStreamerNode constructor called for %s", camera_name.c_str());

        camera_name_ = camera_name;
        server_port_ = server_port;

        std::string status_topic = "/camera_status/" + camera_name_;
        status_publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing status to: %s", status_topic.c_str());

        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraTcpStreamerNode::publishStatus, this)
        );

        system_ = System::GetInstance();
        cam_list_ = system_->GetCameras();
        if (cam_list_.GetSize() < 1)
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras detected");
            error_occurred_ = true;
            last_error_msg_ = "No cameras detected";
            return;
        }
        // GigE 카메라를 시리얼 번호로 탐색
        cam_ = nullptr;
        for (unsigned int i = 0; i < cam_list_.GetSize(); i++)
        {
            CameraPtr tmpCam = cam_list_.GetByIndex(i);
            INodeMap &tlNodeMap = tmpCam->GetTLDeviceNodeMap();
            CStringPtr serialPtr = tlNodeMap.GetNode("DeviceSerialNumber");
            if (IsAvailable(serialPtr) && IsReadable(serialPtr))
            {
                std::string serial = serialPtr->GetValue().c_str();
                RCLCPP_INFO(this->get_logger(), "Found camera [%d]: serial=%s", i, serial.c_str());
                if (serial == "18566179")
                {
                    cam_ = tmpCam;
                    RCLCPP_INFO(this->get_logger(), "Matched GigE camera serial 18566179");
                    break;
                }
            }
        }
        if (!cam_)
        {
            RCLCPP_WARN(this->get_logger(), "Serial 18566179 not found, falling back to index 0");
            cam_ = cam_list_.GetByIndex(0);
        }

        // GigE 카메라 인터페이스 MTU를 점보 프레임으로 설정 (패킷 손실 방지)
        configureNetworkForGigE();

        // Auto ForceIP → 영구 IP 설정 → 카메라 시작
        autoForceIP();
        configurePersistentIP();
        setupCameraAndStart();

        server_thread_ = std::thread(&CameraTcpStreamerNode::startHttpServer, this);

        camera_control_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/camera_control", 10,
            std::bind(&CameraTcpStreamerNode::cameraControlCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera_control topic");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CameraTcpStreamerNode::grabAndStoreFrame, this)
        );

        // 프레임 정체 감시 워치독 (30초마다 점검)
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&CameraTcpStreamerNode::watchdogCheck, this)
        );
        last_good_frame_time_ = std::chrono::steady_clock::now();
    }
    
private:
    // ... (publishStatus, cameraControlCallback, setupCameraAndStart, spinnakerFullReinit, startHttpServer 함수는 이전과 동일) ...
    void publishStatus()
    {
        std_msgs::msg::String status_msg;
        if (error_occurred_) {
            status_msg.data = "error:" + last_error_msg_;
        } else {
            status_msg.data = "normal";
        }
        status_publisher_->publish(status_msg);
    }
    void cameraControlCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        size_t firstColon = command.find(':');
        if (firstColon == std::string::npos) return;
        
        std::string target_camera_name = command.substr(0, firstColon);
        
        if (target_camera_name != this->camera_name_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received command for this camera: %s", command.c_str());
        
        std::string remaining = command.substr(firstColon + 1);

        if (remaining.find("set_gain:") == 0)
        {
            try {
                std::string gain_str = remaining.substr(9);
                float gain_value = std::stof(gain_str);
                RCLCPP_INFO(this->get_logger(), "Setting gain to %.1f for camera %s", gain_value, target_camera_name.c_str());
                if (cam_ && cam_->IsValid()) {
                    INodeMap &nodeMap = cam_->GetNodeMap();
                    CFloatPtr gainPtr = nodeMap.GetNode("Gain");
                    if (IsAvailable(gainPtr) && IsWritable(gainPtr)) {
                        gainPtr->SetValue(gain_value);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error setting gain: %s", e.what());
                error_occurred_ = true;
                last_error_msg_ = std::string("Error setting gain: ") + e.what();
            }
        }
        else if (remaining.find("set_auto_exposure:") == 0)
        {
            try {
                std::string auto_exposure_str = remaining.substr(18);
                RCLCPP_INFO(this->get_logger(), "Setting auto exposure to %s for camera %s", auto_exposure_str.c_str(), target_camera_name.c_str());
                if (cam_ && cam_->IsValid()) {
                    INodeMap &nodeMap = cam_->GetNodeMap();
                    CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
                    if (IsAvailable(exposureAutoPtr) && IsWritable(exposureAutoPtr)) {
                        std::string spinnaker_mode = (auto_exposure_str == "On") ? "Continuous" : "Off";
                        exposureAutoPtr->FromString(spinnaker_mode.c_str());
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error setting auto exposure: %s", e.what());
                error_occurred_ = true;
                last_error_msg_ = std::string("Error setting auto exposure: ") + e.what();
            }
        }
        else if (remaining.find("set_exposure:") == 0)
        {
            try {
                std::string exposure_str = remaining.substr(13);
                float exposure_value = std::stof(exposure_str);
                RCLCPP_INFO(this->get_logger(), "Setting exposure to %.1f us for camera %s", exposure_value, target_camera_name.c_str());
                if (cam_ && cam_->IsValid()) {
                    INodeMap &nodeMap = cam_->GetNodeMap();
                    CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
                    if (IsAvailable(exposureAutoPtr) && IsWritable(exposureAutoPtr)) {
                        exposureAutoPtr->FromString("Off");
                    }
                    CFloatPtr exposureTimePtr = nodeMap.GetNode("ExposureTime");
                    if (IsAvailable(exposureTimePtr) && IsWritable(exposureTimePtr)) {
                        exposureTimePtr->SetValue(exposure_value);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error setting exposure: %s", e.what());
                error_occurred_ = true;
                last_error_msg_ = std::string("Error setting exposure: ") + e.what();
            }
        }
    }
    // GigE Vision용 Linux 네트워크 수신 버퍼 최적화
    // eth2는 USB-Ethernet 어댑터(RTL8153)이므로 점보 프레임 미지원 → 소켓 버퍼로 보상
    void configureNetworkForGigE()
    {
        const std::string iface = "eth2";

        // eth2 MTU를 1500으로 유지 (USB NIC는 점보 프레임 미지원)
        // MTU가 잘못 9000으로 설정되어 있으면 1500으로 복원
        {
            std::ifstream mtu_file("/sys/class/net/" + iface + "/mtu");
            if (mtu_file.is_open()) {
                int current_mtu = 0;
                mtu_file >> current_mtu;
                mtu_file.close();
                if (current_mtu > 1500) {
                    RCLCPP_WARN(this->get_logger(), "NIC %s MTU=%d (USB NIC doesn't support jumbo). Resetting to 1500.",
                        iface.c_str(), current_mtu);
                    std::string cmd = "ifconfig " + iface + " mtu 1500";
                    system(cmd.c_str());
                }
                RCLCPP_INFO(this->get_logger(), "NIC %s: USB-Ethernet (RTL8153), MTU=1500, using small packets.", iface.c_str());
            }
        }

        // Linux 소켓 수신 버퍼 크기 증가 (GigE Vision 대용량 프레임 수신에 필수)
        // 기본값 212KB → 25MB (3600x3600 프레임 2개분 확보)
        struct {const char* param; const char* value;} sysctl_params[] = {
            {"net.core.rmem_max", "26214400"},        // 소켓 수신 버퍼 최대: 25MB
            {"net.core.rmem_default", "26214400"},     // 소켓 수신 버퍼 기본: 25MB
            {"net.core.netdev_max_backlog", "10000"},   // NIC 수신 큐 깊이 증가
        };

        for (auto& p : sysctl_params) {
            std::string cmd = std::string("sysctl -w ") + p.param + "=" + p.value;
            int ret = system(cmd.c_str());
            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(), "sysctl %s=%s set OK", p.param, p.value);
            } else {
                RCLCPP_WARN(this->get_logger(), "sysctl %s=%s failed (ret=%d). Run as root.", p.param, p.value, ret);
            }
        }
    }

    // Auto ForceIP: 카메라가 다른 서브넷에 있을 때 강제로 IP를 할당 (Init 전에 호출)
    void autoForceIP()
    {
        if (!cam_) return;
        try
        {
            INodeMap &tlNodeMap = cam_->GetTLDeviceNodeMap();

            // 현재 카메라 IP 확인
            CIntegerPtr currentIPPtr = tlNodeMap.GetNode("GevDeviceIPAddress");
            CIntegerPtr currentSubnetPtr = tlNodeMap.GetNode("GevDeviceSubnetMask");
            int64_t currentIP = 0, currentSubnet = 0;
            if (IsAvailable(currentIPPtr) && IsReadable(currentIPPtr))
                currentIP = currentIPPtr->GetValue();
            if (IsAvailable(currentSubnetPtr) && IsReadable(currentSubnetPtr))
                currentSubnet = currentSubnetPtr->GetValue();

            // 목표 IP: 192.168.2.1, 서브넷: 255.255.255.0
            int64_t targetIP = (192LL << 24) | (168LL << 16) | (2LL << 8) | 1LL;
            int64_t targetSubnet = (255LL << 24) | (255LL << 16) | (255LL << 8) | 0LL;

            // 현재 IP가 이미 같은 서브넷(192.168.2.x)에 있으면 ForceIP 불필요
            if ((currentIP & targetSubnet) == (targetIP & targetSubnet) && currentSubnet == targetSubnet)
            {
                RCLCPP_INFO(this->get_logger(), "Camera already on correct subnet (IP=%ld.%ld.%ld.%ld). Skipping ForceIP.",
                    (currentIP >> 24) & 0xFF, (currentIP >> 16) & 0xFF,
                    (currentIP >> 8) & 0xFF, currentIP & 0xFF);
                return;
            }

            RCLCPP_WARN(this->get_logger(), "Camera on wrong subnet (IP=%ld.%ld.%ld.%ld). Executing Auto ForceIP to 192.168.2.1...",
                (currentIP >> 24) & 0xFF, (currentIP >> 16) & 0xFF,
                (currentIP >> 8) & 0xFF, currentIP & 0xFF);

            // ForceIP 주소 설정
            CIntegerPtr forceIPPtr = tlNodeMap.GetNode("GevDeviceForceIPAddress");
            if (IsAvailable(forceIPPtr) && IsWritable(forceIPPtr))
                forceIPPtr->SetValue(targetIP);

            // ForceIP 서브넷 설정
            CIntegerPtr forceSubnetPtr = tlNodeMap.GetNode("GevDeviceForceSubnetMask");
            if (IsAvailable(forceSubnetPtr) && IsWritable(forceSubnetPtr))
                forceSubnetPtr->SetValue(targetSubnet);

            // ForceIP 게이트웨이 설정 (0.0.0.0)
            CIntegerPtr forceGwPtr = tlNodeMap.GetNode("GevDeviceForceGateway");
            if (IsAvailable(forceGwPtr) && IsWritable(forceGwPtr))
                forceGwPtr->SetValue(0);

            // ForceIP 실행
            CCommandPtr forceIPCmdPtr = tlNodeMap.GetNode("GevDeviceForceIP");
            if (IsAvailable(forceIPCmdPtr) && IsWritable(forceIPCmdPtr))
            {
                forceIPCmdPtr->Execute();
                RCLCPP_INFO(this->get_logger(), "ForceIP executed: 192.168.2.1/255.255.255.0");

                // ForceIP 후 카메라 재탐색 (최대 5회 재시도, 2초 간격)
                for (int retry = 0; retry < 5; retry++)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                    cam_list_.Clear();
                    cam_list_ = system_->GetCameras();
                    cam_ = nullptr;

                    RCLCPP_INFO(this->get_logger(), "ForceIP re-enum attempt %d/5: found %u camera(s)",
                        retry + 1, cam_list_.GetSize());

                    for (unsigned int i = 0; i < cam_list_.GetSize(); i++)
                    {
                        CameraPtr tmpCam = cam_list_.GetByIndex(i);
                        INodeMap &tl = tmpCam->GetTLDeviceNodeMap();
                        CStringPtr serialPtr = tl.GetNode("DeviceSerialNumber");
                        if (IsAvailable(serialPtr) && IsReadable(serialPtr))
                        {
                            std::string serial = serialPtr->GetValue().c_str();
                            if (serial == "18566179") { cam_ = tmpCam; break; }
                        }
                    }
                    if (!cam_ && cam_list_.GetSize() > 0)
                        cam_ = cam_list_.GetByIndex(0);

                    if (cam_) {
                        RCLCPP_INFO(this->get_logger(), "Camera re-detected after ForceIP (attempt %d).", retry + 1);
                        break;
                    }
                }

                if (!cam_)
                    RCLCPP_ERROR(this->get_logger(), "Camera not found after ForceIP (all retries exhausted)!");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "GevDeviceForceIP command not available.");
            }
        }
        catch (const Spinnaker::Exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Auto ForceIP failed: %s", e.what());
        }
    }

    // 카메라에 영구 IP 설정 (Init 전에 호출 — TL 노드맵 사용)
    void configurePersistentIP()
    {
        if (!cam_) return;
        try
        {
            INodeMap &tlNodeMap = cam_->GetTLDeviceNodeMap();

            // Persistent IP 활성화
            CBooleanPtr persistentIPEnablePtr = tlNodeMap.GetNode("GevCurrentIPConfigurationPersistentIP");
            if (IsAvailable(persistentIPEnablePtr) && IsWritable(persistentIPEnablePtr))
            {
                persistentIPEnablePtr->SetValue(true);
            }

            // 카메라 Init 후 GenICam 노드맵에서 설정
            cam_->Init();

            INodeMap &nodeMap = cam_->GetNodeMap();

            // Persistent IP 주소: 192.168.2.1
            CIntegerPtr persistIPPtr = nodeMap.GetNode("GevPersistentIPAddress");
            if (IsAvailable(persistIPPtr) && IsWritable(persistIPPtr))
            {
                // 192.168.2.1 = (192<<24)|(168<<16)|(2<<8)|1
                int64_t ip = (192LL << 24) | (168LL << 16) | (2LL << 8) | 1LL;
                persistIPPtr->SetValue(ip);
            }

            // Persistent 서브넷: 255.255.255.0
            CIntegerPtr persistSubnetPtr = nodeMap.GetNode("GevPersistentSubnetMask");
            if (IsAvailable(persistSubnetPtr) && IsWritable(persistSubnetPtr))
            {
                int64_t mask = (255LL << 24) | (255LL << 16) | (255LL << 8) | 0LL;
                persistSubnetPtr->SetValue(mask);
            }

            // Persistent 게이트웨이: 0.0.0.0
            CIntegerPtr persistGwPtr = nodeMap.GetNode("GevPersistentDefaultGateway");
            if (IsAvailable(persistGwPtr) && IsWritable(persistGwPtr))
            {
                persistGwPtr->SetValue(0);
            }

            RCLCPP_INFO(this->get_logger(), "Persistent IP configured: 192.168.2.1/24");

            cam_->DeInit();
        }
        catch (const Spinnaker::Exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Persistent IP config failed (non-fatal): %s", e.what());
            try { cam_->DeInit(); } catch (...) {}
        }
    }

    void setupCameraAndStart()
    {
        if (!cam_) return;

        try
        {
            cam_->Init();

            // 이전 세션의 acquisition이 남아있을 수 있으므로 먼저 중지 시도
            try { cam_->EndAcquisition(); }
            catch (...) { /* 이미 중지 상태면 무시 */ }

            {
                INodeMap &sNodeMap = cam_->GetTLStreamNodeMap();
                CEnumerationPtr bufferHandlingPtr = sNodeMap.GetNode("StreamBufferHandlingMode");
                if (IsAvailable(bufferHandlingPtr) && IsWritable(bufferHandlingPtr))
                {
                    bufferHandlingPtr->FromString("NewestOnly");
                }

                // 스트림 버퍼 수 증가 — USB NIC의 지연 변동에 대비
                CIntegerPtr bufferCountPtr = sNodeMap.GetNode("StreamDefaultBufferCount");
                if (IsAvailable(bufferCountPtr) && IsWritable(bufferCountPtr))
                {
                    bufferCountPtr->SetValue(10);
                    RCLCPP_INFO(this->get_logger(), "StreamDefaultBufferCount set to 10");
                }

                // 패킷 재전송 활성화 — Image incomplete 방지 핵심 설정
                CBooleanPtr packetResendPtr = sNodeMap.GetNode("StreamPacketResendEnable");
                if (IsAvailable(packetResendPtr) && IsWritable(packetResendPtr))
                {
                    packetResendPtr->SetValue(true);
                    RCLCPP_INFO(this->get_logger(), "StreamPacketResendEnable = true");
                }

                // 패킷 재전송 최대 요청 수
                CIntegerPtr resendMaxPtr = sNodeMap.GetNode("StreamPacketResendMaxRequests");
                if (IsAvailable(resendMaxPtr) && IsWritable(resendMaxPtr))
                {
                    resendMaxPtr->SetValue(1000);
                    RCLCPP_INFO(this->get_logger(), "StreamPacketResendMaxRequests set to 1000");
                }

                // 패킷 재전송 타임아웃 (ms)
                CIntegerPtr resendTimeoutPtr = sNodeMap.GetNode("StreamPacketResendTimeout");
                if (IsAvailable(resendTimeoutPtr) && IsWritable(resendTimeoutPtr))
                {
                    resendTimeoutPtr->SetValue(2000);
                    RCLCPP_INFO(this->get_logger(), "StreamPacketResendTimeout set to 2000ms");
                }
            }
            // GigE 카메라 전송 설정 (패킷 크기, 패킷 딜레이)
            {
                INodeMap &nodeMap = cam_->GetNodeMap();

                // GevSCPSPacketSize: USB-Ethernet NIC (MTU 1500) 기준 안전한 패킷 크기
                // GigE Vision 헤더(36B) + IP/UDP 오버헤드 고려 → 1400이 안전 최대값
                CIntegerPtr packetSizePtr = nodeMap.GetNode("GevSCPSPacketSize");
                if (IsAvailable(packetSizePtr) && IsWritable(packetSizePtr))
                {
                    int64_t maxPacketSize = packetSizePtr->GetMax();
                    int64_t packetSize = std::min((int64_t)1400, maxPacketSize);
                    packetSizePtr->SetValue(packetSize);
                    RCLCPP_INFO(this->get_logger(), "GevSCPSPacketSize set to %ld (max: %ld, USB NIC safe)",
                        (long)packetSize, (long)maxPacketSize);
                }

                // GevSCPD: 인터패킷 딜레이 — USB NIC의 처리 부하를 고려하여 충분한 딜레이
                // 1400B 패킷 × 9258패킷/프레임 × 25µs = 약 231ms (1fps에 충분)
                CIntegerPtr packetDelayPtr = nodeMap.GetNode("GevSCPD");
                if (IsAvailable(packetDelayPtr) && IsWritable(packetDelayPtr))
                {
                    packetDelayPtr->SetValue(25000);
                    RCLCPP_INFO(this->get_logger(), "GevSCPD (inter-packet delay) set to 25000ns (USB NIC optimized)");
                }

                // GevHeartbeatTimeout: 장시간 운영 시 heartbeat 만료 방지 (기본 3초 → 10초)
                CIntegerPtr heartbeatPtr = nodeMap.GetNode("GevHeartbeatTimeout");
                if (IsAvailable(heartbeatPtr) && IsWritable(heartbeatPtr))
                {
                    heartbeatPtr->SetValue(10000);  // 10초 (밀리초 단위)
                    RCLCPP_INFO(this->get_logger(), "GevHeartbeatTimeout set to 10000ms");
                }

                // DeviceLinkThroughputLimit: 대역폭 제한 완화
                CIntegerPtr throughputPtr = nodeMap.GetNode("DeviceLinkThroughputLimit");
                if (IsAvailable(throughputPtr) && IsWritable(throughputPtr))
                {
                    int64_t maxThroughput = throughputPtr->GetMax();
                    throughputPtr->SetValue(maxThroughput);
                    RCLCPP_INFO(this->get_logger(), "DeviceLinkThroughputLimit set to %ld",
                        (long)maxThroughput);
                }
            }
            {
                INodeMap &nodeMap = cam_->GetNodeMap();

                // AcquisitionMode를 먼저 설정 (acquisition 중지 상태에서만 쓰기 가능)
                CEnumerationPtr acquisitionModePtr = nodeMap.GetNode("AcquisitionMode");
                if (IsAvailable(acquisitionModePtr) && IsWritable(acquisitionModePtr))
                {
                    CEnumEntryPtr contEntry = acquisitionModePtr->GetEntryByName("Continuous");
                    if (IsAvailable(contEntry) && IsReadable(contEntry))
                    {
                        acquisitionModePtr->SetIntValue(contEntry->GetValue());
                        RCLCPP_INFO(this->get_logger(), "AcquisitionMode set to Continuous");
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "AcquisitionMode not writable — assuming already Continuous (default)");
                }

                CEnumerationPtr triggerModePtr = nodeMap.GetNode("TriggerMode");
                if (IsAvailable(triggerModePtr) && IsWritable(triggerModePtr))
                {
                    triggerModePtr->FromString("Off");
                }

                CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
                if (IsAvailable(exposureAutoPtr) && IsWritable(exposureAutoPtr))
                {
                    exposureAutoPtr->FromString("Continuous");
                    RCLCPP_INFO(this->get_logger(), "ExposureAuto set to Continuous.");
                }

                // 해상도: 3600x3600, 센서 5472x3648 기준 중앙 오프셋
                // OffsetX/Y를 먼저 0으로 리셋 후 Width/Height 설정 (순서 중요)
                CIntegerPtr offsetXPtr = nodeMap.GetNode("OffsetX");
                if (IsAvailable(offsetXPtr) && IsWritable(offsetXPtr)) offsetXPtr->SetValue(0);
                CIntegerPtr offsetYPtr = nodeMap.GetNode("OffsetY");
                if (IsAvailable(offsetYPtr) && IsWritable(offsetYPtr)) offsetYPtr->SetValue(0);

                CIntegerPtr widthPtr = nodeMap.GetNode("Width");
                if (IsAvailable(widthPtr) && IsWritable(widthPtr)) widthPtr->SetValue(3600);
                CIntegerPtr heightPtr = nodeMap.GetNode("Height");
                if (IsAvailable(heightPtr) && IsWritable(heightPtr)) heightPtr->SetValue(3600);

                // 중앙 오프셋 적용: (5472-3600)/2=936, (3648-3600)/2=24
                if (IsAvailable(offsetXPtr) && IsWritable(offsetXPtr)) offsetXPtr->SetValue(936);
                if (IsAvailable(offsetYPtr) && IsWritable(offsetYPtr)) offsetYPtr->SetValue(24);

                RCLCPP_INFO(this->get_logger(), "Resolution: 3600x3600, Offset: (936, 24)");

                CEnumerationPtr pixelFormatPtr = nodeMap.GetNode("PixelFormat");
                if (IsAvailable(pixelFormatPtr) && IsWritable(pixelFormatPtr)) pixelFormatPtr->FromString("Mono8");

                // GigE: AcquisitionFrameRateEnable을 먼저 활성화해야 프레임레이트 설정 가능
                CBooleanPtr frameRateEnablePtr = nodeMap.GetNode("AcquisitionFrameRateEnable");
                if (IsAvailable(frameRateEnablePtr) && IsWritable(frameRateEnablePtr))
                {
                    frameRateEnablePtr->SetValue(true);
                    RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRateEnable = true");
                }

                CFloatPtr frameRatePtr = nodeMap.GetNode("AcquisitionFrameRate");
                if (IsAvailable(frameRatePtr) && IsWritable(frameRatePtr))
                {
                    frameRatePtr->SetValue(1.0);
                    RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate=1fps");
                }
            }

            cam_->BeginAcquisition();
            RCLCPP_INFO(this->get_logger(), "Camera acquisition started (Continuous).");

            // 스트림 안정화 대기 — 첫 프레임 전송 전 카메라 내부 초기화 시간 확보
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            RCLCPP_INFO(this->get_logger(), "Stream stabilization wait complete.");

            error_occurred_ = false;
            last_error_msg_ = "";
        }
        catch (const Spinnaker::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "setupCameraAndStart() exception: %s", e.what());
            error_occurred_ = true;
            last_error_msg_ = std::string("Camera setup failed: ") + e.what();
        }
    }
    // 워치독: 장시간 프레임 업데이트 없으면 카메라 재초기화
    void watchdogCheck()
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_good_frame_time_).count();

        if (elapsed > 60) {
            RCLCPP_ERROR(this->get_logger(),
                "Watchdog: no good frame for %ld seconds, reinitializing camera...",
                (long)elapsed);
            last_good_frame_time_ = now;  // 재초기화 중 반복 방지
            spinnakerFullReinit();
        }
    }

    void spinnakerFullReinit()
    {
        RCLCPP_INFO(this->get_logger(), "Attempting full Spinnaker re-init...");
        if (cam_) {
            try { cam_->EndAcquisition(); } catch (...) { }
            try { cam_->DeInit(); } catch (...) { }
        }
        cam_list_.Clear();
        try { system_->ReleaseInstance(); } catch (...) { }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 재초기화 시에도 MTU 확인
        configureNetworkForGigE();

        system_ = System::GetInstance();
        cam_list_ = system_->GetCameras();
        if (cam_list_.GetSize() < 1) {
            RCLCPP_ERROR(this->get_logger(), "No cameras detected after full re-init. Will keep trying next time...");
            error_occurred_ = true;
            last_error_msg_ = "No cameras detected after re-init";
            return;
        }
        cam_ = nullptr;
        for (unsigned int i = 0; i < cam_list_.GetSize(); i++)
        {
            CameraPtr tmpCam = cam_list_.GetByIndex(i);
            INodeMap &tlNodeMap = tmpCam->GetTLDeviceNodeMap();
            CStringPtr serialPtr = tlNodeMap.GetNode("DeviceSerialNumber");
            if (IsAvailable(serialPtr) && IsReadable(serialPtr))
            {
                std::string serial = serialPtr->GetValue().c_str();
                if (serial == "18566179") { cam_ = tmpCam; break; }
            }
        }
        if (!cam_) cam_ = cam_list_.GetByIndex(0);

        // 서브넷 불일치 대비: Auto ForceIP 실행 후 영구 IP 재설정
        autoForceIP();
        configurePersistentIP();
        setupCameraAndStart();
    }

    // 이미지를 획득하여 공유 포인터 버퍼에 저장하는 함수
    void grabAndStoreFrame()
    {
        if (!cam_ || !cam_->IsStreaming()) {
            RCLCPP_WARN(this->get_logger(), "Camera not ready, skipping frame grab.");
            if (!error_occurred_) {
                 error_occurred_ = true;
                 last_error_msg_ = "Camera not streaming or not ready";
            }
            consecutive_fail_count_++;
            if (consecutive_fail_count_ >= max_consecutive_fails_) {
                RCLCPP_ERROR(this->get_logger(),
                    "Camera not streaming for %d consecutive attempts, reinitializing...",
                    consecutive_fail_count_);
                consecutive_fail_count_ = 0;
                spinnakerFullReinit();
            }
            return;
        }

        try
        {
            ImagePtr pResultImage = cam_->GetNextImage(10000);

            if (pResultImage->IsIncomplete()) {
                auto status = pResultImage->GetImageStatus();
                RCLCPP_WARN(this->get_logger(), "Image incomplete (status=%d: %s)",
                    (int)status, Image::GetImageStatusDescription(status));
                pResultImage->Release();
                consecutive_fail_count_++;
                if (consecutive_fail_count_ >= max_consecutive_fails_) {
                    RCLCPP_ERROR(this->get_logger(),
                        "%d consecutive incomplete images, reinitializing camera...",
                        consecutive_fail_count_);
                    consecutive_fail_count_ = 0;
                    spinnakerFullReinit();
                }
                return;
            }
            cv::Mat mono(pResultImage->GetHeight(), pResultImage->GetWidth(), CV_8UC1, pResultImage->GetData(), pResultImage->GetStride());

            auto jpgBuf = std::make_shared<std::vector<uchar>>();
            std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 60};
            if (!cv::imencode(".jpg", mono, *jpgBuf, jpegParams)) {
                RCLCPP_WARN(this->get_logger(), "Failed to encode to JPG.");
                pResultImage->Release();
                return;
            }

            pResultImage->Release();

            // 공유 버퍼에 최신 프레임 저장 + 프레임 번호 증가
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_jpeg_buffer_ = jpgBuf;
                frame_seq_++;
            }
            frame_cv_.notify_all();

            consecutive_fail_count_ = 0;
            last_good_frame_time_ = std::chrono::steady_clock::now();

            if (error_occurred_) {
                error_occurred_ = false;
                last_error_msg_ = "";
            }
        }
        catch (const Spinnaker::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "GetNextImage exception: %s", e.what());
            error_occurred_ = true;
            last_error_msg_ = std::string("GetNextImage failed: ") + e.what();
            consecutive_fail_count_ = 0;
            spinnakerFullReinit();
        }
    }
    
    void startHttpServer() {
        server_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "socket() failed.");
            error_occurred_ = true;
            last_error_msg_ = "MJPEG server socket creation failed";
            return;
        }

        int opt = 1;
        setsockopt(server_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(server_port_);

        if (bind(server_sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "bind() failed for port %d", server_port_);
            error_occurred_ = true;
            last_error_msg_ = "MJPEG server bind failed";
            return;
        }
        if (listen(server_sockfd_, 5) < 0) {
            RCLCPP_ERROR(this->get_logger(), "listen() failed.");
            error_occurred_ = true;
            last_error_msg_ = "MJPEG server listen failed";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MJPEG server listening on port %d. Open http://<IP>:%d", server_port_, server_port_);

        while (rclcpp::ok()) {
            sockaddr_in cli_addr;
            socklen_t clilen = sizeof(cli_addr);
            int client_sockfd = accept(server_sockfd_, (struct sockaddr *)&cli_addr, &clilen);
            if (client_sockfd < 0) {
                if (!rclcpp::ok()) break;
                RCLCPP_WARN(this->get_logger(), "accept() failed.");
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Client connected from %s", inet_ntoa(cli_addr.sin_addr));
            std::thread(&CameraTcpStreamerNode::handleClientConnection, this, client_sockfd).detach();
        }
    }

    // 클라이언트에게 MJPEG 스트림을 전송하는 함수
    void handleClientConnection(int client_sockfd) {
        std::string header = "HTTP/1.1 200 OK\r\n"
                            "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
                            "Connection: close\r\n"
                            "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                            "Pragma: no-cache\r\n"
                            "Expires: 0\r\n\r\n";
        if (send(client_sockfd, header.c_str(), header.length(), MSG_NOSIGNAL) < 0) {
            close(client_sockfd);
            return;
        }

        uint64_t last_sent_seq = 0;

        while (rclcpp::ok()) {
            std::shared_ptr<std::vector<uchar>> jpg_buffer_ptr;
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                // 새 프레임이 올 때까지 대기 (최대 5초 타임아웃)
                bool got_frame = frame_cv_.wait_for(lock, std::chrono::seconds(5),
                    [this, last_sent_seq]{ return frame_seq_ > last_sent_seq; });
                if (!got_frame) {
                    // 5초간 새 프레임 없음 — 연결 유지하면서 계속 대기
                    continue;
                }
                jpg_buffer_ptr = latest_jpeg_buffer_;
                last_sent_seq = frame_seq_;
            }

            if (!jpg_buffer_ptr) continue;

            std::string frame_header = "\r\n--frame\r\n"
                                    "Content-Type: image/jpeg\r\n"
                                    "Content-Length: " + std::to_string(jpg_buffer_ptr->size()) + "\r\n\r\n";

            if (send(client_sockfd, frame_header.c_str(), frame_header.length(), MSG_NOSIGNAL) < 0) break;
            if (send(client_sockfd, jpg_buffer_ptr->data(), jpg_buffer_ptr->size(), MSG_NOSIGNAL) < 0) break;
        }
        RCLCPP_INFO(this->get_logger(), "Client disconnected.");
        close(client_sockfd);
    }
private:
    std::string camera_name_;
    int server_port_;
    SystemPtr system_;
    CameraList cam_list_;
    CameraPtr cam_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_control_subscription_;
    
    int server_sockfd_{-1};
    std::thread server_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;

    std::shared_ptr<std::vector<uchar>> latest_jpeg_buffer_;
    uint64_t frame_seq_{0};  // 프레임 시퀀스 번호 — 클라이언트가 새 프레임만 전송하도록

    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    bool error_occurred_{false};
    std::string last_error_msg_{""};

    // 연속 실패 감지 → 자동 재초기화
    int consecutive_fail_count_{0};
    static constexpr int max_consecutive_fails_{10};

    // 프레임 정체 감시
    std::chrono::steady_clock::time_point last_good_frame_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // ... (main 함수는 이전과 동일) ...
    std::string camera_name = "Robot_Local";
    int server_port = 9100;

    auto node = std::make_shared<CameraTcpStreamerNode>(camera_name, server_port);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
