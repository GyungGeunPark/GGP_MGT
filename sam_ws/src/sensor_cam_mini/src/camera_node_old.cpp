#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

// 스피네이커 관련 헤더
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <thread>

// 소켓 통신 관련
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>   // inet_addr
#include <unistd.h>      // close
#include <cstring>       // memset
#include <vector>
#include <netinet/tcp.h>  // TCP_NODELAY 정의를 위한 헤더

// ROS2 메시지 타입
#include "std_msgs/msg/string.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

class CameraTcpStreamerNode : public rclcpp::Node
{
public:
    CameraTcpStreamerNode(const std::string& server_ip = "127.0.0.1", int server_port = 9100)
        : Node("camera_tcp_streamer_node")
    {
        RCLCPP_INFO(this->get_logger(), "CameraTcpStreamerNode constructor called.");

        // 서버 IP와 포트 저장
        server_ip_ = server_ip;
        server_port_ = server_port;

        // (1) 스피네이커 초기화
        system_ = System::GetInstance();
        cam_list_ = system_->GetCameras();
        if (cam_list_.GetSize() < 1)
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras detected");
            return;
        }
        cam_ = cam_list_.GetByIndex(0);

        // 카메라 설정 & 시작
        setupCameraAndStart();

        RCLCPP_INFO(this->get_logger(), "Connecting to server at %s:%d", server_ip.c_str(), server_port);
        if (!connectToServer(server_ip, server_port))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to TCP server (Python).");
        }

        // (3) 카메라 제어 명령 구독자 추가
        camera_control_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/camera_control", 10,
            std::bind(&CameraTcpStreamerNode::cameraControlCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera_control topic");

        // (4) 타이머로 주기적 이미지 캡처 & 소켓 전송
        //     10fps → 100ms 간격
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(333),
            std::bind(&CameraTcpStreamerNode::grabFrameAndSend, this)
        );
    }

    ~CameraTcpStreamerNode()
    {
        // 카메라 및 스피네이커 정리
        if (cam_)
        {
            try
            {
                cam_->EndAcquisition();
                cam_->DeInit();
            }
            catch(...)
            {
                // 무시
            }
        }
        cam_list_.Clear();
        system_->ReleaseInstance();

        // 소켓 닫기
        if (sockfd_ >= 0)
        {
            close(sockfd_);
            sockfd_ = -1;
        }
    }

private:
    // ------------------------------------------------------
    // 카메라 제어 명령 처리 콜백
    // ------------------------------------------------------
    void cameraControlCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received camera control command: %s", msg->data.c_str());
        
        std::string command = msg->data;
        
        // 명령 형식: "카메라명:set_gain:값"으로 변경
        size_t firstColon = command.find(':');
        if (firstColon != std::string::npos)
        {
            std::string camera_name = command.substr(0, firstColon);
            std::string remaining = command.substr(firstColon + 1);
            
            // gain 값 설정 명령 처리
            if (remaining.find("set_gain:") == 0)
            {
                try {
                    // 예: "set_gain:0.0" -> "0.0" 추출
                    std::string gain_str = remaining.substr(9);
                    float gain_value = std::stof(gain_str);
                    
                    RCLCPP_INFO(this->get_logger(), "Setting gain to %.1f for camera %s", 
                               gain_value, camera_name.c_str());
                    
                    // 카메라 gain 값 설정
                    if (cam_ && cam_->IsValid())
                    {
                        INodeMap &nodeMap = cam_->GetNodeMap();
                        CFloatPtr gainPtr = nodeMap.GetNode("Gain");
                        if (IsAvailable(gainPtr) && IsWritable(gainPtr))
                        {
                            // 현재 gain 값 확인 (로그용)
                            float current_gain = gainPtr->GetValue();
                            
                            // 새 gain 값 설정
                            gainPtr->SetValue(gain_value);
                            
                            // 설정 후 실제 값 확인
                            float actual_gain = gainPtr->GetValue();
                            
                            RCLCPP_INFO(this->get_logger(), 
                                       "Camera %s gain changed: %.1f -> %.1f (requested: %.1f)", 
                                       camera_name.c_str(), current_gain, actual_gain, gain_value);
                            
                            // 설정값과 실제값이 크게 다르면 경고
                            if (std::abs(actual_gain - gain_value) > 0.1) {
                                RCLCPP_WARN(this->get_logger(), 
                                            "Requested gain (%.1f) differs from actual gain (%.1f)",
                                            gain_value, actual_gain);
                            }
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "Gain node is not available or writable");
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Camera is not valid for gain adjustment");
                    }
                }
                catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error setting gain: %s", e.what());
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid command format. Expected 'camera_name:command'");
        }
    }

    // ------------------------------------------------------
    // (A) 카메라 Init + TransportLayer/GenICam 설정 + BeginAcquisition
    // ------------------------------------------------------
    void setupCameraAndStart()
    {
        if (!cam_)
        {
            RCLCPP_WARN(this->get_logger(), "setupCameraAndStart() called but cam_ is null!");
            return;
        }

        try
        {
            cam_->Init();

            // ========== Transport Layer ==========
            {
                INodeMap &sNodeMap = cam_->GetTLStreamNodeMap();

                // 1) EventMode=Polling (fallback to Off if Polling not available)
                {
                    CEnumerationPtr eventModePtr = sNodeMap.GetNode("EventMode");
                    if (IsAvailable(eventModePtr) && IsWritable(eventModePtr))
                    {
                        try
                        {
                            eventModePtr->FromString("Polling");
                            RCLCPP_INFO(this->get_logger(), "EventMode set to Polling");
                        }
                        catch (const Spinnaker::Exception &e)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "Failed to set EventMode=Polling: %s, fallback to Off",
                                        e.what());
                            try
                            {
                                eventModePtr->FromString("Off");
                                RCLCPP_INFO(this->get_logger(), "EventMode set to Off");
                            }
                            catch (...)
                            {
                                // 무시
                            }
                        }
                    }
                }

                // 2) StreamBufferHandlingMode=NewestOnly
                {
                    CEnumerationPtr bufferHandlingPtr = sNodeMap.GetNode("StreamBufferHandlingMode");
                    if (IsAvailable(bufferHandlingPtr) && IsWritable(bufferHandlingPtr))
                    {
                        bufferHandlingPtr->FromString("NewestOnly");
                        RCLCPP_INFO(this->get_logger(), "StreamBufferHandlingMode=NewestOnly");
                    }
                }

                // 3) 버퍼를 작게 유지(4장 정도)
                {
                    CEnumerationPtr bufferCountModePtr = sNodeMap.GetNode("StreamBufferCountMode");
                    if (IsAvailable(bufferCountModePtr) && IsWritable(bufferCountModePtr))
                    {
                        bufferCountModePtr->FromString("Manual");
                        RCLCPP_INFO(this->get_logger(), "BufferCountMode=Manual");
                    }
                    CIntegerPtr bufferCountPtr = sNodeMap.GetNode("StreamBufferCount");
                    if (IsAvailable(bufferCountPtr) && IsWritable(bufferCountPtr))
                    {
                        bufferCountPtr->SetValue(4);
                        RCLCPP_INFO(this->get_logger(), "StreamBufferCount=4");
                    }
                }
            }

            // ========== GenICam 노드맵 ==========
            {
                INodeMap &nodeMap = cam_->GetNodeMap();

                // (중요) 트리거 모드 Off → 카메라가 계속해서 프레임을 스트리밍
                {
                    CEnumerationPtr triggerModePtr = nodeMap.GetNode("TriggerMode");
                    if (IsAvailable(triggerModePtr) && IsWritable(triggerModePtr))
                    {
                        triggerModePtr->FromString("Off");
                        RCLCPP_INFO(this->get_logger(), "TriggerMode=Off (Continuous stream)");
                    }
                }

                // AutoExposure 꺼두기
                {
                    CEnumerationPtr exposureAutoPtr = cam_->ExposureAuto.GetNode();
                    if (IsAvailable(exposureAutoPtr) && IsWritable(exposureAutoPtr))
                    {
                        exposureAutoPtr->FromString("Off");
                    }
                }

                // 노출(ExposureTime) 설정. 예: 20ms
                {
                    CFloatPtr exposureTimePtr = cam_->ExposureTime.GetNode();
                    if (IsAvailable(exposureTimePtr) && IsWritable(exposureTimePtr))
                    {
                        exposureTimePtr->SetValue(5000.0); // 20ms
                        RCLCPP_INFO(this->get_logger(), "ExposureTime=50ms");
                    }
                }

                // 대역폭 제한 해제
                {
                    CIntegerPtr devThroughputPtr = nodeMap.GetNode("DeviceLinkThroughputLimit");
                    if (IsAvailable(devThroughputPtr) && IsWritable(devThroughputPtr))
                    {
                        devThroughputPtr->SetValue(devThroughputPtr->GetMax());
                        RCLCPP_INFO(this->get_logger(), "DeviceLinkThroughputLimit=Max");
                    }
                }

                // 해상도: BFS-U3-244S8M-C가 지원하는 최대 해상도 설정 (예: 5320 x 4600)
                {
                    CIntegerPtr widthPtr = nodeMap.GetNode("Width");
                    CIntegerPtr heightPtr = nodeMap.GetNode("Height");
                    if (IsAvailable(widthPtr) && IsWritable(widthPtr))
                    {
                        widthPtr->SetValue(4000);
                    }
                    if (IsAvailable(heightPtr) && IsWritable(heightPtr))
                    {
                        heightPtr->SetValue(4000);
                    }
                    // OffsetX, OffsetY는 0으로 세팅
                    CIntegerPtr offsetXPtr = nodeMap.GetNode("OffsetX");
                    CIntegerPtr offsetYPtr = nodeMap.GetNode("OffsetY");
                    if (IsAvailable(offsetXPtr) && IsWritable(offsetXPtr))
                    {
                        offsetXPtr->SetValue(660);
                    }
                    if (IsAvailable(offsetYPtr) && IsWritable(offsetYPtr))
                    {
                        offsetYPtr->SetValue(300);
                    }
                    CEnumerationPtr pixelFormatPtr = nodeMap.GetNode("PixelFormat");
                    if (IsAvailable(pixelFormatPtr) && IsWritable(pixelFormatPtr))
                    {
                        pixelFormatPtr->FromString("Mono8");
                        RCLCPP_INFO(this->get_logger(), "PixelFormat=Mono8");
                    }
                    RCLCPP_INFO(this->get_logger(), "Resolution set to 4000x4000 (Full Res)");
                }

                // Gain(ISO) 수동 = 0
                {
                    CEnumerationPtr gainAutoPtr = nodeMap.GetNode("GainAuto");
                    if (IsAvailable(gainAutoPtr) && IsWritable(gainAutoPtr))
                    {
                        gainAutoPtr->FromString("Off");
                    }
                    CFloatPtr gainPtr = nodeMap.GetNode("Gain");
                    if (IsAvailable(gainPtr) && IsWritable(gainPtr))
                    {
                        gainPtr->SetValue(6.0);
                        RCLCPP_INFO(this->get_logger(), "Gain=6.0");
                    }
                }

                // (중요) 내부 프레임레이트 10fps로 고정
                {
                    // FrameRateAuto가 있다면 Off
                    CEnumerationPtr frameRateAutoPtr = nodeMap.GetNode("AcquisitionFrameRateAuto");
                    if (IsAvailable(frameRateAutoPtr) && IsWritable(frameRateAutoPtr))
                    {
                        frameRateAutoPtr->FromString("Off");
                        RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRateAuto=Off");
                    }

                    // AcquisitionFrameRateEnable = true
                    CBooleanPtr frameRateEnablePtr = nodeMap.GetNode("AcquisitionFrameRateEnable");
                    if (IsAvailable(frameRateEnablePtr) && IsWritable(frameRateEnablePtr))
                    {
                        frameRateEnablePtr->SetValue(true);
                        RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRateEnable=On");
                    }

                    // AcquisitionFrameRate = 10.0
                    CFloatPtr frameRatePtr = nodeMap.GetNode("AcquisitionFrameRate");
                    if (IsAvailable(frameRatePtr) && IsWritable(frameRatePtr))
                    {
                        frameRatePtr->SetValue(3.0);
                        RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate=3fps");
                    }
                }
            }

            // 4) Acquisition 시작
            cam_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
            cam_->BeginAcquisition();
            RCLCPP_INFO(this->get_logger(), "Camera acquisition started (Continuous).");
        }
        catch (const Spinnaker::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "setupCameraAndStart() exception: %s", e.what());
        }
    }

    // ------------------------------------------------------
    // (B) Spinnaker Full ReInit (ReleaseInstance -> 재할당)
    // ------------------------------------------------------
    void spinnakerFullReinit()
    {
        RCLCPP_INFO(this->get_logger(), "Attempting full Spinnaker re-init...");

        // 1) 기존 카메라 중지/해제
        if (cam_)
        {
            try
            {
                cam_->EndAcquisition();
            }
            catch (...)
            {
            }
            try
            {
                cam_->DeInit();
            }
            catch (...)
            {
            }
        }
        cam_list_.Clear();

        try
        {
            system_->ReleaseInstance();
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to ReleaseInstance (possibly already closed).");
        }

        // 2) 잠깐 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 3) 새로 GetInstance
        system_ = System::GetInstance();
        cam_list_ = system_->GetCameras();
        if (cam_list_.GetSize() < 1)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "No cameras detected after full re-init. Will keep trying next time...");
            return;
        }

        // 4) 인덱스 0 카메라 재할당 & 재설정
        cam_ = cam_list_.GetByIndex(0);
        setupCameraAndStart();
    }

    // ------------------------------------------------------
    // (C) 이미지 획득 후, TCP 소켓으로 전송
    // ------------------------------------------------------
    void grabFrameAndSend()
    {
        // 디버그 레벨로만 표시
        RCLCPP_DEBUG(this->get_logger(), "grabFrameAndSend() called");

        if (!cam_)
        {
            RCLCPP_WARN(this->get_logger(), "cam_ is null - skipping frame grab");
            return;
        }

        if (sockfd_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "TCP socket not connected - skipping.");
            return;
        }

        const int maxRetries = 1;
        bool acquired = false;
        ImagePtr pResultImage = nullptr;

        for (int attempt = 0; attempt < maxRetries; ++attempt)
        {
            try
            {
                // 이미지 획득 (타임아웃 500ms)
                pResultImage = cam_->GetNextImage(1000); 
            }
            catch (const Spinnaker::Exception &e)
            {
                RCLCPP_WARN(this->get_logger(),
                            "GetNextImage exception (attempt %d): %s",
                            attempt + 1, e.what());
                std::string errStr(e.what());
                // 특정 에러코드 감지 시 재초기화
                if (errStr.find("-1015") != std::string::npos ||
                    errStr.find("Error writing to StreamPort") != std::string::npos ||
                    errStr.find("Please try reconnecting the device") != std::string::npos ||
                    errStr.find("-1011") != std::string::npos ||
                    errStr.find("Failed waiting for EventData on NEW_BUFFER_DATA event") != std::string::npos)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Detected [-1015 / -1011]. Full reinit...");
                    spinnakerFullReinit();
                    return;
                }
                if (errStr.find("Stream is not started") != std::string::npos ||
                    errStr.find("-1010") != std::string::npos)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Detected [-1010], reinit full system...");
                    spinnakerFullReinit();
                    return;
                }
                continue;
            }

            // 3) Check if incomplete
            if (!pResultImage || pResultImage->IsIncomplete())
            {
                if (pResultImage && pResultImage->IsIncomplete())
                {
                    auto status = pResultImage->GetImageStatus();
                    RCLCPP_WARN(this->get_logger(),
                                "Image incomplete (status %d) (attempt %d)",
                                static_cast<int>(status), attempt + 1);
                    pResultImage->Release();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                                "ImagePtr is null (attempt %d)", attempt + 1);
                }
                continue;
            }

            acquired = true;
            break;
        }

        if (!acquired)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to acquire image after %d attempts", maxRetries);
            return;
        }

        // → OpenCV Mat 변환
        cv::Mat mono(
            pResultImage->GetHeight(),
            pResultImage->GetWidth(),
            CV_8UC1,
            pResultImage->GetData(),
            pResultImage->GetStride()
        );

        pResultImage->Release();

        // → JPEG 인코딩 (gray)
        std::vector<uchar> jpgBuf;
        // 품질 옵션을 약간 낮추면(예: 80~90) 인코딩 및 전송 속도 개선 가능
        std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 85};
        if (!cv::imencode(".jpg", mono, jpgBuf, jpegParams))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to encode to JPG. Skipping frame.");
            return;
        }

        // → TCP 전송 (프레임 길이(4바이트) + 프레임 데이터)
        int frameSize = static_cast<int>(jpgBuf.size());
        int sentBytes = send(sockfd_, reinterpret_cast<char*>(&frameSize), sizeof(frameSize), 0);
        if (sentBytes < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send frame size. Maybe disconnected?");
            return;
        }
        sentBytes = send(sockfd_, reinterpret_cast<char*>(jpgBuf.data()), frameSize, 0);
        if (sentBytes < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send frame data. Maybe disconnected?");
            return;
        }

        // 디버그 레벨 로그
        RCLCPP_DEBUG(this->get_logger(), "Frame sent successfully: %d bytes", frameSize);
    }

    // ------------------------------------------------------
    // 소켓 연결
    // ------------------------------------------------------
    bool connectToServer(const std::string &ip, int port)
    {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "socket() failed.");
            return false;
        }

        sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr = inet_addr(ip.c_str());

        // TCP 소켓 버퍼 사이즈 증가
        int sendbuff = 262144;  // 256KB
        if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Couldn't set TCP send buffer size");
        }
        
        // No delay 설정 (Nagle 알고리즘 비활성화)
        int flag = 1;
        if (setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Couldn't set TCP_NODELAY");
        }

        // connect
        if (connect(sockfd_, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "connect() failed to %s:%d", ip.c_str(), port);
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to TCP server %s:%d", ip.c_str(), port);
        return true;
    }

    // ------------------------------------------------------
    // 멤버 변수
    // ------------------------------------------------------
    std::string server_ip_;
    int server_port_;
    
    SystemPtr system_;
    CameraList cam_list_;
    CameraPtr cam_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_control_subscription_;

    int sockfd_{-1}; // TCP 소켓 fd
};

// main 함수
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 명령행 인수에서 서버 IP와 포트 읽기
    std::string server_ip = "127.0.0.1";  // 기본값
    int server_port = 9100;              // 기본값
    
    if (argc > 1) {
        server_ip = argv[1];
    }
    if (argc > 2) {
        try {
            server_port = std::stoi(argv[2]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid port number: " << argv[2] << std::endl;
        }
    }
    
    // 파싱한 값을 생성자에 전달
    auto node = std::make_shared<CameraTcpStreamerNode>(server_ip, server_port);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
