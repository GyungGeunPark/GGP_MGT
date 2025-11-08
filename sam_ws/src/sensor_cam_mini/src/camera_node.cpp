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
        cam_ = cam_list_.GetByIndex(0);

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
    void setupCameraAndStart()
    {
        if (!cam_) return;

        try
        {
            cam_->Init();
            {
                INodeMap &sNodeMap = cam_->GetTLStreamNodeMap();
                CEnumerationPtr bufferHandlingPtr = sNodeMap.GetNode("StreamBufferHandlingMode");
                if (IsAvailable(bufferHandlingPtr) && IsWritable(bufferHandlingPtr))
                {
                    bufferHandlingPtr->FromString("NewestOnly");
                }
            }
            {
                INodeMap &nodeMap = cam_->GetNodeMap();
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

                CIntegerPtr widthPtr = nodeMap.GetNode("Width");
                if (IsAvailable(widthPtr) && IsWritable(widthPtr)) widthPtr->SetValue(4000);
                CIntegerPtr heightPtr = nodeMap.GetNode("Height");
                if (IsAvailable(heightPtr) && IsWritable(heightPtr)) heightPtr->SetValue(4000);
                CIntegerPtr offsetXPtr = nodeMap.GetNode("OffsetX");
                if (IsAvailable(offsetXPtr) && IsWritable(offsetXPtr)) offsetXPtr->SetValue(660);
                CIntegerPtr offsetYPtr = nodeMap.GetNode("OffsetY");
                if (IsAvailable(offsetYPtr) && IsWritable(offsetYPtr)) offsetYPtr->SetValue(300);
                CEnumerationPtr pixelFormatPtr = nodeMap.GetNode("PixelFormat");
                if (IsAvailable(pixelFormatPtr) && IsWritable(pixelFormatPtr)) pixelFormatPtr->FromString("Mono8");

                CFloatPtr frameRatePtr = nodeMap.GetNode("AcquisitionFrameRate");
                if (IsAvailable(frameRatePtr) && IsWritable(frameRatePtr))
                {
                    frameRatePtr->SetValue(1.0);
                    RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRate=1fps");
                }
            }

            cam_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
            cam_->BeginAcquisition();
            RCLCPP_INFO(this->get_logger(), "Camera acquisition started (Continuous).");

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
        system_ = System::GetInstance();
        cam_list_ = system_->GetCameras();
        if (cam_list_.GetSize() < 1) {
            RCLCPP_ERROR(this->get_logger(), "No cameras detected after full re-init. Will keep trying next time...");
            error_occurred_ = true;
            last_error_msg_ = "No cameras detected after re-init";
            return;
        }
        cam_ = cam_list_.GetByIndex(0);
        setupCameraAndStart();
    }

    // [최적화] 이미지를 획득하여 공유 포인터 버퍼에 저장하는 함수
    void grabAndStoreFrame()
    {
        if (!cam_ || !cam_->IsStreaming()) {
            RCLCPP_WARN(this->get_logger(), "Camera not ready, skipping frame grab.");
            if (!error_occurred_) { 
                 error_occurred_ = true;
                 last_error_msg_ = "Camera not streaming or not ready";
            }
            return;
        }

        try
        {
            ImagePtr pResultImage = cam_->GetNextImage(1100);

            if (pResultImage->IsIncomplete()) {
                RCLCPP_WARN(this->get_logger(), "Image incomplete");
                pResultImage->Release();
                return;
            }
            cv::Mat mono(pResultImage->GetHeight(), pResultImage->GetWidth(), CV_8UC1, pResultImage->GetData(), pResultImage->GetStride());
            
            // [최적화] JPEG 품질을 60으로 낮춰 CPU 부하 감소
            auto jpgBuf = std::make_shared<std::vector<uchar>>();
            std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 60}; 
            if (!cv::imencode(".jpg", mono, *jpgBuf, jpegParams)) {
                RCLCPP_WARN(this->get_logger(), "Failed to encode to JPG.");
                pResultImage->Release();
                return;
            }
            
            pResultImage->Release();

            // [최적화] 공유 버퍼에 최신 프레임의 '포인터'를 저장 (메모리 복사 없음)
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_jpeg_buffer_ = jpgBuf;
            }
            frame_cv_.notify_all();

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

    // [최적화] 클라이언트에게 MJPEG 스트림을 전송하는 함수 (메모리 복사 최소화)
    void handleClientConnection(int client_sockfd) {
        std::string header = "HTTP/1.1 200 OK\r\n"
                            "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
                            "Connection: close\r\n"
                            "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                            "Pragma: no-cache\r\n"
                            "Expires: 0\r\n\r\n";
        // [수정] SIGPIPE 방지를 위해 MSG_NOSIGNAL 플래그 추가
        if (send(client_sockfd, header.c_str(), header.length(), MSG_NOSIGNAL) < 0) {
            close(client_sockfd);
            return;
        }

        while (rclcpp::ok()) {
            std::shared_ptr<std::vector<uchar>> jpg_buffer_ptr;
            {
                std::unique_lock<std::mutex> lock(frame_mutex_);
                frame_cv_.wait(lock, [this]{ return latest_jpeg_buffer_ != nullptr; });
                jpg_buffer_ptr = latest_jpeg_buffer_;
            }

            std::string frame_header = "\r\n--frame\r\n"
                                    "Content-Type: image/jpeg\r\n"
                                    "Content-Length: " + std::to_string(jpg_buffer_ptr->size()) + "\r\n\r\n";
            
            // [수정] SIGPIPE 방지를 위해 MSG_NOSIGNAL 플래그 추가
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

    // [최적화] std::vector 대신 std::shared_ptr<std::vector>를 사용하여 메모리 복사를 방지
    std::shared_ptr<std::vector<uchar>> latest_jpeg_buffer_;

    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    bool error_occurred_{false};
    std::string last_error_msg_{""};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // ... (main 함수는 이전과 동일) ...
    std::string camera_name = "robot_local";
    int server_port = 9100;

    auto node = std::make_shared<CameraTcpStreamerNode>(camera_name, server_port);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}