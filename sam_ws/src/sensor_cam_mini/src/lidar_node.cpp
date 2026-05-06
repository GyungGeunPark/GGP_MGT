#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <vector>
#include <array>
#include <fstream>
#include <filesystem>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace fs = std::filesystem;

class LidarNode : public rclcpp::Node {
public:
    LidarNode()
        : Node("lidar_node"),
          collecting_(false),
          point_count_(0)
    {
        // 로컬 PCD 저장 디렉터리
        // Jetson 도커 내부 경로: /root/sam_ws (볼륨 마운트)
        // → Jetson 호스트: /home/sam-cam-s1/sam_ws (SCP 접근 경로)
        storage_dir_ = "/root/sam_ws/storage/lidar_img1";

        // 시작 시 저장 폴더 초기화 (기존 파일 모두 삭제)
        cleanup_storage_folder();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_1", 10,
            std::bind(&LidarNode::lidarCallback, this, std::placeholders::_1));

        collect_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/lidar_collect",
            std::bind(&LidarNode::collectServiceCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
            "LidarNode started (Option B: local PCD save). Storage: %s",
            storage_dir_.c_str());
    }

private:
    // ── 시작 시 저장 폴더 초기화 ──
    void cleanup_storage_folder()
    {
        try {
            if (fs::exists(storage_dir_)) {
                size_t count = 0;
                for (auto& entry : fs::directory_iterator(storage_dir_)) {
                    if (entry.path().extension() == ".pcd") {
                        fs::remove(entry.path());
                        count++;
                    }
                }
                RCLCPP_INFO(this->get_logger(),
                    "Storage folder initialized: deleted %zu old PCD files", count);
            } else {
                fs::create_directories(storage_dir_);
                RCLCPP_INFO(this->get_logger(),
                    "Storage folder created: %s", storage_dir_.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to initialize storage folder: %s", e.what());
        }
    }

    // ── 300초 이상 지난 PCD 파일 삭제 ──
    void cleanup_old_pcd_files()
    {
        try {
            if (!fs::exists(storage_dir_)) return;

            auto now = fs::file_time_type::clock::now();
            size_t count = 0;

            for (auto& entry : fs::directory_iterator(storage_dir_)) {
                if (entry.path().extension() != ".pcd") continue;

                auto file_time = fs::last_write_time(entry.path());
                auto age = std::chrono::duration_cast<std::chrono::seconds>(
                    now - file_time).count();

                if (age > 300) {
                    fs::remove(entry.path());
                    count++;
                    RCLCPP_INFO(this->get_logger(),
                        "Deleted old PCD file (age=%lds): %s",
                        age, entry.path().filename().c_str());
                }
            }

            if (count > 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Cleaned up %zu old PCD files (>300s)", count);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                "Error during old file cleanup: %s", e.what());
        }
    }

    // ── PCD 파일 저장 (Binary 형식, PCL 의존성 없음) ──
    std::string save_pcd_file()
    {
        std::lock_guard<std::mutex> lock(points_mutex_);

        // 타임스탬프 생성
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        std::tm tm_buf;
        localtime_r(&time_t_now, &tm_buf);

        std::ostringstream oss;
        oss << storage_dir_ << "/lidar_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
            << "_" << std::setfill('0') << std::setw(3) << ms.count()
            << ".pcd";
        std::string filepath = oss.str();

        // PCD Binary 포맷으로 저장 (ASCII 대비 5~10배 빠름, 파일 크기 60% 감소)
        std::ofstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open file for writing: %s", filepath.c_str());
            return "";
        }

        size_t num_points = accumulated_points_.size();

        // PCD 헤더 (텍스트)
        std::ostringstream header;
        header << "# .PCD v0.7 - Point Cloud Data file format\n";
        header << "FIELDS x y z\n";
        header << "SIZE 4 4 4\n";
        header << "TYPE F F F\n";
        header << "COUNT 1 1 1\n";
        header << "WIDTH " << num_points << "\n";
        header << "HEIGHT 1\n";
        header << "VIEWPOINT 0 0 0 1 0 0 0\n";
        header << "POINTS " << num_points << "\n";
        header << "DATA binary\n";

        std::string header_str = header.str();
        file.write(header_str.c_str(), header_str.size());

        // 바이너리 포인트 데이터 (12 bytes per point: 3 × float32)
        // accumulated_points_는 std::vector<std::array<float,3>> 이므로 연속 메모리
        if (num_points > 0) {
            file.write(reinterpret_cast<const char*>(accumulated_points_.data()),
                       num_points * sizeof(std::array<float, 3>));
        }

        file.close();

        RCLCPP_INFO(this->get_logger(),
            "PCD saved: %zu points -> %s", num_points, filepath.c_str());

        return filepath;
    }

    // ── 서비스 콜백 ──
    void collectServiceCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            // === 수집 시작 ===
            if (collecting_) {
                response->success = false;
                response->message = "Already collecting.";
                return;
            }

            // 누적 데이터 초기화
            {
                std::lock_guard<std::mutex> lock(points_mutex_);
                accumulated_points_.clear();
                accumulated_points_.reserve(10000000);  // ~10M points 예약 (~115MB)
            }
            point_count_ = 0;
            collecting_ = true;
            collect_start_time_ = this->now();

            response->success = true;
            response->message = "Collection STARTED.";
            RCLCPP_INFO(this->get_logger(), ">>> Collection STARTED <<<");

        } else {
            // === 수집 중지 + 로컬 PCD 저장 ===
            if (!collecting_) {
                response->success = false;
                response->message = "Not currently collecting.";
                return;
            }

            collecting_ = false;
            double duration_sec = (this->now() - collect_start_time_).seconds();
            size_t count = point_count_.load();

            // 로컬 PCD 파일 저장
            std::string saved_path = save_pcd_file();

            // 오래된 파일 정리
            cleanup_old_pcd_files();

            if (saved_path.empty()) {
                response->success = false;
                response->message = "Collection stopped but PCD save FAILED. " +
                    std::to_string(count) + " points (" +
                    std::to_string(duration_sec) + "s)";
            } else {
                response->success = true;
                // 응답 형식: "STOPPED:<파일경로>:<포인트수>"
                response->message = "STOPPED:" + saved_path + ":" + std::to_string(count);
            }

            RCLCPP_INFO(this->get_logger(),
                ">>> Collection STOPPED. %zu points (%.1fs) -> %s <<<",
                count, duration_sec, saved_path.c_str());
        }
    }

    // ── 라이다 콜백: 포인트 누적 ──
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!collecting_) {
            return;
        }

        // PointCloud2에서 x, y, z 오프셋 찾기
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            return;
        }

        size_t num_points = msg->width * msg->height;
        uint32_t point_step = msg->point_step;
        const uint8_t* data_ptr = msg->data.data();

        // 포인트 추출 및 누적
        std::vector<std::array<float, 3>> chunk;
        chunk.reserve(num_points);

        for (size_t i = 0; i < num_points; i++) {
            const uint8_t* point_ptr = data_ptr + i * point_step;
            float x, y, z;
            std::memcpy(&x, point_ptr + x_offset, sizeof(float));
            std::memcpy(&y, point_ptr + y_offset, sizeof(float));
            std::memcpy(&z, point_ptr + z_offset, sizeof(float));

            // NaN 스킵
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                continue;
            }

            chunk.push_back({x, y, z});
        }

        if (!chunk.empty()) {
            std::lock_guard<std::mutex> lock(points_mutex_);
            accumulated_points_.insert(
                accumulated_points_.end(), chunk.begin(), chunk.end());
        }

        point_count_ += num_points;
    }

    // ── 멤버 변수 ──
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collect_service_;

    std::atomic<bool> collecting_;
    std::atomic<size_t> point_count_;
    rclcpp::Time collect_start_time_;

    std::string storage_dir_;
    std::mutex points_mutex_;
    std::vector<std::array<float, 3>> accumulated_points_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
