#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <cmath>

class LidarNode : public rclcpp::Node {
public:
    LidarNode()
        : Node("lidar_node"),
          accumulate_duration_(5), // 누적 시간(초)
          delete_threshold_(15.0)    // 오래된 PCD 삭제 임계치(초)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_1", 10,
            std::bind(&LidarNode::lidarCallback, this, std::placeholders::_1));

        angle_min_ = 0.0;
        angle_max_ = 2.0 * M_PI;
        angle_increment_ = M_PI / 180.0;
        range_min_ = 0.1;
        range_max_ = 100.0;

        shared_folder_path_ = "/root/sam_ws/storage/lidar_img1";
        std::filesystem::create_directories(shared_folder_path_);

        accumulated_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        last_accum_time_ = this->now();
    }

private:
    void removeOldPcdFiles(const std::string& folder_path, double seconds_threshold)
    {
        auto now_sctp = std::chrono::system_clock::now();
        for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                auto ftime = std::filesystem::last_write_time(entry.path());
                auto ftime_sctp = std::chrono::system_clock::time_point{
                    std::chrono::duration_cast<std::chrono::system_clock::duration>(
                        ftime.time_since_epoch()
                        - std::filesystem::file_time_type::clock::now().time_since_epoch()
                        + now_sctp.time_since_epoch())
                };
                auto age_sec = std::chrono::duration_cast<std::chrono::seconds>(now_sctp - ftime_sctp).count();
                if (age_sec >= static_cast<long>(seconds_threshold)) {
                    std::filesystem::remove(entry.path());
                    RCLCPP_INFO(this->get_logger(), "Removed old PCD file: %s", entry.path().c_str());
                }
            }
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        accumulated_cloud_->insert(accumulated_cloud_->end(), temp_cloud.begin(), temp_cloud.end());

        rclcpp::Time current_time = this->now();
        if ((current_time - last_accum_time_).seconds() >= accumulate_duration_)
        {
            // 2D 시각화 및 LaserScan 변환 로직 (기존과 동일)
            int beam_count = static_cast<int>((angle_max_ - angle_min_) / angle_increment_);
            std::vector<float> ranges(beam_count, range_max_);
            for (auto &p : accumulated_cloud_->points) {
                float angle = std::atan2(p.y, p.x);
                if (angle < 0) angle += 2.0f * M_PI;
                float dist = std::sqrt(p.x * p.x + p.y * p.y);
                if (dist >= range_min_ && dist <= range_max_) {
                    int idx = static_cast<int>((angle - angle_min_) / angle_increment_);
                    if (idx >= 0 && idx < beam_count) {
                        if (dist < ranges[idx]) ranges[idx] = dist;
                    }
                }
            }
            
            // 2D 이미지 생성 (기존과 동일)
            cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);
            const int center_x = 300, center_y = 300;
            float scale = 3.0f;
            for (int i = 0; i < beam_count; ++i) {
                float r = ranges[i];
                if (r < range_max_) {
                    float angle = angle_min_ + i * angle_increment_;
                    int px = static_cast<int>(center_x + r * scale * std::cos(angle));
                    int py = static_cast<int>(center_y - r * scale * std::sin(angle));
                    if (px >= 0 && px < 600 && py >= 0 && py < 600)
                        cv::circle(image, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
                }
            }
            for (int ring_m = 10; ring_m <= 50; ring_m += 10) {
                int radius = static_cast<int>(ring_m * scale);
                cv::circle(image, cv::Point(center_x, center_y), radius, cv::Scalar(255, 255, 255), 1);
            }

            // ==============================
            // 3C) PCD 파일로 저장 (수정된 부분)
            // ==============================
            auto sysnow = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(sysnow);
            std::tm tm_buf;
            localtime_r(&t, &tm_buf);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(sysnow.time_since_epoch()).count() % 1000;

            std::ostringstream filename;
            filename << shared_folder_path_ << "/lidar_cloud_"
                     << std::put_time(&tm_buf, "%Y%m%d_%H%M%S_")
                     << std::setw(3) << std::setfill('0') << ms
                     << ".pcd";

            // 포인트가 하나 이상 있을 때만 ASCII 형식으로 저장
            if (!accumulated_cloud_->points.empty()) {
                int result = pcl::io::savePCDFileASCII(filename.str(), *accumulated_cloud_); // Binary -> ASCII
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "Saved PCD file: %s", filename.str().c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to save PCD file: %s", filename.str().c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty. Skipping save.");
            }
            
            removeOldPcdFiles(shared_folder_path_, delete_threshold_);
            accumulated_cloud_->clear();
            last_accum_time_ = current_time;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    double angle_min_, angle_max_, angle_increment_, range_min_, range_max_;
    double accumulate_duration_, delete_threshold_;
    std::string shared_folder_path_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    rclcpp::Time last_accum_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}