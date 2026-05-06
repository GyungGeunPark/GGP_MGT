#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <chrono>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Livox PointXYZRTL point type definition (input from Livox driver)
struct PointXYZRTL
{
    PCL_ADD_POINT4D;
    float reflectivity;
    std::uint8_t tag;
    std::uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRTL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)
)

// Output point type matching bundle program format (mm, integer-like)
struct PointXYZI_mm
{
    PCL_ADD_POINT4D;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZI_mm,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

class DualLidarNode : public rclcpp::Node {
public:
    DualLidarNode()
        : Node("dual_lidar_node"),
          is_collecting_(false)
    {
        // Declare parameters (Mid-70 broadcast code based topics)
        // Note: livox_ros2_driver uses format /livox/lidar_1_<broadcast_code>
        this->declare_parameter<std::string>("left_topic", "/livox/lidar_1_3GGDN8700225691");
        this->declare_parameter<std::string>("right_topic", "/livox/lidar_1_3GGDN8700226431");
        this->declare_parameter<std::string>("save_path_left", "./pcd_file/left");
        this->declare_parameter<std::string>("save_path_right", "./pcd_file/right");
        this->declare_parameter<double>("accumulate_duration", 5.0);

        // Coordinate correction parameters
        this->declare_parameter<bool>("use_mm_scale", true);           // Convert to mm (bundle program format)
        this->declare_parameter<bool>("correct_right_yaw", true);      // Correct yaw=180 on right LiDAR
        this->declare_parameter<double>("right_yaw_correction", 180.0); // Yaw correction angle in degrees

        // Point filtering parameters
        this->declare_parameter<bool>("filter_invalid_points", true);  // Filter out (0,0,0) invalid points
        this->declare_parameter<double>("max_range", 10.0);            // Max range in meters (Mid-70: 40m)
        this->declare_parameter<double>("min_range", 0.1);             // Min range in meters

        // Bounding box filtering (in mm, applied after scale conversion)
        // Coordinate: x=forward(depth), y=lateral(side), z=vertical(not bounded)
        this->declare_parameter<bool>("use_bounding_box", false);      // Bounding box filter (disabled by default)
        // Left LiDAR bounding box
        this->declare_parameter<double>("left_x_max", 3000.0);         // mm
        this->declare_parameter<double>("left_y_min", -500.0);         // mm
        this->declare_parameter<double>("left_y_max", 1000.0);         // mm
        // Right LiDAR bounding box
        this->declare_parameter<double>("right_x_max", 3000.0);        // mm
        this->declare_parameter<double>("right_y_min", -1000.0);       // mm
        this->declare_parameter<double>("right_y_max", 500.0);         // mm

        // Get parameters
        left_topic_ = this->get_parameter("left_topic").as_string();
        right_topic_ = this->get_parameter("right_topic").as_string();
        save_path_left_ = this->get_parameter("save_path_left").as_string();
        save_path_right_ = this->get_parameter("save_path_right").as_string();
        accumulate_duration_ = this->get_parameter("accumulate_duration").as_double();
        use_mm_scale_ = this->get_parameter("use_mm_scale").as_bool();
        correct_right_yaw_ = this->get_parameter("correct_right_yaw").as_bool();
        right_yaw_correction_ = this->get_parameter("right_yaw_correction").as_double();
        filter_invalid_points_ = this->get_parameter("filter_invalid_points").as_bool();
        max_range_ = this->get_parameter("max_range").as_double();
        min_range_ = this->get_parameter("min_range").as_double();

        // Get bounding box parameters
        use_bounding_box_ = this->get_parameter("use_bounding_box").as_bool();
        left_x_max_ = this->get_parameter("left_x_max").as_double();
        left_y_min_ = this->get_parameter("left_y_min").as_double();
        left_y_max_ = this->get_parameter("left_y_max").as_double();
        right_x_max_ = this->get_parameter("right_x_max").as_double();
        right_y_min_ = this->get_parameter("right_y_min").as_double();
        right_y_max_ = this->get_parameter("right_y_max").as_double();

        // Resolve relative paths to absolute
        if (save_path_left_[0] != '/') {
            save_path_left_ = std::filesystem::current_path().string() + "/" + save_path_left_;
        }
        if (save_path_right_[0] != '/') {
            save_path_right_ = std::filesystem::current_path().string() + "/" + save_path_right_;
        }

        // Create save directories
        std::filesystem::create_directories(save_path_left_);
        std::filesystem::create_directories(save_path_right_);

        // Create subscribers
        left_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            left_topic_, 10,
            std::bind(&DualLidarNode::leftLidarCallback, this, std::placeholders::_1)
        );

        right_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            right_topic_, 10,
            std::bind(&DualLidarNode::rightLidarCallback, this, std::placeholders::_1)
        );

        // Create services
        start_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/dual_lidar/start_collection",
            std::bind(&DualLidarNode::startCollectionCallback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/dual_lidar/save_pointcloud",
            std::bind(&DualLidarNode::savePointcloudCallback, this,
                std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize point clouds (using PointXYZI_mm for output)
        left_cloud_ = pcl::make_shared<pcl::PointCloud<PointXYZI_mm>>();
        right_cloud_ = pcl::make_shared<pcl::PointCloud<PointXYZI_mm>>();

        // Set sensor origin (0,0,0) and orientation (identity) for each LiDAR
        // This will be saved as VIEWPOINT in the PCD file header
        left_cloud_->sensor_origin_ = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
        left_cloud_->sensor_orientation_ = Eigen::Quaternionf::Identity();

        right_cloud_->sensor_origin_ = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
        right_cloud_->sensor_orientation_ = Eigen::Quaternionf::Identity();

        // Precompute yaw correction rotation matrix for right LiDAR
        // This corrects for yaw=180 that may be applied by the Livox driver
        double yaw_rad = right_yaw_correction_ * M_PI / 180.0;
        cos_yaw_ = std::cos(yaw_rad);
        sin_yaw_ = std::sin(yaw_rad);

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Dual LiDAR Node Initialized (Mid-70)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Left topic: %s", left_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right topic: %s", right_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left save path: %s", save_path_left_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right save path: %s", save_path_right_.c_str());
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Coordinate Settings:");
        RCLCPP_INFO(this->get_logger(), "  Scale: %s", use_mm_scale_ ? "millimeters (mm)" : "meters (m)");
        RCLCPP_INFO(this->get_logger(), "  Right yaw correction: %s (%.1f deg)",
                    correct_right_yaw_ ? "ENABLED" : "disabled", right_yaw_correction_);
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Bounding Box Filter: %s", use_bounding_box_ ? "ENABLED" : "disabled");
        if (use_bounding_box_) {
            RCLCPP_INFO(this->get_logger(), "  Left LiDAR:  x=[0, %.0f], y=[%.0f, %.0f] mm",
                        left_x_max_, left_y_min_, left_y_max_);
            RCLCPP_INFO(this->get_logger(), "  Right LiDAR: x=[0, %.0f], y=[%.0f, %.0f] mm",
                        right_x_max_, right_y_min_, right_y_max_);
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Services:");
        RCLCPP_INFO(this->get_logger(), "  /dual_lidar/start_collection (SetBool)");
        RCLCPP_INFO(this->get_logger(), "  /dual_lidar/save_pointcloud (Trigger)");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    // Check if a point is valid (not at origin) - operates on raw meters
    bool isValidPoint(const PointXYZRTL& pt)
    {
        if (!filter_invalid_points_) return true;

        // Check for invalid (0,0,0) points
        if (pt.x == 0.0f && pt.y == 0.0f && pt.z == 0.0f) {
            return false;
        }

        // Distance-based range filter (in meters, before mm conversion)
        float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (distance < min_range_ || distance > max_range_) {
            return false;
        }

        return true;
    }

    // Check if a converted point (in mm) is within left LiDAR bounding box
    // Coordinate system: x=forward(depth), y=lateral(side), z=vertical(height)
    // Note: z-axis is not bounded (no height constraint)
    bool isInLeftBoundingBox(const PointXYZI_mm& pt)
    {
        if (!use_bounding_box_) return true;

        // x: forward direction (depth) - discard points behind LiDAR (x<0) and beyond x_max
        if (pt.x < 0.0f || pt.x > left_x_max_) {
            return false;
        }
        // y: lateral direction (left/right)
        if (pt.y < left_y_min_ || pt.y > left_y_max_) {
            return false;
        }
        return true;
    }

    // Check if a converted point (in mm) is within right LiDAR bounding box
    // Coordinate system: x=forward(depth), y=lateral(side), z=vertical(height)
    // Note: z-axis is not bounded (no height constraint)
    bool isInRightBoundingBox(const PointXYZI_mm& pt)
    {
        if (!use_bounding_box_) return true;

        // x: forward direction (depth) - discard points behind LiDAR (x<0) and beyond x_max
        if (pt.x < 0.0f || pt.x > right_x_max_) {
            return false;
        }
        // y: lateral direction (left/right)
        if (pt.y < right_y_min_ || pt.y > right_y_max_) {
            return false;
        }
        return true;
    }

    // Convert input point to output format with scale conversion
    PointXYZI_mm convertPoint(const PointXYZRTL& in, bool apply_yaw_correction = false)
    {
        PointXYZI_mm out;
        float x = in.x;
        float y = in.y;
        float z = in.z;

        // Apply yaw correction if needed (inverse rotation)
        // If driver applied yaw=180, we apply yaw=-180 (same as yaw=180) to cancel
        if (apply_yaw_correction) {
            float new_x = x * cos_yaw_ + y * sin_yaw_;
            float new_y = -x * sin_yaw_ + y * cos_yaw_;
            x = new_x;
            y = new_y;
        }

        // Convert to mm if enabled (bundle program format)
        if (use_mm_scale_) {
            out.x = x * 1000.0f;
            out.y = y * 1000.0f;
            out.z = z * 1000.0f;
        } else {
            out.x = x;
            out.y = y;
            out.z = z;
        }
        out.intensity = in.reflectivity;

        return out;
    }

    void leftLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_collecting_) return;

        std::lock_guard<std::mutex> lock(left_mutex_);
        pcl::PointCloud<PointXYZRTL> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);

        if (!temp_cloud.empty()) {
            // Convert each point to output format (no yaw correction for left)
            // Filter out invalid points (0,0,0), out-of-range, and outside bounding box
            size_t valid_count = 0;
            for (const auto& pt : temp_cloud) {
                if (isValidPoint(pt)) {
                    PointXYZI_mm converted = convertPoint(pt, false);
                    if (isInLeftBoundingBox(converted)) {
                        left_cloud_->push_back(converted);
                        valid_count++;
                    }
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "[LEFT] Received %zu points, Valid: %zu, Total: %zu",
                temp_cloud.size(), valid_count, left_cloud_->size());
        }
    }

    void rightLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_collecting_) return;

        std::lock_guard<std::mutex> lock(right_mutex_);
        pcl::PointCloud<PointXYZRTL> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);

        if (!temp_cloud.empty()) {
            // Convert each point to output format (apply yaw correction if enabled)
            // Filter out invalid points (0,0,0), out-of-range, and outside bounding box
            size_t valid_count = 0;
            for (const auto& pt : temp_cloud) {
                if (isValidPoint(pt)) {
                    PointXYZI_mm converted = convertPoint(pt, correct_right_yaw_);
                    if (isInRightBoundingBox(converted)) {
                        right_cloud_->push_back(converted);
                        valid_count++;
                    }
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "[RIGHT] Received %zu points, Valid: %zu, Total: %zu",
                temp_cloud.size(), valid_count, right_cloud_->size());
        }
    }

    void startCollectionCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            // Start collection - clear existing data but preserve sensor origin/orientation
            {
                std::lock_guard<std::mutex> lock_l(left_mutex_);
                std::lock_guard<std::mutex> lock_r(right_mutex_);

                // Save sensor origin/orientation before clearing
                auto left_origin = left_cloud_->sensor_origin_;
                auto left_orient = left_cloud_->sensor_orientation_;
                auto right_origin = right_cloud_->sensor_origin_;
                auto right_orient = right_cloud_->sensor_orientation_;

                left_cloud_->clear();
                right_cloud_->clear();

                // Restore sensor origin/orientation after clearing
                left_cloud_->sensor_origin_ = left_origin;
                left_cloud_->sensor_orientation_ = left_orient;
                right_cloud_->sensor_origin_ = right_origin;
                right_cloud_->sensor_orientation_ = right_orient;
            }
            is_collecting_ = true;
            collection_start_time_ = this->now();
            response->success = true;
            response->message = "Point cloud collection started";
            RCLCPP_INFO(this->get_logger(), ">>> Collection STARTED (scale: %s, right yaw correction: %s) <<<",
                        use_mm_scale_ ? "mm" : "m",
                        correct_right_yaw_ ? "ON" : "OFF");
        } else {
            // Stop collection
            is_collecting_ = false;
            response->success = true;
            response->message = "Point cloud collection stopped";
            RCLCPP_INFO(this->get_logger(), ">>> Collection STOPPED <<<");
        }
    }

    std::string generateTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&t, &tm_buf);

        std::ostringstream timestamp;
        // Format: YYMMDD_HHMMSS (e.g., 260130_123537)
        timestamp << std::setfill('0')
                  << std::setw(2) << (tm_buf.tm_year % 100)  // YY
                  << std::setw(2) << (tm_buf.tm_mon + 1)      // MM
                  << std::setw(2) << tm_buf.tm_mday           // DD
                  << "_"
                  << std::setw(2) << tm_buf.tm_hour           // HH
                  << std::setw(2) << tm_buf.tm_min            // MM
                  << std::setw(2) << tm_buf.tm_sec;           // SS

        return timestamp.str();
    }

    void savePointcloudCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        // Stop collection
        is_collecting_ = false;

        // Generate timestamp
        std::string timestamp = generateTimestamp();

        bool left_saved = false, right_saved = false;
        std::string left_file, right_file;
        size_t left_points = 0, right_points = 0;

        // Save Left point cloud
        {
            std::lock_guard<std::mutex> lock(left_mutex_);
            left_points = left_cloud_->size();
            if (!left_cloud_->empty()) {
                left_file = save_path_left_ + "/left_pcl_" + timestamp + ".pcd";
                // Use ASCII format to match bundle program output (human readable)
                if (pcl::io::savePCDFileASCII(left_file, *left_cloud_) == 0) {
                    left_saved = true;
                    RCLCPP_INFO(this->get_logger(), "[LEFT] Saved: %s (%zu points, format: %s)",
                        left_file.c_str(), left_cloud_->size(),
                        use_mm_scale_ ? "mm" : "m");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[LEFT] Failed to save: %s", left_file.c_str());
                }
                // Save sensor info before clearing
                auto origin = left_cloud_->sensor_origin_;
                auto orient = left_cloud_->sensor_orientation_;
                left_cloud_->clear();
                // Restore sensor info after clearing
                left_cloud_->sensor_origin_ = origin;
                left_cloud_->sensor_orientation_ = orient;
            } else {
                RCLCPP_WARN(this->get_logger(), "[LEFT] No points to save");
            }
        }

        // Save Right point cloud
        {
            std::lock_guard<std::mutex> lock(right_mutex_);
            right_points = right_cloud_->size();
            if (!right_cloud_->empty()) {
                right_file = save_path_right_ + "/right_pcl_" + timestamp + ".pcd";
                // Use ASCII format to match bundle program output (human readable)
                if (pcl::io::savePCDFileASCII(right_file, *right_cloud_) == 0) {
                    right_saved = true;
                    RCLCPP_INFO(this->get_logger(), "[RIGHT] Saved: %s (%zu points, format: %s, yaw corrected: %s)",
                        right_file.c_str(), right_cloud_->size(),
                        use_mm_scale_ ? "mm" : "m",
                        correct_right_yaw_ ? "yes" : "no");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[RIGHT] Failed to save: %s", right_file.c_str());
                }
                // Save sensor info before clearing
                auto origin = right_cloud_->sensor_origin_;
                auto orient = right_cloud_->sensor_orientation_;
                right_cloud_->clear();
                // Restore sensor info after clearing
                right_cloud_->sensor_origin_ = origin;
                right_cloud_->sensor_orientation_ = orient;
            } else {
                RCLCPP_WARN(this->get_logger(), "[RIGHT] No points to save");
            }
        }

        // Build response message
        std::ostringstream msg;
        if (left_saved && right_saved) {
            response->success = true;
            msg << "Both saved successfully. "
                << "Left: " << left_points << " points, "
                << "Right: " << right_points << " points. "
                << "Timestamp: " << timestamp;
        } else if (left_saved || right_saved) {
            response->success = true;
            msg << "Partial save. ";
            if (left_saved) msg << "Left: " << left_points << " points. ";
            else msg << "Left: empty. ";
            if (right_saved) msg << "Right: " << right_points << " points.";
            else msg << "Right: empty.";
        } else {
            response->success = false;
            msg << "No data to save. Make sure LiDARs are publishing data.";
        }
        response->message = msg.str();

        RCLCPP_INFO(this->get_logger(), "Save result: %s", response->message.c_str());
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;

    pcl::PointCloud<PointXYZI_mm>::Ptr left_cloud_;
    pcl::PointCloud<PointXYZI_mm>::Ptr right_cloud_;

    std::mutex left_mutex_;
    std::mutex right_mutex_;

    std::atomic<bool> is_collecting_;
    rclcpp::Time collection_start_time_;

    std::string left_topic_;
    std::string right_topic_;
    std::string save_path_left_;
    std::string save_path_right_;
    double accumulate_duration_;

    // Coordinate correction parameters
    bool use_mm_scale_;
    bool correct_right_yaw_;
    double right_yaw_correction_;
    double cos_yaw_;
    double sin_yaw_;

    // Point filtering parameters
    bool filter_invalid_points_;
    double max_range_;
    double min_range_;

    // Bounding box parameters (in mm)
    bool use_bounding_box_;
    double left_x_max_;
    double left_y_min_;
    double left_y_max_;
    double right_x_max_;
    double right_y_min_;
    double right_y_max_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualLidarNode>();
    RCLCPP_INFO(node->get_logger(), "Dual LiDAR Node is running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
