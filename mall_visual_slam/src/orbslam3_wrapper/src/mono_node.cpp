#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "System.h"
#include "CameraModels/GeometricCamera.h"

#include <string>
#include <mutex>

class MonoNode : public rclcpp::Node
{
public:
    MonoNode() : Node("mono_node")
    {
        // === 参数声明 ===
        this->declare_parameter("vocab_path", "/home/ros/ros2_orbslam3/Opensource code/ORB_SLAM3-master/Vocabulary/ORBvoc.txt");
        this->declare_parameter("settings_path", "/home/ros/ros2_orbslam3/config/KannalaBrandt8_960x540.yaml");
        this->declare_parameter("enable_viewer", false);
        this->declare_parameter("enable_edge_enhancement", true);
        this->declare_parameter("enable_dynamic_mask", false);
        this->declare_parameter("mask_fill_value", 128);
        this->declare_parameter("publish_map_every_n_frames", 10);

        std::string vocab_path = this->get_parameter("vocab_path").as_string();
        std::string settings_path = this->get_parameter("settings_path").as_string();
        bool enable_viewer = this->get_parameter("enable_viewer").as_bool();
        enable_edge_enhancement_ = this->get_parameter("enable_edge_enhancement").as_bool();
        enable_dynamic_mask_ = this->get_parameter("enable_dynamic_mask").as_bool();
        mask_fill_value_ = this->get_parameter("mask_fill_value").as_int();
        publish_map_every_n_ = this->get_parameter("publish_map_every_n_frames").as_int();

        // === 初始化 ORB-SLAM3 ===
        SLAM_ = std::make_shared<ORB_SLAM3::System>(
            vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, enable_viewer);

        // === 订阅 ===
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&MonoNode::ImageCallback, this, std::placeholders::_1));

        sub_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/yolo/dynamic_mask", 10,
            std::bind(&MonoNode::MaskCallback, this, std::placeholders::_1));

        // === 发布者 ===
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/slam/odom", 10);
        pub_state_ = this->create_publisher<std_msgs::msg::Int32>("/slam/tracking_state", 10);
        pub_map_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_points", 10);
        pub_debug_image_ = this->create_publisher<sensor_msgs::msg::Image>("/slam/debug_image", 10);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Mono Node Started");
        RCLCPP_INFO(this->get_logger(), "  Config: %s", settings_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  Vocab:  %s", vocab_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  Viewer:  %s", enable_viewer ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "  Dynamic mask: %s", enable_dynamic_mask_ ? "enabled" : "disabled");
    }

    ~MonoNode()
    {
        if (SLAM_) {
            SLAM_->Shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 shut down");
    }

private:
    // ========================
    // YOLO 动态物体掩码回调
    // ========================
    void MaskCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat mask = cv_bridge::toCvShare(msg, "mono8")->image;
            if (!mask.empty()) {
                std::lock_guard<std::mutex> lock(mask_mutex_);
                // 深拷贝，防止 cv_bridge 回收数据
                latest_mask_ = mask.clone();
                mask_timestamp_ = msg->header.stamp;
            }
        } catch (std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Mask conversion error: %s", e.what());
        }
    }

    // ========================
    // 图像回调（SLAM 主流程）
    // ========================
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // --- 1. ROS Image → OpenCV ---
            cv::Mat im_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

            // --- 2. 转灰度 ---
            cv::Mat im_gray;
            cv::cvtColor(im_bgr, im_gray, cv::COLOR_BGR2GRAY);

            // --- 3. 应用动态物体/地面掩码 ---
            if (enable_dynamic_mask_) {
                im_gray = applyDynamicMask(im_gray, msg->header.stamp);
            }

            // --- 4. CLAHE 局部对比度增强 (待启用) ---
            // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            // clahe->apply(im_gray, im_gray);

            // --- 5. 边缘增强（Unsharp Masking）---
            if (enable_edge_enhancement_) {
                cv::Mat blurred;
                cv::GaussianBlur(im_gray, blurred, cv::Size(0, 0), 3.0);
                cv::addWeighted(im_gray, 1.5, blurred, -0.5, 0, im_gray);
            }

            // --- 6. 保存调试帧（每100帧一次）---
            if (frame_count_ % 100 == 0) {
                cv::imwrite("/home/ros/ros2_orbslam3/output/debug_slam_input.jpg", im_gray);
            }

            // --- 7. SLAM 追踪 ---
            double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            Sophus::SE3f pose = SLAM_->TrackMonocular(im_gray, timestamp);

            // --- 8. 发布位姿 ---
            publishPose(pose, msg->header.stamp);

            // --- 9. 发布追踪状态 ---
            int tracking_state = SLAM_->GetTrackingState();
            auto state_msg = std_msgs::msg::Int32();
            state_msg.data = tracking_state;
            pub_state_->publish(state_msg);

            // --- 10. 周期性发布地图点云 ---
            if (tracking_state == 2 && frame_count_ % publish_map_every_n_ == 0) {
                publishMapPoints(msg->header.stamp);
            }

            // --- 11. 发布调试图像（带特征点）---
            publishDebugImage(im_bgr, msg->header.stamp);

            frame_count_++;
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ImageCallback error: %s", e.what());
        }
    }

    // ========================
    // 应用动态物体掩码
    // ========================
    cv::Mat applyDynamicMask(const cv::Mat &im_gray, const rclcpp::Time &image_time)
    {
        cv::Mat result = im_gray.clone();
        cv::Mat mask_to_apply;

        {
            std::lock_guard<std::mutex> lock(mask_mutex_);
            if (!latest_mask_.empty()) {
                // 检查时间戳差值（mask 不能太旧，0.2 秒内有效）
                double time_diff = std::abs((image_time - mask_timestamp_).seconds());
                if (time_diff < 0.2 && latest_mask_.size() == im_gray.size()) {
                    mask_to_apply = latest_mask_.clone();
                }
            }
        }

        if (!mask_to_apply.empty()) {
            // 在动态区域填充中性灰（128），使 ORB 不会在此提取特征
            result.setTo(cv::Scalar(mask_fill_value_), mask_to_apply > 0);
        }

        return result;
    }

    // ========================
    // 发布相机位姿
    // ========================
    void publishPose(const Sophus::SE3f &pose, const rclcpp::Time &stamp)
    {
        // 空位姿表示追踪失败，不发布
        if (pose.matrix().isIdentity(1e-6f)) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "world";

        // 平移
        Eigen::Vector3f t = pose.translation();
        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();

        // 旋转（从旋转矩阵提取四元数）
        Eigen::Matrix3f R = pose.rotationMatrix();
        Eigen::Quaternionf q(R);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pub_pose_->publish(pose_msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "camera";
        odom_msg.pose.pose = pose_msg.pose;
        pub_odom_->publish(odom_msg);
    }

    // ========================
    // 发布地图点云
    // ========================
    void publishMapPoints(const rclcpp::Time &stamp)
    {
        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM_->GetTrackedMapPoints();
        if (map_points.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = "world";
        cloud.width = map_points.size();
        cloud.height = 1;
        cloud.is_dense = false;

        // 定义字段
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (auto *mp : map_points) {
            if (mp && !mp->isBad()) {
                Eigen::Vector3f pos = mp->GetWorldPos();
                *iter_x = pos.x(); ++iter_x;
                *iter_y = pos.y(); ++iter_y;
                *iter_z = pos.z(); ++iter_z;
            } else {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                ++iter_x; ++iter_y; ++iter_z;
            }
        }

        pub_map_points_->publish(cloud);
    }

    // ========================
    // 发布调试图像
    // ========================
    void publishDebugImage(const cv::Mat &im_bgr, const rclcpp::Time &stamp)
    {
        // 每 5 帧发一次调试图像
        if (frame_count_ % 5 != 0) return;

        std::vector<cv::KeyPoint> keypoints = SLAM_->GetTrackedKeyPointsUn();
        if (keypoints.empty()) return;

        cv::Mat im_debug = im_bgr.clone();
        cv::drawKeypoints(im_debug, keypoints, im_debug,
                          cv::Scalar(0, 255, 0),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_debug).toImageMsg();
        debug_msg->header.stamp = stamp;
        debug_msg->header.frame_id = "camera";
        pub_debug_image_->publish(*debug_msg);
    }

    // ========================
    // 成员变量
    // ========================
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mask_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_state_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_points_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

    std::shared_ptr<ORB_SLAM3::System> SLAM_;

    // 掩码和同步
    cv::Mat latest_mask_;
    rclcpp::Time mask_timestamp_{0, 0};
    std::mutex mask_mutex_;

    // 计数和配置
    int frame_count_ = 0;
    int publish_map_every_n_ = 10;
    bool enable_edge_enhancement_ = true;
    bool enable_dynamic_mask_ = false;
    int mask_fill_value_ = 128;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoNode>());
    rclcpp::shutdown();
    return 0;
}
