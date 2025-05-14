#include "duplos_mapper.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

DuplosMapper::DuplosMapper() : Node("duplos_mapper")
{
    m_detectionsSub = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
        "/camera/detections", 20, std::bind(&DuplosMapper::detectionsCallback, this, std::placeholders::_1));

    // Placeholder for future subscriptions/publications
    // m_pose_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //     "/robot/position", 20, std::bind(&DuplosMapper::poseCallback, this, std::placeholders::_1));

    // duplo_pub_ = this->create_publisher<robocops_msgs::msg::DuploArray>("/raw_duplos", 20);
}

void DuplosMapper::detectionsCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections)
{
    // Camera-to-world transform:
    // - Translation: camera is 1.0m above the ground (along Z in world)
    // - Rotation: camera pitched down 22 degrees

    const Eigen::Vector3d T(0.0, 1.0, 0.0); // [x, y, z] in world frame (e.g., 1m above ground)

    double pitch_rad = -22.0 * M_PI / 180.0; // pitch down = negative
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitX());

    for (const auto &det : detections->detections)
    {
        double x = det.position.x;
        double y = det.position.y;
        double z = det.position.z;

        Eigen::Vector3d P_cam(x, y, z);
        Eigen::Vector3d P_world = R * P_cam + T;

        RCLCPP_INFO(rclcpp::get_logger("duplos_mapper"),
                    "World Position: [x=%.2f, y=%.2f, z=%.2f]",
                    P_world.x(), P_world.y(), P_world.z());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DuplosMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
