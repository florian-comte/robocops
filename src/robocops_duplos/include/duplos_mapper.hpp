#ifndef DUPLOS_MAPPER_HPP
#define DUPLOS_MAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <Eigen/Dense>

class DuplosMapper : public rclcpp::Node
{
public:
    DuplosMapper();

private:
    void detectionsCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections);

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detectionsSub;
};

#endif // DUPLOS_MAPPER_HPP
