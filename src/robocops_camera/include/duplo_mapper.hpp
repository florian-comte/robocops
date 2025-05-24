#ifndef DUPLO_MAPPER_HPP
#define DUPLO_MAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <Eigen/Dense>

class DuploMapper : public rclcpp::Node
{
public:
    DuploMapper();

private:
    void detectionsCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections);

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detectionsSub;
};

#endif // DUPLO_MAPPER_HPP
