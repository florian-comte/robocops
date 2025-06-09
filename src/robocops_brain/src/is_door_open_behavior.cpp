#include "get_lidar_distance_at_angle_node.h"
#include <cmath>
#include <iostream>

// Constructor
GetLidarDistanceAtAngle::GetLidarDistanceAtAngle(const std::string &name,
                                                 const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    lidar_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan/filtered", 10,
        std::bind(&GetLidarDistanceAtAngle::lidar_callback, this, std::placeholders::_1));
}

// Declare input and output ports
BT::PortsList GetLidarDistanceAtAngle::providedPorts()
{
    return {
        BT::InputPort<float>("timeout_duration"),
        BT::InputPort<float>("angle"),
        BT::OutputPort<float>("lidar_angle"),
        BT::OutputPort<float>("lidar_distance")};
}

BT::NodeStatus GetLidarDistanceAtAngle::onStart()
{
    if (!getInput("angle", angle_))
    {
        throw BT::RuntimeError("Missing required input [angle]");
    }

    if (angle_ > 2* M_PI || angle_ < 0)
    {
        throw BT::RuntimeError("Angle should be between 0 and pi");
    }

    if (!getInput("timeout_duration", timeout_duration_))
    {
        throw BT::RuntimeError("Missing required input [timeout_duration]");
    }

    start_time_ = node_->get_clock()->now();

    // At first no distance found
    distance_ = -1;
}

BT::NodeStatus GetLidarDistanceAtAngle::onRunning()
{
    if ((node_->get_clock()->now() - start_time_) > timeout_duration_)
    {
        RCLCPP_INFO(node_->get_logger(), "Reading lidar duration exceeded. Return FAILURE.");
        return BT::NodeStatus::FAILURE;
    }

    if (distance_ >= 0)
    {
        setOutput("lidar_angle", angle);
        setOutput("lidar_distance", distance_);

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void GetLidarDistanceAtAngle::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    size_t num_readings = msg->ranges.size();

    // Validate the requested angle
    if (angle_ < angle_min || angle_ > angle_max)
    {
        RCLCPP_WARN(node_->get_logger(), "Requested angle %.2f is out of bounds [%.2f, %.2f]",
                    angle_, angle_min, angle_max);
        return;
    }

    // Compute the index of the scan that corresponds to the desired angle
    size_t index = static_cast<size_t>((angle_ - angle_min) / angle_increment);

    if (index >= num_readings)
    {
        RCLCPP_WARN(node_->get_logger(), "Computed index %zu out of range (max %zu)",
                    index, num_readings - 1);
        return;
    }

    float range = msg->ranges[index];

    if (std::isnan(range) || std::isinf(range) || range <= 0.0)
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid LIDAR reading at angle %.2f (index %zu)", angle_, index);
        return;
    }

    distance_ = range;
}
