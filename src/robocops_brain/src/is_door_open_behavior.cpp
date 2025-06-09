#include "is_door_open_behavior.h"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

// Constructor
IsDoorOpen::IsDoorOpen(const std::string &name,
                       const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    lidar_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan/filtered", 10,
        std::bind(&IsDoorOpen::lidar_callback, this, std::placeholders::_1));
}

// Declare input and output ports
BT::PortsList IsDoorOpen::providedPorts()
{
    return
    {
        BT::InputPort<float>("timeout_duration"),
    };
}

BT::NodeStatus IsDoorOpen::onStart()
{
    if (!getInput("timeout_duration", timeout_duration_))
    {
        throw BT::RuntimeError("Missing required input [timeout_duration]");
        return BT::NodeStatus::FAILURE;
    }

    start_time_ = node_->get_clock()->now();

    angle_ = M_PI;

    // At first no distance found
    distance_ = -1;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IsDoorOpen::onRunning()
{
    if ((node_->get_clock()->now() - start_time_).seconds() > timeout_duration_)
    {
        RCLCPP_INFO(node_->get_logger(), "Reading lidar duration exceeded. Return FAILURE.");
        return BT::NodeStatus::FAILURE;
    }

    if (distance_ >= 0)
    {
        RCLCPP_INFO(node_->get_logger(), "Current distance on door: %.2f", distance_);
        return distance_ > 1.5 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void IsDoorOpen::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "IsDoorOpen halted.");
}

void IsDoorOpen::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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
