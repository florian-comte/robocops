#include "is_door_open_behavior.h"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

// Constructor
IsDoorOpen::IsDoorOpen(const std::string &name,
                       const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    gpio_sub_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/gpio_states", 10,
        std::bind(&IsDoorOpen::gpio_state_callback, this, std::placeholders::_1));
}

// Declare input and output ports
BT::PortsList IsDoorOpen::providedPorts()
{
    return {
        BT::InputPort<int>("is_door_open"),
        BT::InputPort<float>("timeout_duration"),
        BT::OutputPort<int>("is_door_open"),
    };
}

BT::NodeStatus IsDoorOpen::onStart()
{
    if (!getInput("timeout_duration", timeout_duration_))
    {
        throw BT::RuntimeError("Missing required input [timeout_duration]");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("is_door_open", is_door_open_))
    {
        throw BT::RuntimeError("Missing required input [is_door_open]");
        return BT::NodeStatus::FAILURE;
    }

    if (is_door_open_)
    {
        return BT::NodeStatus::SUCCESS;
    }

    start_time_ = node_->get_clock()->now();

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
        setOutput("is_door_open", 1);
        return distance_ > MIN_DISTANCE_DOOR ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void IsDoorOpen::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "IsDoorOpen halted.");
}

void IsDoorOpen::gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
{
    for (size_t i = 0; i < msg->interface_groups.size(); ++i)
    {
        if (msg->interface_groups[i] == "back_ultrasound")
        {
            for (size_t j = 0; j < msg->interface_values[i].interface_names.size(); ++j)
            {
                if (msg->interface_values[i].interface_names[j] == "distance")
                {
                    double val = msg->interface_values[i].values[j];

                    RCLCPP_DEBUG(node_->get_logger(), "GPIO state received: [%s] -> [%s] = %.2f",
                                 "back_ultrasound", "distance", val);
                    distance_ = val;
                    return;
                }
            }
        }
    }
}
