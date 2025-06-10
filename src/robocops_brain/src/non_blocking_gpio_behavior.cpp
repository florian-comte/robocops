#include "non_blocking_gpio_behavior.h"
#include "rclcpp/rclcpp.hpp"

NonBlockingGPIO::NonBlockingGPIO(const std::string &name,
                                 const BT::NodeConfiguration &config,
                                 rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    gpio_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/commands", 10);
}

BT::PortsList NonBlockingGPIO::providedPorts()
{
    return {
        BT::InputPort<std::string>("gpio_name"),
        BT::InputPort<std::string>("interface_name"),
        BT::InputPort<int>("active"),
    };
}

BT::NodeStatus NonBlockingGPIO::onStart()
{
    getInput("gpio_name", gpio_name_);
    getInput("interface_name", interface_name_);
    getInput("active", active_);

    RCLCPP_INFO(node_->get_logger(), "Starting NonBlockingGPIO for [%s] interface [%s]",
                gpio_name_.c_str(), interface_name_.c_str());

    send_gpio_command(active_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NonBlockingGPIO::onRunning()
{
    return BT::NodeStatus::SUCCESS;
}

void NonBlockingGPIO::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "NonBlockingGPIO halted.");
}

void NonBlockingGPIO::send_gpio_command(bool active)
{
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    msg.interface_groups.push_back(gpio_name_);

    control_msgs::msg::InterfaceValue iface;
    iface.interface_names.push_back(interface_name_);
    iface.values.push_back(active ? 1.0 : 0.0);

    msg.interface_values.push_back(iface);

    RCLCPP_INFO(node_->get_logger(), "Sending GPIO command: %s", active ? "ON" : "OFF");

    gpio_pub_->publish(msg);
}
