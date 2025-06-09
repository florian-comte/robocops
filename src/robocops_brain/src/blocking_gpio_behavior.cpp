#include "blocking_gpio_behavior.h"
#include "rclcpp/rclcpp.hpp"

BlockingGPIO::BlockingGPIO(const std::string &name,
                           const BT::NodeConfiguration &config,
                           rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    gpio_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/commands", 10);

    gpio_sub_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/gpio_states", 10,
        std::bind(&BlockingGPIO::gpio_state_callback, this, std::placeholders::_1));
}

BT::PortsList BlockingGPIO::providedPorts()
{
    return {
        BT::InputPort<std::string>("gpio_name"),
        BT::InputPort<std::string>("interface_name"),
        BT::InputPort<double>("timeout"),
        BT::InputPort<double>("pulse_duration")};
}

BT::NodeStatus BlockingGPIO::onStart()
{
    getInput("gpio_name", gpio_name_);
    getInput("interface_name", interface_name_);
    getInput("timeout", timeout_sec_);
    getInput("pulse_duration", pulse_duration_);
    getInput("wait_after_duration", wait_after_duration_);

    RCLCPP_INFO(node_->get_logger(), "Starting BlockingGPIO for [%s] interface [%s]",
                gpio_name_.c_str(), interface_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Pulse duration: %.2f s, Timeout: %.2f s",
                pulse_duration_, timeout_sec_);

    if (pulse_duration_ > timeout_sec_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Pulse duration exceeds timeout. Aborting.");
        return BT::NodeStatus::FAILURE;
    }

    already_pulsed_ = false;
    is_waiting_ = false;

    send_gpio_command(true);
    start_time_ = node_->get_clock()->now();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BlockingGPIO::onRunning()
{
    auto now = node_->get_clock()->now();

    if ((now - start_time_).seconds() > timeout_sec_)
    {
        RCLCPP_WARN(node_->get_logger(), "Timeout exceeded waiting for GPIO to deactivate.");
        return BT::NodeStatus::FAILURE;
    }

    if ((now - start_time_).seconds() > pulse_duration_ && !already_pulsed_)
    {
        RCLCPP_INFO(node_->get_logger(), "Pulse duration exceeded. Sending OFF command.");
        send_gpio_command(false);
        already_pulsed_ = true;
    }

    if (!gpio_active_)
    {
        if (is_waiting_ && ((now - wait_time_).seconds() > wait_after_duration_))
        {
            RCLCPP_INFO(node_->get_logger(), "GPIO [%s] Returned success.", gpio_name_.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "GPIO [%s] deactivated. Returning SUCCESS.", gpio_name_.c_str());
            is_waiting_ = true;
            wait_time_ = now;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void BlockingGPIO::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "BlockingGPIO halted.");
}

void BlockingGPIO::send_gpio_command(bool active)
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

void BlockingGPIO::gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
{
    for (size_t i = 0; i < msg->interface_groups.size(); ++i)
    {
        if (msg->interface_groups[i] == gpio_name_)
        {
            for (size_t j = 0; j < msg->interface_values[i].interface_names.size(); ++j)
            {
                if (msg->interface_values[i].interface_names[j] == interface_name_)
                {
                    double val = msg->interface_values[i].values[j];
                    gpio_active_ = (val > 0.5);

                    RCLCPP_DEBUG(node_->get_logger(), "GPIO state received: [%s] -> [%s] = %.2f",
                                 gpio_name_.c_str(), interface_name_.c_str(), val);
                    return;
                }
            }
        }
    }
}
