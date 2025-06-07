#include "search_and_grab_behavior.h"

SearchAndGrab::SearchAndGrab(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    gpio_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/commands", 10);

    gpio_sub_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/state", 10,
        std::bind(&SearchAndGrab::gpio_state_callback, this, std::placeholders::_1));

    detections_sub_ = node_->create_subscription<robocops_msgs::msg::DuploArray>(
        "/", 10,
        std::bind(&SearchAndGrab::detections_state_callback, this, std::placeholders::_1));
}

BT::PortsList SearchAndGrab::providedPorts()
{
    return
    {
        BT::InputPort<int>("zone"),
    };
}

BT::NodeStatus SearchAndGrab::onStart()
{
    getInput("zone", zone_);

    start_time_ = node_->get_clock()->now();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchAndGrab::onRunning()
{

    auto now = node_->get_clock()->now();

    if ((now - start_time_).seconds() > timeout_sec_)
    {
        return BT::NodeStatus::FAILURE;
    }


    return BT::NodeStatus::RUNNING;
}

void SearchAndGrab::onHalted()
{
}

void SearchAndGrab::gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
{
    for (size_t i = 0; i < msg->interface_groups.size(); ++i)
    {
        if (msg->interface_groups[i] == "TODO")
        {
            for (size_t j = 0; j < msg->interface_values[i].interface_names.size(); ++j)
            {
                if (msg->interface_values[i].interface_names[j] == "TODO")
                {
                    double val = msg->interface_values[i].values[j];
                    return;
                }
            }
        }
    }
}

void SearchAndGrab::detections_state_callback(const robocops_msgs::msg::DuploArray msg)
{
    for (size_t i = 0; i < msg.duplos.size(); ++i)
    {
    }
}
