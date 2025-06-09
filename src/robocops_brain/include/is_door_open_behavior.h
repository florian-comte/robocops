#ifndef GET_LIDAR_READING_NODE_H
#define GET_LIDAR_READING_NODE_H

#include <behaviortree_cpp/bt_factory.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/rclcpp.hpp"

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/interface_value.hpp>

#define MIN_DISTANCE_DOOR 150

class IsDoorOpen : public BT::StatefulActionNode
{
public:
    IsDoorOpen(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    void gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg);
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_sub_;

    std::shared_ptr<rclcpp::Node> node_;

    double start_angle_ = 0;
    double end_angle_ = 0;
    double distance_ = -1.0;

    double timeout_duration_;
    rclcpp::Time start_time_;
};

#endif // GET_LIDAR_READING_NODE_H
