#ifndef NON_BLOCKING_GPIO_H_
#define NON_BLOCKING_GPIO_H_

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/interface_value.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <chrono>
#include <string>

class NonBlockingGPIO : public BT::StatefulActionNode
{
public:
    NonBlockingGPIO(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void send_gpio_command(bool active);
   
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_pub_;

    std::string gpio_name_;
    std::string interface_name_;
    int active_;
};

#endif // NON_BLOCKING_GPIO_H_
