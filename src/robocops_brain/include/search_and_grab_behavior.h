#ifndef SEARCH_AND_GRAB_H_
#define SEARCH_AND_GRAB_H_

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/interface_value.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <chrono>
#include <string>

class SearchAndGrab : public BT::StatefulActionNode
{
public:
    SearchAndGrab(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_pub_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_sub_;

    double timeout_sec_;
    rclcpp::Time start_time_;
    int zone_;
};

#endif // SEARCH_AND_GRAB_H_
