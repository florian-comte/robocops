#ifndef SET_POSE_BEHAVIOR_H
#define SET_POSE_BEHAVIOR_H

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class SetPose : public BT::SyncActionNode
{
public:
    SetPose(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

#endif // SET_POSE_BEHAVIOR_H
