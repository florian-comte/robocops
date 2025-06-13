#include "set_pose_behavior.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SetPose::SetPose(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
}

BT::PortsList SetPose::providedPorts()
{
    return {
        BT::InputPort<std::vector<double>>("pose") // [x, y, theta]
    };
}

BT::NodeStatus SetPose::tick()
{
    std::vector<double> pose_vec;
    if (!getInput<std::vector<double>>("pose", pose_vec))
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [pose]");
        return BT::NodeStatus::FAILURE;
    }

    if (pose_vec.size() != 3)
    {
        RCLCPP_ERROR(node_->get_logger(), "Pose vector must have 3 elements: [x, y, theta]");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = pose_vec[0];
    pose_msg.pose.pose.position.y = pose_vec[1];
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_vec[2]);
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    // Basic covariance setup for AMCL to not reject the update
    for (int i = 0; i < 36; ++i)
    {
        pose_msg.pose.covariance[i] = 0.0;
    }
    pose_msg.pose.covariance[0] = 0.25;    // x
    pose_msg.pose.covariance[7] = 0.25;    // y
    pose_msg.pose.covariance[35] = 0.0685; // yaw

    pose_pub_->publish(pose_msg);

    return BT::NodeStatus::SUCCESS;
}