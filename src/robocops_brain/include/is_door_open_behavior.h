#ifndef GET_LIDAR_READING_NODE_H
#define GET_LIDAR_READING_NODE_H

#include <behaviortree_cpp/bt_factory.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/rclcpp.hpp"

#define ANGLE_DOOR 3.10
#define START_ANGLE_DOOR 170
#define END_ANGLE_DOOR 180

class IsDoorOpen : public BT::StatefulActionNode
{
public:
    IsDoorOpen(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;

    double start_angle_ = 0;
    double end_angle_ = 0;
    double distance_ = -1.0;

    double timeout_duration_;
    rclcpp::Time start_time_;
};

#endif // GET_LIDAR_READING_NODE_H
