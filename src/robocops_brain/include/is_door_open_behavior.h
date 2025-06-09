#ifndef GET_LIDAR_READING_NODE_H
#define GET_LIDAR_READING_NODE_H

#include <behaviortree_cpp/bt_factory.h>

class GetLidarDistanceAtAngle : public BT::StatefulActionNode
{
public:
    GetLidarDistanceAtAngle(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;

    double angle_ = 0;
    double distance_ = -1.0;

    double timeout_duration_;
    rclcpp::Time start_time_;
};

#endif // GET_LIDAR_READING_NODE_H
