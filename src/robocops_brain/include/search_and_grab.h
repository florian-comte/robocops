#ifndef SEARCH_AND_GRAB_H_
#define SEARCH_AND_GRAB_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <robocops_msgs/msg/duplo_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

class SearchAndGrab : public BT::AsyncActionNode
{
public:
    SearchAndGrab(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    void halt() override;

private:
    void detectionsCallback(const robocops_msgs::msg::DuploArray::SharedPtr msg);
    void gpioCallback(const control_msgs::msg::DynamicInterfaceGroupValues msg);

    robocops_msgs::msg::Duplo getClosestDuplo();
    bool goToDuplo(const robocops_msgs::msg::Duplo &duplo);
    bool grabDuplo(const robocops_msgs::msg::Duplo &duplo);
    bool isLiftActive();
    void removeDuplo(const robocops_msgs::msg::Duplo &duplo);
    void exploreZone();

    int zone_;
    bool is_halted_;
    std::vector<robocops_msgs::msg::Duplo> duplos_;
    rclcpp::Subscription<robocops_msgs::msg::DuploArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_sub_;

    std::chrono::steady_clock::time_point start_time_;
};

#endif // SEARCH_AND_GRAB_H_
