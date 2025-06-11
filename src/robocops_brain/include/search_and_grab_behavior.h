#ifndef SEARCH_AND_GRAB_BEHAVIOR_H
#define SEARCH_AND_GRAB_BEHAVIOR_H

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robocops_msgs/msg/duplo_array.hpp"
#include "robocops_msgs/msg/duplo.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include <chrono>
#include <vector>
#include <memory>

class SearchAndGrab : public BT::StatefulActionNode
{
public:
    SearchAndGrab(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    // BT functions
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // Search states
    enum SearchState {
        SEARCHING,
        APPROACHING,
        GRABBING,
        RETURNING
    };

    // Constants
    const float SEARCHING_TIME_PER_STOP = 5.0;
    const float ANGLE_STEP = 0.5;
    const float GRABBING_TIME = 6.0;

    // ROS 2 components
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_pub_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_sub_;
    rclcpp::Subscription<robocops_msgs::msg::DuploArray>::SharedPtr detections_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;

    // State variables
    SearchState search_state_;
    bool search_started_ = false;
    bool approach_started_ = false;
    rclcpp::Time start_time_;
    rclcpp::Time search_start_time_;
    rclcpp::Time grab_start_time_;
    int zone_;
    int timeout_duration_;
    std::vector<int> current_grabbed_zones_;
    bool is_moving = false;

    int initial_duplos_counter_;
    int current_inventory_;

    // Duplo tracking
    std::vector<robocops_msgs::msg::Duplo> duplos_list_;
    std::shared_ptr<robocops_msgs::msg::Duplo> closest_duplo_;

    // Callbacks
    void gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg);
    void detections_state_callback(const robocops_msgs::msg::DuploArray::SharedPtr msg);

    // Helper methods
    void activate_detection();
    void deactivate_detection();
    void update_closest_duplo();
    double distance_to_point(const geometry_msgs::msg::Point& point);
    void enable_capture();
    void disable_capture();
    void clear_duplos();
    void go_to_pose(float x, float y, float yaw = 0.0);
    void spin(float angle);

    // State handlers
    BT::NodeStatus handleSearching();
    BT::NodeStatus handleApproaching();
    BT::NodeStatus handleGrabbing();
    BT::NodeStatus handleReturning();
};

#endif // SEARCH_AND_GRAB_BEHAVIOR_H