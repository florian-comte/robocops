#include "search_and_grab_behavior.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

SearchAndGrab::SearchAndGrab(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    gpio_pub_ = node_->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/commands", 10);

    gpio_sub_ = node_->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/gpio_states", 10,
        std::bind(&SearchAndGrab::gpio_state_callback, this, std::placeholders::_1));

    detections_sub_ = node_->create_subscription<robocops_msgs::msg::DuploArray>(
        "/duplos", 10,
        std::bind(&SearchAndGrab::detections_state_callback, this, std::placeholders::_1));

    activate_client_ = node_->create_client<std_srvs::srv::SetBool>("activate_detection");
    clear_client_ = node_->create_client<std_srvs::srv::Empty>("clear_duplos");

    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_->get_node_base_interface(),
        node_->get_node_graph_interface(),
        node_->get_node_logging_interface(),
        node_->get_node_waitables_interface(),
        "navigate_to_pose");

    spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(
        node_->get_node_base_interface(),
        node_->get_node_graph_interface(),
        node_->get_node_logging_interface(),
        node_->get_node_waitables_interface(),
        "spin");

    initial_duplos_counter_ = -1;
}

BT::PortsList SearchAndGrab::providedPorts()
{
    return {
        BT::InputPort<int>("zone"),
        BT::InputPort<int>("timeout_duration"),
        BT::InputPort<std::vector<int>>("current_grabbed_zones"),
        BT::OutputPort<std::vector<int>>("current_grabbed_zones"),
        BT::InputPort<std::vector<int>>("never_timed_out_zones"),
        BT::OutputPort<std::vector<int>>("never_timed_out_zones"),
        BT::InputPort<int>("current_inventory"),
        BT::OutputPort<int>("current_inventory"),
    };
}

BT::NodeStatus SearchAndGrab::onStart()
{
    if (!getInput("zone", zone_))
    {
        throw BT::RuntimeError("Missing required input [zone]");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("timeout_duration", timeout_duration_))
    {
        throw BT::RuntimeError("Missing required input [timeout_duration]");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("current_grabbed_zones", current_grabbed_zones_))
    {
        throw BT::RuntimeError("Missing required input [current_grabbed_zones]");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("current_inventory", current_inventory_))
    {
        throw BT::RuntimeError("Missing required input [current_inventory]");
        return BT::NodeStatus::FAILURE;
    }

    start_time_ = node_->get_clock()->now();
    search_state_ = SEARCHING;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchAndGrab::onRunning()
{
    auto now = node_->get_clock()->now();

    if ((now - start_time_).seconds() > timeout_duration_)
    {
        std::vector<int> never_timed_out_zones;

        if (!getInput("never_timed_out_zones", never_timed_out_zones))
        {
            throw BT::RuntimeError("Missing required input [never_timed_out_zones]");
            return BT::NodeStatus::FAILURE;
        }

        never_timed_out_zones[zone_] = 1;

        setOutput("never_timed_out_zones", never_timed_out_zones);

        // here timeout should be a success
        return BT::NodeStatus::SUCCESS;
    }

    if (is_moving)
    {
        return BT::NodeStatus::RUNNING;
    }

    switch (search_state_)
    {
    case SEARCHING:
        return handleSearching();
    case APPROACHING:
        return handleApproaching();
    case GRABBING:
        return handleGrabbing();
    case RETURNING:
        return handleReturning();
    default:
        return BT::NodeStatus::FAILURE;
    }
}

void SearchAndGrab::onHalted()
{
    deactivate_detection();
    disable_capture();
    clear_duplos();
}

void SearchAndGrab::gpio_state_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
{
    for (size_t i = 0; i < msg->interface_groups.size(); ++i)
    {
        if (msg->interface_groups[i] == "captured_duplos")
        {
            for (size_t j = 0; j < msg->interface_values[i].interface_names.size(); ++j)
            {
                if (msg->interface_values[i].interface_names[j] == "number")
                {
                    double val = msg->interface_values[i].values[j];

                    if (initial_duplos_counter_ == -1)
                    {
                        initial_duplos_counter_ = (int)val;
                    }

                    if (current_grabbed_zones_.size() <= zone_)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Almsot seg fault.");
                        return;
                    }

                    current_grabbed_zones_[zone_] = (int)val - initial_duplos_counter_;

                    setOutput("current_grabbed_zones", current_grabbed_zones_);
                    setOutput("current_inventory", ++current_inventory_);

                    // RCLCPP_DEBUG(node_->get_logger(), "GPIO state received: [%s] -> [%s] = %.2f",
                    //              "gpio_name_.c_str()", "interface_name_.c_str()", val);
                    return;
                }
            }
        }
    }
}

void SearchAndGrab::detections_state_callback(const robocops_msgs::msg::DuploArray::SharedPtr msg)
{
    if (!msg->duplos.empty())
    {
        duplos_list_ = msg->duplos;
        update_closest_duplo();
    }
}

void SearchAndGrab::activate_detection()
{
    clear_duplos();
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    activate_client_->async_send_request(request);
}

void SearchAndGrab::deactivate_detection()
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    activate_client_->async_send_request(request);
}

void SearchAndGrab::update_closest_duplo()
{
    if (duplos_list_.empty())
    {
        closest_duplo_ = nullptr;
        return;
    }

    closest_duplo_ = std::make_shared<robocops_msgs::msg::Duplo>(*std::min_element(
        duplos_list_.begin(), duplos_list_.end(),
        [this](const robocops_msgs::msg::Duplo &a, const robocops_msgs::msg::Duplo &b)
        {
            return distance_to_point(a.position.point) < distance_to_point(b.position.point);
        }));
}

double SearchAndGrab::distance_to_point(const geometry_msgs::msg::Point &point)
{
    return std::sqrt(point.x * point.x + point.y * point.y);
}

void SearchAndGrab::enable_capture()
{
    auto msg = std::make_shared<control_msgs::msg::DynamicInterfaceGroupValues>();
    msg->interface_groups = {"capture"};

    control_msgs::msg::InterfaceValue capture_msg;
    capture_msg.interface_names = {"active"};
    capture_msg.values = {1.0};

    msg->interface_values.push_back(capture_msg);
    gpio_pub_->publish(*msg);
}

void SearchAndGrab::disable_capture()
{
    auto msg = std::make_shared<control_msgs::msg::DynamicInterfaceGroupValues>();
    msg->interface_groups = {"capture"};

    control_msgs::msg::InterfaceValue capture_msg;
    capture_msg.interface_names = {"active"};
    capture_msg.values = {0.0};

    msg->interface_values.push_back(capture_msg);
    gpio_pub_->publish(*msg);
}

void SearchAndGrab::clear_duplos()
{
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    clear_client_->async_send_request(request);
    duplos_list_.clear();
    closest_duplo_ = nullptr;
}

void SearchAndGrab::go_to_pose(float x, float y, float yaw)
{
    using namespace nav2_msgs::action;

    if (!nav_to_pose_client_->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "base_link";
    goal_msg.pose.header.stamp = node_->get_clock()->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = yaw;

    is_moving = true;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "Navigation goal succeeded.");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Navigation goal failed or was canceled.");
        }
        is_moving = false;
    };

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void SearchAndGrab::spin(float angle)
{
    using namespace nav2_msgs::action;

    if (!spin_client_->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(node_->get_logger(), "Spin action server not available.");
        return;
    }

    auto goal_msg = Spin::Goal();
    goal_msg.target_yaw = angle;
    goal_msg.time_allowance.sec = 10;

    is_moving = true;

    auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "Spin action succeeded.");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Spin action failed or was canceled.");
        }
        is_moving = false;
    };

    spin_client_->async_send_goal(goal_msg, send_goal_options);
}

BT::NodeStatus SearchAndGrab::handleSearching()
{
    if (!search_started_)
    {
        RCLCPP_INFO(node_->get_logger(), "Starting search pattern");
        activate_detection();
        search_start_time_ = node_->get_clock()->now();
        search_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    auto now = node_->get_clock()->now();
    if ((now - search_start_time_).seconds() < SEARCHING_TIME_PER_STOP)
    {
        return BT::NodeStatus::RUNNING;
    }

    if (closest_duplo_ != nullptr)
    {
        RCLCPP_INFO(node_->get_logger(), "Duplo found, switching to approach");
        deactivate_detection();
        search_state_ = APPROACHING;
        return BT::NodeStatus::RUNNING;
    }

    // No duplo found, rotate and continue searching
    spin(ANGLE_STEP);
    search_start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchAndGrab::handleApproaching()
{
    if (!approach_started_)
    {
        RCLCPP_INFO(node_->get_logger(), "Approaching duplo");
        enable_capture();

        auto pos = closest_duplo_->position.point;
        go_to_pose(pos.x + 0.05, pos.y, 1.57);

        approach_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node_->get_logger(), "Approach complete, switching to grab");
    search_state_ = GRABBING;
    grab_start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchAndGrab::handleGrabbing()
{
    auto now = node_->get_clock()->now();
    if ((now - grab_start_time_).seconds() < GRABBING_TIME)
    {
        return BT::NodeStatus::RUNNING;
    }

    disable_capture();
    clear_duplos();

    // Reset for next search
    search_started_ = false;
    approach_started_ = false;
    closest_duplo_ = nullptr;
    search_state_ = SEARCHING;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchAndGrab::handleReturning()
{
    // Implement return logic if needed
    return BT::NodeStatus::SUCCESS;
}