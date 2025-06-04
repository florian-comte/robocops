// If timeout
// define has timeout
// exit failure

// If took all duplos of the zone
// exit success

// If stills duplo in the zone
// Get closest duplo in the zone
// wait if lift still active
// Go to closest duplo (check if lift is active during this time -> pause)
// Grab it

// If no duplos in the zone, just navigate randomly around (turn on itself ...)

#include "search_and_grab.h"

#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <iostream>
#include <algorithm>

using namespace std::chrono;

SearchAndGrab::SearchAndGrab(const std::string &name, const BT::NodeConfiguration &config)
    : BT::AsyncActionNode(name, config),
      rclcpp::Node("search_and_grab_node"),
      zone_(0),
      is_halted_(false)
{
    int zone_param = 0;
    getInput("zone", zone_param);

    detections_sub_ = this->create_subscription<robocops_msgs::msg::DuploArray>(
        "/duplos/zone" + std::to_string(zone_param), 10,
        std::bind(&SearchAndGrab::detectionsCallback, this, std::placeholders::_1));

    gpio_sub_ = this->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
        "/gpio_controller/gpio_states", 10,
        std::bind(&SearchAndGrab::gpioCallback, this, std::placeholders::_1));

    start_time_ = std::chrono::steady_clock::now();

    std::cout << "[SearchAndGrab] Subscribed to topic: " << topic << std::endl;
}

BT::NodeStatus SearchAndGrab::tick()
{
    is_halted_ = false;

    if (!getInput<int>("zone", zone_))
    {
        std::cerr << "[SearchAndGrab] Missing required input [zone]" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int timeout_sec = 60;
    getInput<int>("timeout", timeout_sec);

    auto now = steady_clock::now();
    auto elapsed = duration_cast<seconds>(now - start_time_).count();

    if (elapsed > timeout_sec)
    {
        std::cerr << "[SearchAndGrab] Timeout reached (" << elapsed << "s)." << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (is_halted_)
    {
        std::cout << "[SearchAndGrab] Halted!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Process duplos in the zone
    if (duplos_.empty())
    {
        std::cout << "[SearchAndGrab] No duplos found. Exploring..." << std::endl;
        exploreZone();
        return BT::NodeStatus::RUNNING;
    }

    auto closest = getClosestDuplo();

    if (isLiftActive())
    {
        std::cout << "[SearchAndGrab] Waiting for lift to be ready..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    // Activate the brushes

    // Go to duplo
    if (goToDuplo(closest))
    {
        if (grabDuplo(closest))
        {
            std::cout << "[SearchAndGrab] Successfully grabbed duplo." << std::endl;
            removeDuplo(closest);
        }
        else
        {
            std::cerr << "[SearchAndGrab] Failed to grab duplo." << std::endl;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void SearchAndGrab::halt()
{
    is_halted_ = true;
}

BT::PortsList SearchAndGrab::providedPorts()
{
    return {
        BT::InputPort<int>("zone", "Zone number to search and grab in"),
        BT::InputPort<int>("timeout", "Timeout time (in seconds)")};
}

void SearchAndGrab::detectionsCallback(const robocops_msgs::msg::DuploArray::SharedPtr msg)
{
    duplos_.clear();

    for (const auto &duplo : msg->duplos)
    {
        duplos_.push_back(duplo);
    }

    std::cout << "[SearchAndGrab] Updated duplos list: " << duplos_.size() << " duplos detected." << std::endl;
}

void SearchAndGrab::gpioCallback(const control_msgs::msg::DynamicInterfaceGroupValues msg)
{
}
robocops_msgs::msg::Duplo SearchAndGrab::getClosestDuplo()
{
    // Naively return first duplo (you can add distance sorting here)
    return duplos_.front();
}

bool SearchAndGrab::isLiftActive()
{
    // Simulate lift check
    return false;
}

bool SearchAndGrab::goToDuplo(const robocops_msgs::msg::Duplo &duplo)
{
    std::cout << "[SearchAndGrab] Navigating to duplo at (" << duplo.x << ", " << duplo.y << ")" << std::endl;
    return true;
}

bool SearchAndGrab::grabDuplo(const robocops_msgs::msg::Duplo &duplo)
{
    std::cout << "[SearchAndGrab] Grabbing duplo at (" << duplo.x << ", " << duplo.y << ")" << std::endl;
    return true;
}

void SearchAndGrab::removeDuplo(const robocops_msgs::msg::Duplo &duplo)
{
    // should send to service of robocpos_duplos
}

void SearchAndGrab::exploreZone()
{
    // Simulate turning or random navigation
    std::cout << "[SearchAndGrab] Exploring zone " << zone_ << " (turning...)" << std::endl;
}
