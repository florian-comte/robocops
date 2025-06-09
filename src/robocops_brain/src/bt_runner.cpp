/*
 * Main behavior node for TurtleBot.
 */

#include <chrono>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <robocops_msgs/msg/duplo.hpp>
#include <robocops_msgs/msg/duplo_array.hpp>

#include "navigation_behaviors.h"
#include "blocking_gpio_behavior.h"
#include "search_and_grab_behavior.h"
#include "set_pose_behavior.h"
#include "can_go_to_zone_behavior.h"
#include "is_door_open_behavior.h"

using namespace std::chrono_literals;

class BtRunner : public rclcpp::Node
{
public:
    BtRunner() : Node("bt_runner")
    {
        this->declare_parameter<std::string>("tree_xml_file", "");
        tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();

        this->declare_parameter<std::string>("locations_file", "");
        locations_file_ = this->get_parameter("locations_file").as_string();
    }

    void execute()
    {
        // Build and initialize the behavior tree based on parameters.
        create_behavior_tree();

        // Create a timer to tick the behavior tree.
        const auto timer_period = 500ms;
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&BtRunner::update_behavior_tree, this));

        rclcpp::spin(shared_from_this());
        rclcpp::shutdown();
    }

    void create_behavior_tree()
    {
        RCLCPP_INFO(this->get_logger(), "Loading Behavior Tree from: %s", tree_xml_file_.c_str());

        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<SetLocations>("SetLocations");
        factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
        factory.registerNodeType<GoToPose>("GoToPose", shared_from_this());
        factory.registerNodeType<BlockingGPIO>("BlockingGPIO", shared_from_this());
        factory.registerNodeType<SetPose>("SetPose", shared_from_this());
        factory.registerNodeType<SearchAndGrab>("SearchAndGrab", shared_from_this());
        factory.registerNodeType<CanGoToZone>("CanGoToZone");
        factory.registerNodeType<IsDoorOpen>("IsDoorOpen", shared_from_this());

        auto blackboard = BT::Blackboard::create();

        blackboard->set<std::string>("locations_file", locations_file_);

        blackboard->set<int>("MAX_INVENTORY", 15);
        blackboard->set<std::vector<int>>("MIN_TO_GO_ZONES", std::vector<int>{ 0, 0, 1, 1 });
        blackboard->set<std::vector<int>>("TOTAL_ZONES", std::vector<int>{15, 6, 6, 6});

        blackboard->set<std::vector<int>>("current_grabbed_zones", std::vector<int>{0, 0, 0, 0});
        blackboard->set<std::vector<bool>>("never_timed_out_zones", std::vector<bool>{ true, true, true, true});

        blackboard->set<int>("current_inventory", 0);
        blackboard->set<bool>("never_timed_out_opening_door", true);

        blackboard->set<bool>("is_door_open", false);

        blackboard->set<double>("lidar_angle", 0.0);
        blackboard->set<double>("lidar_distance", -1.0);

        RCLCPP_INFO(this->get_logger(), "Attempting to create tree from XML...");

        try
        {
            tree_ = factory.createTreeFromFile(tree_xml_file_, blackboard);
            RCLCPP_INFO(this->get_logger(), "BT successfully created");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create tree: %s", e.what());
            rclcpp::shutdown();
        }

        publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree_, 1668);
    }

    void update_behavior_tree()
    {
        // Tick the behavior tree.
        BT::NodeStatus tree_status = tree_.tickOnce();
        if (tree_status == BT::NodeStatus::RUNNING)
        {
            return;
        }

        // Cancel the timer if we hit a terminal state.
        if (tree_status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
            timer_->cancel();
        }
        else if (tree_status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
            timer_->cancel();
        }
    }

    // Configuration parameters.
    std::string tree_xml_file_;
    std::string locations_file_;

    // ROS and BehaviorTree.CPP variables.
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BtRunner>();
    std::unique_ptr<BT::Groot2Publisher> publisher_ptr_;
    node->execute();
    return 0;
}
