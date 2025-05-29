#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <robocops_brain/action_goto.h>
#include <robocops_brain/action_wait.h>
#include <robocops_brain/condition_is_at.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = ros2_bt_utils::ROSNode();

  std::string param_bt_filename;
  node->declare_parameter("bt_path", "")

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ActionGoto>("GoTo");
  factory.registerNodeType<ActionWait>("Wait");

  factory.registerNodeType<ConditionIsAt>("IsAtPose");

  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromFile(param_bt_filename.c_str(), blackboard);

  geometry_msgs::msg::Pose zone3_entrance;
  zone3_entrance.position.x = 4.0;
  zone3_entrance.position.y = 3.0;

  zone3_entrance.orientation.z = 0.707;
  zone3_entrance.orientation.w = 0.707;

  blackboard->set("zone3_entrance", zone3_entrance);

  geometry_msgs::msg::Pose zone4_entrance;
  zone4_entrance.position.x = 4.0;
  zone4_entrance.position.y = 3.0;

  zone4_entrance.orientation.z = 0.707;
  zone4_entrance.orientation.w = 0.707;

  blackboard->set("zone4_entrance", zone4_entrance);

  BT::FileLogger logger_file(tree, "bt_trace.fbl");

  while (rclcpp::ok()) {
    tree.rootNode()->executeTick();
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return 0;
}