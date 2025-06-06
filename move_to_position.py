import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ROSInterruptException

class NavigateToPosition(Node):
    def __init__(self):
        super().__init__('navigate_to_position')
        # Create an ActionClient for NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y):
        # Create a PoseStamped message to represent the goal
        goal_msg = NavigateToPose.Goal()
        
        # Set up the pose to move to (simple pose with x, y and orientation)
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward (no rotation)

        # Send the goal and wait for the result
        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal to position: ({x}, {y})")
        self._action_client.send_goal_async(goal_msg)


def main():
    rclpy.init()

    # Instantiate the node
    nav_node = NavigateToPosition()

    try:
        # Ask user for the target position
        x = float(input("Enter the target X position: "))
        y = float(input("Enter the target Y position: "))

        # Send the goal to move the robot to the target position
        nav_node.send_goal(x, y)

        # Spin to allow action to complete
        rclpy.spin_once(nav_node)

    except ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
