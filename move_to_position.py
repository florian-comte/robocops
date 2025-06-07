import rclpy
from robocops_msgs.srv import SetPosition
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MoveToPositionClient(Node):
    def __init__(self):
        super().__init__('move_to_position_client')

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.client = self.create_client(SetPosition, 'set_position')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
    def call_service(self, x, y, z):
        """Call the service to update the robot's target position."""
        request = SetPosition.Request()
        request.x = x
        request.y = y
        request.z = z

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"Service call success: {x}, {y}, {z}")
            return True
        else:
            self.get_logger().error("Service call failed.")
            return False

    def send_goal(self, x, y, z):
        """Send the robot's goal position based on the current position and new position."""
        # Get the robot's current position (e.g., by subscribing to the /amcl_pose topic)
        current_position = self.get_current_position()

        # Add the current position to the target position
        goal_x = current_position.x + x
        goal_y = current_position.y + y
        goal_z = current_position.z + z

        # Create the navigation goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = goal_z
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        # Send the goal to the navigation action server
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

    def get_current_position(self):
        """Retrieve the current robot position (dummy example, use real data in actual code)."""
        # You can retrieve the robot position from a topic such as /amcl_pose
        return Point(0.0, 0.0, 0.0)  # Replace with actual data source

def main():
    rclpy.init()

    client = MoveToPositionClient()

    try:
        # Example coordinates received from the C++ service (this would be dynamically received)
        target_x, target_y, target_z = 1.0, 2.0, 0.0

        # Call the service to set the target position
        if client.call_service(target_x, target_y, target_z):
            client.send_goal(target_x, target_y, target_z)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
