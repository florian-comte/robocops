import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from robocops_msgs.msg import DuploArray, Duplo
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
import math


class DuploControl(Node):
    def __init__(self):
        super().__init__('duplo_control')

        # Create service clients
        self.activate_client = self.create_client(SetBool, 'activate_detection')
        self.clear_client = self.create_client(Empty, 'clear_duplos')

        # Create a subscriber to read from the duplos topic
        self.duplo_subscriber = self.create_subscription(
            DuploArray,
            '/duplos', 
            self.duplo_callback,
            10
        )

        # Create a publisher to control robot movement (cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_smoothed', 10)

        # Initialize variables
        self.duplos_list = []
        self.rotation_active = False
        self.target_position = None
        
        # Wait for services to be available
        while not self.activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('activate_detection service not available, waiting again...')
        
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear_duplos service not available, waiting again...')
        
        self.get_logger().info('Services are now available.')

    def activate_detection(self):
        request = SetBool.Request()
        request.data = True
        self.get_logger().info('Activating detection...')
        self.activate_client.call_async(request)

    def deactivate_detection(self):
        request = SetBool.Request()
        request.data = False
        self.get_logger().info('Deactivating detection...')
        self.activate_client.call_async(request)

    def clear_duplos(self):
        self.get_logger().info('Clearing duplos...')
        self.clear_client.call_async(Empty.Request())

    def duplo_callback(self, msg: DuploArray):
        if msg.duplos:
            
            self.target_position = msg.duplos[0].position
            self.get_logger().info(f"Target Position received: {self.target_position}")
            if self.rotation_active:
                self.align_robot_with_target()

    def read_duplos(self):
        if not self.duplos_list:
            self.get_logger().info("No duplos received yet.")
        else:
            self.get_logger().info(f"Current duplos: {len(self.duplos_list)}")
            for duplo in self.duplos_list:
                self.get_logger().info(f"Duplo ID: {duplo.id}, Position: {duplo.position}, Score: {duplo.score}")

    def print_menu(self):
        print("\nDuplo Control Menu:")
        print("1. Start detection")
        print("2. Stop detection")
        print("3. Clear duplos")
        print("4. Read duplos")
        print("5. Toggle Rotation")
        print("6. Exit")
        choice = input("Enter your choice (1-6): ")

        return choice

    def toggle_rotation(self):
        """ Toggle rotation on/off """
        self.rotation_active = not self.rotation_active
        if self.rotation_active:
            self.get_logger().info("Rotation started.")
        else:
            self.get_logger().info("Rotation stopped.")
            self.stop_robot()

    def stop_robot(self):
        """ Stop the robot's motion (in case of manual stop) """
        stop_msg = TwistStamped()
        self.cmd_vel_publisher.publish(stop_msg)

    def align_robot_with_target(self):
        """ Rotate robot to align with the target position """
        if self.target_position is None:
            self.get_logger().info("No target position to align with.")
            return

        # Calculate the angle to rotate
        robot_x, robot_y = 0.0, 0.0  # Assuming base_link origin is (0, 0)
        target_x, target_y = self.target_position.x, self.target_position.y

        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)

        self.get_logger().info(f"Aligning robot to angle: {angle_to_target} radians")

        # Rotate robot until it is aligned
        self.rotate_robot(angle_to_target)

    def rotate_robot(self, angle):
        """ Rotate the robot to a specific angle """
        # Assume the robot rotates at a fixed speed
        rotation_speed = 0.5
        
        twist = TwistStamped()
        twist.angular.z = rotation_speed if angle > 0 else -rotation_speed
        
        # Publish the twist message to rotate
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    duplo_control_node = DuploControl()

    # Use a MultiThreadedExecutor for handling both menu input and subscription callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(duplo_control_node)

    while rclpy.ok():
        # Display menu
        user_choice = duplo_control_node.print_menu()

        if user_choice == '1':
            duplo_control_node.activate_detection()
        elif user_choice == '2':
            duplo_control_node.deactivate_detection()
        elif user_choice == '3':
            duplo_control_node.clear_duplos()
        elif user_choice == '4':
            duplo_control_node.read_duplos()
        elif user_choice == '5':
            duplo_control_node.toggle_rotation()
        elif user_choice == '6':
            duplo_control_node.get_logger().info("Exiting...")
            break
        else:
            print("Invalid option. Please choose between 1-6.")

        # Spin the executor and keep the node alive
        executor.spin_once()

    duplo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
