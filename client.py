import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from robocops_msgs.msg import DuploArray, Duplo
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
import math
import threading


class DuploControl(Node):
    def __init__(self):
        super().__init__('duplo_control')

        self.activate_client = self.create_client(SetBool, 'activate_detection')
        self.clear_client = self.create_client(Empty, 'clear_duplos')

        self.duplo_subscriber = self.create_subscription(
            DuploArray,
            '/duplos', 
            self.duplo_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel_smoothed', 10)

        self.duplos_list = []
        self.rotation_active = False
        self.target_position = None
        self.angle_to_target = None

        # Timer for rotating at 10Hz
        self.rotation_timer = None
        
        # Wait for services to be available
        while not self.activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('activate_detection service not available, waiting again...')
        
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear_duplos service not available, waiting again...')
        
        self.get_logger().info('Services are now available.')
        
        # For handling user input in a non-blocking way
        self.menu_thread = threading.Thread(target=self.run_menu, daemon=True)
        self.menu_thread.start()

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
            self.duplos_list = msg.duplos
            self.get_logger().info(f"Target Position received: {self.target_position}")
            if self.rotation_active:
                self.calculate_angle_to_target()

    def read_duplos(self):
        if not self.duplos_list:
            self.get_logger().info("No duplos received yet.")
        else:
            self.get_logger().info(f"Current duplos: {len(self.duplos_list)}")
            for duplo in self.duplos_list:
                self.get_logger().info(f"Duplo ID: {duplo.id}, Position: {duplo.position}, Score: {duplo.score}")

    def run_menu(self):
        """ This method handles user input in a separate thread """
        while rclpy.ok():
            print("\nDuplo Control Menu:")
            print("1. Start detection")
            print("2. Stop detection")
            print("3. Clear duplos")
            print("4. Read duplos")
            print("5. Toggle Rotation")
            print("6. Exit")
            choice = input("Enter your choice (1-6): ")

            if choice == '1':
                self.activate_detection()
            elif choice == '2':
                self.deactivate_detection()
            elif choice == '3':
                self.clear_duplos()
            elif choice == '4':
                self.read_duplos()
            elif choice == '5':
                self.toggle_rotation()
            elif choice == '6':
                self.get_logger().info("Exiting...")
                self.stop_robot()  # Stop robot before shutdown
                break
            else:
                print("Invalid option. Please choose between 1-6.")

    def toggle_rotation(self):
        """ Toggle rotation on/off """
        self.rotation_active = not self.rotation_active
        if self.rotation_active:
            self.get_logger().info("Rotation started.")
            # Recalculate the angle to the target and set up the timer
            self.calculate_angle_to_target()
            self.rotation_timer = self.create_timer(0.1, self.rotate_robot)  # Timer for 10 Hz (0.1s)
        else:
            self.get_logger().info("Rotation stopped.")
            self.stop_robot()
            if self.rotation_timer:
                self.rotation_timer.cancel()
                self.rotation_timer = None

    def stop_robot(self):
        """ Stop the robot's motion (in case of manual stop) """
        stop_msg = TwistStamped()
        self.cmd_vel_publisher.publish(stop_msg)

    def calculate_angle_to_target(self):
        """ Calculate the angle to the target and set it as the goal angle """
        if self.target_position is None:
            self.get_logger().info("No target position to align with.")
            return

        # Calculate the angle to rotate
        robot_x, robot_y = 0.0, 0.0  # Assuming base_link origin is (0, 0)
        target_x, target_y = self.target_position.x, self.target_position.y

        self.angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        self.get_logger().info(f"Calculated angle to target: {self.angle_to_target} radians")

    def rotate_robot(self):
        """ Rotate the robot to a specific angle at 10 Hz """
        if self.angle_to_target is None:
            self.get_logger().info("No target angle to rotate towards.")
            return
        
        # Introduce tolerance for stopping
        tolerance = 0.1

        if abs(self.angle_to_target) < tolerance:
            self.get_logger().info("Target alignment achieved.")
            self.stop_robot()
            if self.rotation_timer:
                self.rotation_timer.cancel()
                self.rotation_timer = None
            return

        # Rotate at a fixed speed
        rotation_speed = 0.5
        twist = TwistStamped()
        twist.angular.z = rotation_speed if self.angle_to_target > 0 else -rotation_speed
        
        # Publish the twist message to rotate
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    duplo_control_node = DuploControl()

    # Use a MultiThreadedExecutor for handling both menu input and subscription callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(duplo_control_node)

    while rclpy.ok():
        # Spin the executor and keep the node alive
        executor.spin_once()

    duplo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
