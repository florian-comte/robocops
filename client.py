import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from robocops_msgs.msg import DuploArray, Duplo
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64

import math
import threading
import time


class DuploControl(Node):
    def __init__(self):
        super().__init__('duplo_control')

        # Service clients
        self.activate_client = self.create_client(SetBool, 'activate_detection')
        self.clear_client = self.create_client(Empty, 'clear_duplos')

        # Duplo detection subscriber
        self.duplo_subscriber = self.create_subscription(
            DuploArray,
            '/duplos',
            self.duplo_callback,
            10
        )

        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # GPIO command publisher for capture/active
        self.capture_pub = self.create_publisher(Float64, '/gpio_command/capture/active', 10)

        # Internal state
        self.duplos_list = []

        # Wait for required services and action servers
        while not self.activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for activate_detection service...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear_duplos service...')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_to_pose action server...')

        self.get_logger().info('All services and action servers available.')

        # Launch CLI menu
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

    def enable_capture(self, enable: bool):
        msg = Float64()
        msg.data = 1.0 if enable else 0.0
        self.capture_pub.publish(msg)
        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} capture...")

    def duplo_callback(self, msg: DuploArray):
        if msg.duplos:
            self.duplos_list = msg.duplos

    def read_duplos(self):
        if not self.duplos_list:
            self.get_logger().info("No Duplos received yet.")
        else:
            self.get_logger().info(f"Current Duplos: {len(self.duplos_list)}")
            for duplo in self.duplos_list:
                pos = duplo.position.point
                self.get_logger().info(
                    f"Duplo ID: {duplo.id}, Position: x={pos.x:.2f}, y={pos.y:.2f}, Score: {duplo.score:.2f}"
                )

    def run_menu(self):
        while rclpy.ok():
            print("\n--- Duplo Control Menu ---")
            print("1. Start detection")
            print("2. Stop detection")
            print("3. Clear duplos")
            print("4. Read duplos")
            print("5. Search and grab closest Duplo")
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
                self.search_and_grab()
            elif choice == '6':
                self.get_logger().info("Exiting...")
                break
            else:
                print("Invalid option. Please choose between 1 and 6.")

    def get_closest_duplo(self):
        if not self.duplos_list:
            return None
        return min(self.duplos_list, key=lambda d: self.distance_to_point(d.position.point))

    def distance_to_point(self, point: Point):
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def send_navigation_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'camera'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = yaw

        goal_msg.pose = goal_pose
        self.get_logger().info(f"Sending navigation goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected.")
            return

        self.get_logger().info("Navigation goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().warn(f"Goal failed with status code: {result.status}")

    def search_and_grab(self):
        self.get_logger().info("Starting search and grab sequence.")

        self.enable_capture(True)
        time.sleep(0.5)

        self.activate_detection()
        time.sleep(1.0)  # Allow time for detection
        self.deactivate_detection()

        self.enable_capture(False)

        closest_duplo = self.get_closest_duplo()
        if not closest_duplo:
            self.get_logger().info("No Duplos found.")
            return

        pos = closest_duplo.position.point
        self.get_logger().info(
            f"Closest Duplo found: ID {closest_duplo.id} at x={pos.x:.2f}, y={pos.y:.2f}"
        )

        self.send_navigation_goal(pos.x, pos.y)

        # Simulated grab
        self.get_logger().info(f"Simulated grab: Duplo ID {closest_duplo.id}")
        self.clear_duplos()


def main(args=None):
    rclpy.init(args=args)
    node = DuploControl()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
