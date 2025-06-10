import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from robocops_msgs.msg import DuploArray, Duplo
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
import threading
import time


class DuploControl(Node):
    def __init__(self):
        super().__init__('duplo_control')

        # Service clients
        self.activate_client = self.create_client(SetBool, 'activate_detection')
        self.clear_client = self.create_client(Empty, 'clear_duplos')
        
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        # self.navigator.changeMap('/home/robocops/robocops/src/robocops_navigation/maps/final_arena_blank.yaml')

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
        self.gpio_pub = self.create_publisher(DynamicInterfaceGroupValues, '/gpio_controller/commands', 10)

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
        self.duplos_list = []

    def enable_capture(self, enable: bool):
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = ["capture"]

        capture_msg = InterfaceValue()
        capture_msg.interface_names = ["active"]
        capture_msg.values = [1.0 if enable else 0.0]

        msg.interface_values.append(capture_msg)
        self.gpio_pub.publish(msg)

        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} capture via DynamicInterfaceGroupValues.")

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
            print("6. Stop capturing")
            print("7. Exit")
            choice = input("Enter your choice (1-7): ")

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
                self.stop_capture()
            elif choice == '7':
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
        self.navigator.clearLocalCostmap()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = yaw
        
        go_to_pose_task = self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()


        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = self.navigator.getTaskError()
            print('Goal failed!{error_code}:{error_msg}')
        else:
            print('Goal has an invalid return status!')

    def search_and_grab(self):
        self.get_logger().info("Starting search and grab sequence.")
        
        SEARCHING_TIME_PER_STOP = 3
        ANGLE_STEP = 0.70
        
        time.sleep(3)
        
        closest_duplo = None
        start_searching_time = self.get_clock().now()
        
        # Take photo and find d
        while not closest_duplo or (self.get_clock().now() - start_searching_time) > 15:
            self.activate_detection()
            
            time.sleep(SEARCHING_TIME_PER_STOP)

            closest_duplo = self.get_closest_duplo()
            start_searching_time = self.get_clock().now()
            
            if not closest_duplo:
                self.send_navigation_goal(0.0, 0.0, ANGLE_STEP)
                
            self.deactivate_detection()
        
        if not closest_duplo:
            self.get_logger().info("No Duplos found.")
            return

        pos = closest_duplo.position.point
        self.get_logger().info(
            f"Closest Duplo found: ID {closest_duplo.id} at x={pos.x:.2f}, y={pos.y:.2f}"
        )
        
        self.enable_capture(True)
        
        self.send_navigation_goal(pos.x - 0.10, pos.y, 1.57)
        
        time.sleep(5)
        
        self.enable_capture(False)
        
        self.clear_duplos()
        
    def stop_capture(self):
        self.enable_capture(False)

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
