import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from robocops_msgs.msg import DuploArray, Duplo
from rclpy.executors import MultiThreadedExecutor


class DuploControl(Node):
    def __init__(self):
        super().__init__('duplo_control')

        # Create service clients
        self.activate_client = self.create_client(SetBool, 'activate_detection')
        self.clear_client = self.create_client(Empty, 'clear_duplos')

        # Create a subscriber to read from the duplos topic
        self.duplo_subscriber = self.create_subscription(
            DuploArray,
            'duplos/zone1',  # Adjust topic as per your setup
            self.duplo_callback,
            10
        )
        
        self.duplos_list = []  # This will hold the duplos read from the topic

        # Wait for services to be available
        while not self.activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('activate_detection service not available, waiting again...')
        
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('clear_duplos service not available, waiting again...')
        
        self.get_logger().info('Services are now available.')

    def activate_detection(self):
        request = SetBool.Request()
        request.data = True  # Set to True to activate detection
        self.get_logger().info('Activating detection...')
        self.activate_client.call_async(request)

    def deactivate_detection(self):
        request = SetBool.Request()
        request.data = False  # Set to False to deactivate detection
        self.get_logger().info('Deactivating detection...')
        self.activate_client.call_async(request)

    def clear_duplos(self):
        self.get_logger().info('Clearing duplos...')
        self.clear_client.call_async(Empty.Request())

    def duplo_callback(self, msg: DuploArray):
        """ Callback to process received duplos from the topic """
        self.get_logger().info(f"Received {len(self.duplos_list)} duplos.")

    def read_duplos(self):
        """ Display the duplos read from the topic """
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
        print("5. Exit")
        choice = input("Enter your choice (1-5): ")

        return choice


def main(args=None):
    rclpy.init(args=args)

    duplo_control_node = DuploControl()

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
            duplo_control_node.get_logger().info("Exiting...")
            break
        else:
            print("Invalid option. Please choose between 1-5.")

        # Spin and keep the node alive
        rclpy.spin_once(duplo_control_node)

    duplo_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
