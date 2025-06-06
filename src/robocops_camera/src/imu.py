import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import time
import cv2


class DepthAIROS2Node(Node):
    def __init__(self):
        super().__init__('robocops_imu')

        # Create publishers for ROS2
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        # Create CvBridge to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Create Camera node
        cam = self.pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080P)
        cam.setFps(30)

        # Create IMU node
        imu = self.pipeline.create(dai.node.IMU)
        imu.setFps(200)  # IMU frequency (200 Hz)

        # Link nodes
        xlinkOut_video = self.pipeline.create(dai.node.XLinkOut)
        xlinkOut_video.setStreamName("video")
        cam.video.link(xlinkOut_video.input)

        xlinkOut_imu = self.pipeline.create(dai.node.XLinkOut)
        xlinkOut_imu.setStreamName("imu")
        imu.out.link(xlinkOut_imu.input)

        # Connect to DepthAI device and start pipeline
        self.device = dai.Device(self.pipeline)

        # Output queues
        self.video_queue = self.device.getOutputQueue(name="video", maxSize=8, blocking=False)
        self.imu_queue = self.device.getOutputQueue(name="imu", maxSize=8, blocking=False)

        self.get_logger().info('DepthAI ROS2 Node Started!')

    def publish_data(self):
        while rclpy.ok():
            # Get a frame from the camera
            frame = self.video_queue.get()
            frame_cv = frame.getCvFrame()

            # Publish camera image
            ros_image = self.bridge.cv2_to_imgmsg(frame_cv, encoding="bgr8")
            self.image_pub.publish(ros_image)

            # Get IMU data
            imu_data = self.imu_queue.get()

            # Create IMU ROS message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "camera"

            # Accelerometer data (x, y, z)
            imu_msg.linear_acceleration.x = imu_data.acceleroMeter.x
            imu_msg.linear_acceleration.y = imu_data.acceleroMeter.y
            imu_msg.linear_acceleration.z = imu_data.acceleroMeter.z

            # Gyroscope data (x, y, z)
            imu_msg.angular_velocity.x = imu_data.gyroscope.x
            imu_msg.angular_velocity.y = imu_data.gyroscope.y
            imu_msg.angular_velocity.z = imu_data.gyroscope.z

            # Publish IMU data
            self.imu_pub.publish(imu_msg)

            time.sleep(0.05)  # Sleep for 50ms (20Hz)

    def run(self):
        try:
            self.publish_data()
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
        finally:
            self.device.close()


def main(args=None):
    rclpy.init(args=args)
    node = DepthAIROS2Node()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
