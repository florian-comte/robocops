/*
 * duplo_detection_publisher.cpp
 *
 * This program sets up a DepthAI pipeline for detecting objects using a YOLO-based spatial detection network.
 * It initializes the DepthAI device, configures RGB and mono cameras, and processes detections for ROS 2 publication.
 *
 * The node is designed to interface with the DepthAI device to perform real-time object detection and spatial
 * detection using a YOLO model. The detected objects are then published as spatial detection messages in ROS 2,
 * with optional publishing of the raw RGB image and depth data.
 *
 * The primary outputs of this node are:
 *  - `/camera/detections`: Spatial detection data including detected objects' bounding boxes, confidence scores, and 3D coordinates.
 *  - `/camera/raw_rgb`: Raw RGB camera feed (if display is enabled).
 *  - `/camera/depth`: Depth data from stereo cameras (if display is enabled).
 *
 */
#include <cstdio>
#include <iostream>
#include <unordered_map>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <robocops_msgs/msg/duplo.hpp>
#include <robocops_msgs/msg/duplo_array.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

float calculateNnFps()
{
    static auto last_time = std::chrono::steady_clock::now();
    static int count = 0;
    static float fps = 0.0f;

    count++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (elapsed >= 1000)
    {
        fps = count * 1000.0f / elapsed;
        count = 0;
        last_time = now;
    }
    return fps;
}

/**
 * @brief Creates a DepthAI pipeline for object detection.
 * @param rgb_resolution_str Desired resolution of the RGB camera.
 * @param nn_name Path to the neural network blob file.
 * @param with_display True if you want to have out links for depth and raw rgb
 * @return Configured DepthAI pipeline.
 */
dai::Pipeline create_pipeline(const std::string nn_name, bool with_display)
{
    dai::Pipeline pipeline;

    // Define camera and processing nodes
    auto rgb_camera = pipeline.create<dai::node::ColorCamera>();
    auto spatial_detection_network = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xout_nn = pipeline.create<dai::node::XLinkOut>();
    xout_nn->setStreamName("detections");
    spatial_detection_network->setNumInferenceThreads(2);
    spatial_detection_network->setNumNCEPerInferenceThread(1);

    // Configure RGB camera
    rgb_camera->setPreviewSize(416, 416);
    rgb_camera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    rgb_camera->setInterleaved(false);
    rgb_camera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    rgb_camera->setFps(25);

    // Configure mono cameras (480p resolution)
    mono_left->setResolution(dai::node::MonoCamera::Properties::SensorResolution::THE_480_P);
    mono_left->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    mono_right->setResolution(dai::node::MonoCamera::Properties::SensorResolution::THE_480_P);
    mono_right->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    // Configure stereo depth processing
    stereo->initialConfig.setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0);
    stereo->initialConfig.setLeftRightCheckThreshold(true);
    stereo->setSubpixel(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    // Configure YOLO spatial detection network
    spatial_detection_network->setBlobPath(nn_name);
    spatial_detection_network->setConfidenceThreshold(0.5f);
    spatial_detection_network->setBoundingBoxScaleFactor(0.5);
    spatial_detection_network->setDepthLowerThreshold(100);
    spatial_detection_network->setDepthUpperThreshold(5000);

    // Set YOLO network parameters
    spatial_detection_network->setNumClasses(1);
    spatial_detection_network->setCoordinateSize(4);
    spatial_detection_network->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatial_detection_network->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatial_detection_network->setIouThreshold(0.5f);

    // Link stereo and RGB camera outputs
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);

    rgb_camera->preview.link(spatial_detection_network->input);

    stereo->depth.link(spatial_detection_network->inputDepth);
    spatial_detection_network->out.link(xout_nn->input);

    // If we want display, we also create out link for rgb and depth
    if (with_display)
    {
        auto xout_rgb = pipeline.create<dai::node::XLinkOut>();
        xout_rgb->setStreamName("preview");
        spatial_detection_network->passthrough.link(xout_rgb->input);
    }

    return pipeline;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("duplo_detection_publisher");

    std::string resource_base_folder, nn_name;
    bool with_display;
    int queue_size;

    // Declare parameters with default values
    node->declare_parameter("nn_name", "");
    node->declare_parameter("resource_base_folder", "");
    node->declare_parameter("with_display", false);
    node->declare_parameter("queue_size", 30);

    // Get parameters from ROS2 parameter server
    node->get_parameter("nn_name", nn_name);
    node->get_parameter("resource_base_folder", resource_base_folder);
    node->get_parameter("with_display", with_display);
    node->get_parameter("queue_size", queue_size);

    // Create the pipeline and device
    dai::Pipeline pipeline = create_pipeline(resource_base_folder + "/" + nn_name, with_display);
    dai::Device device(pipeline);

    // Set up detection queue
    auto detection_queue = device.getOutputQueue("detections", queue_size, false);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::Rate rate(30);

    while (rclcpp::ok())
    {
        auto detections = detection_queue->get<dai::SpatialImgDetections>();

        float nn_fps = calculateNnFps();
        if (nn_fps > 0)
        {
            RCLCPP_INFO(node->get_logger(), "NN Hertz (FPS): %.2f", nn_fps);
        }

        for (const auto &det : detections->detections)
        {
            robocops_msgs::msg::Duplo duplo_msg;
            geometry_msgs::msg::PointStamped camera_point;
            geometry_msgs::msg::PointStamped map_point;
            
            camera_point.header.frame_id = "camera";
            camera_point.header.stamp = node->get_clock()->now();
            camera_point.point.x = det.spatialCoordinates.z;
            camera_point.point.y = det.spatialCoordinates.x;
            camera_point.point.z = det.spatialCoordinates.y;

            try
            {
                map_point = tf_buffer.transform(camera_point, "base_link", tf2::durationFromSec(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(node->get_logger(), "TF2 transform failed: %s", ex.what());
                return -1;
            }

            duplo_msg.position = map_point;
            duplo_msg.score = det.confidence;
            duplo_msg.count = 1;

            duplo_msg.id = -1;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}