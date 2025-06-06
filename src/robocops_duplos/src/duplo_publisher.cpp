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

#define BUFFER_SIZE 2000
#define TOLERANCE_CM 20
#define MIN_COUNT 20
#define SCORE_THRESHOLD 0.90

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
    rgb_camera->setFps(18);

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

double calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

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

    // Debug log for parameters
    RCLCPP_DEBUG(node->get_logger(), "Parameters Loaded - nn_name: %s, resource_base_folder: %s, with_display: %s, queue_size: %d",
                 nn_name.c_str(), resource_base_folder.c_str(), with_display ? "True" : "False", queue_size);

    // Create the pipeline and device
    dai::Pipeline pipeline = create_pipeline(resource_base_folder + "/" + nn_name, with_display);
    dai::Device device(pipeline);

    // Set up detection queue
    auto detection_queue = device.getOutputQueue("detections", queue_size, false);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::Rate rate(18);

    std::vector<rclcpp::Publisher<robocops_msgs::msg::DuploArray>::SharedPtr> m_duploPubs;
    int current_duplo_id_ = 0;

    robocops_msgs::msg::Duplo new_duplo;
    geometry_msgs::msg::PointStamped camera_point;
    geometry_msgs::msg::PointStamped map_point;

    for (int i = 0; i < 4; ++i)
    {
        std::string topic = "/duplos/zone" + std::to_string(i + 1);
        m_duploPubs.push_back(node->create_publisher<robocops_msgs::msg::DuploArray>(topic, 10));
    }

    std::vector<robocops_msgs::msg::Duplo> duplos_buffer;
    std::vector<robocops_msgs::msg::Duplo> duplos_official;
    std::vector<int> already_official_;

    while (rclcpp::ok())
    {
        auto detections = detection_queue->get<dai::SpatialImgDetections>();

        for (const auto &det : detections->detections)
        {
            // RCLCPP_INFO(node->get_logger(), "Detection - Confidence: %.2f, Bounding Box: [%.2f, %.2f, %.2f, %.2f], Spatial Coordinates: [%.2f, %.2f, %.2f]",
            //              det.confidence, det.xmin, det.ymin, det.xmax, det.ymax, det.spatialCoordinates.x, det.spatialCoordinates.y, det.spatialCoordinates.z);

            if (det.confidence < SCORE_THRESHOLD)
            {
                continue;
            }

            int zone = 1;

            camera_point.header.frame_id = "camera";
            camera_point.header.stamp = node->get_clock()->now();
            camera_point.point.x = det.spatialCoordinates.z / 1000;
            camera_point.point.y = det.spatialCoordinates.x / 1000;
            camera_point.point.z = det.spatialCoordinates.y / 1000;

            try
            {
                map_point = tf_buffer.transform(camera_point, "map", tf2::durationFromSec(0.1));
                // RCLCPP_INFO(node->get_logger(), "Transform Successful - Camera Point: [%.2f, %.2f, %.2f], Map Point: [%.2f, %.2f, %.2f]",
                //             camera_point.point.x, camera_point.point.y, camera_point.point.z,
                //             map_point.point.x, map_point.point.y, map_point.point.z);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(node->get_logger(), "TF2 transform failed: %s", ex.what());
                continue;
            }

            new_duplo.position = map_point;
            new_duplo.score = det.confidence;
            new_duplo.count = 1;
            new_duplo.id = -1;

            // Add info on x, y, z in map frame
            bool found = false;
            for (robocops_msgs::msg::Duplo &existing_duplo : duplos_buffer)
            {
                if (calculate_distance(existing_duplo.position.point, new_duplo.position.point) < TOLERANCE_CM / 100.0)
                {
                    found = true;

                    existing_duplo.position.point.x = (new_duplo.position.point.x + (existing_duplo.count) * existing_duplo.position.point.x) / (existing_duplo + 1);
                    existing_duplo.position.point.y = (new_duplo.position.point.y + (existing_duplo.count) * existing_duplo.position.point.y) / (existing_duplo + 1);
                    existing_duplo.position.point.z = (new_duplo.position.point.z + (existing_duplo.count) * existing_duplo.position.point.z) / (existing_duplo + 1);

                    if (++existing_duplo.count >= MIN_COUNT)
                    {
                        if (std::find(already_official_.begin(), already_official_.end(), existing_duplo.id) == already_official_.end())
                        {
                            duplos_official.push_back(existing_duplo);
                            already_official_.push_back(existing_duplo.id);
                            RCLCPP_INFO(node->get_logger(), "Duplo %d became official in zone %d", existing_duplo.id, zone);
                        }
                    }
                }
            }

            if (!found)
            {
                if (duplos_buffer.size() >= BUFFER_SIZE)
                {
                    RCLCPP_WARN(node->get_logger(), "Buffer full for zone %d, discarding duplo", zone);
                    continue;
                }

                new_duplo.id = current_duplo_id_++;
                duplos_buffer.push_back(new_duplo);
                RCLCPP_INFO(node->get_logger(), "Added new duplo to buffer in zone %d", zone);
            }
        }

        if (!duplos_official.empty())
        {
            robocops_msgs::msg::DuploArray array_msg;
            array_msg.duplos = duplos_official;
            m_duploPubs[0]->publish(array_msg);
            RCLCPP_INFO(node->get_logger(), "Published %zu official duplos.", duplos_official.size());
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
