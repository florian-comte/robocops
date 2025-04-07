/*
 * detections_display_publisher.cpp
 *
 * This file contains the implementation of the `detections_display_publisher` class,
 * which is a ROS 2 node responsible for subscribing to raw RGB image data and spatial
 * detection information from the camera and overlaying bounding boxes, labels, 
 * confidence scores, and spatial coordinates on the image (and then publishing the processed
 * image with detections on a new topic).
 *
 * The node listens to two topics: `/camera/raw_rgb` for the raw RGB images and 
 * `/camera/detections` for the spatial detection data, and it outputs the annotated
 * image to the `/camera/rgb_with_detections` topic.
 */

#include <chrono>
#include "depthai/depthai.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

/**
 * @class DetectionsDisplayPublisher
 * @brief A ROS2 node that subscribes to raw RGB images and spatial detection messages,
 *        and publishes the RGB image with the detections overlaid.
 */
class DetectionsDisplayPublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructs a DetectionsDisplayPublisher node.
     *
     * Initializes the node, subscribes to raw RGB and detection streams, and creates a
     * publisher for the RGB image with detections overlaid.
     */
    DetectionsDisplayPublisher() : Node("detections_display_publisher") {
        m_rgbSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/raw_rgb", 30, std::bind(&DetectionsDisplayPublisher::rgbCallback, this, std::placeholders::_1));
        
        // Subscription to the spatial detections stream
        m_detectionsSub = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/camera/detections", 30, std::bind(&DetectionsDisplayPublisher::detectionsCallback, this, std::placeholders::_1));
            
        // Publisher to output the RGB image with detections overlayed
        m_rgbWithDetectionsPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_with_detections", 30);
    }

private:
    /**
     * @brief Callback function for handling raw RGB images.
     *
     * Converts the received ROS image message into an OpenCV `Mat` format.
     *
     * @param msg The incoming image message.
     */
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            m_rgbFrame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            return;
        }
    }

    /**
     * @brief Callback function for handling spatial detection messages.
     *
     * Processes each detection, draws bounding boxes, and overlays confidence and depth information on the RGB image.
     * Then, it publishes the updated image with detections overlaid.
     *
     * @param detections The received spatial detection array message.
     */
    void detectionsCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections) {
        // If the RGB frame is empty, skip overlaying detections
        if (m_rgbFrame.empty()) {
            RCLCPP_WARN(this->get_logger(), "RGB frame is empty, skipping detection overlay.");
            return;
        }
        
        // Compute FPS
        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - startTime);
        if(elapsed > std::chrono::seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        // Loop through each detection and overlay it on the image
        for (const auto& detection : detections->detections) {
            // Create a bounding box based on the detection's center and size
            cv::Rect box(
                detection.bbox.center.position.x - detection.bbox.size_x / 2, 
                detection.bbox.center.position.y - detection.bbox.size_y / 2,
                detection.bbox.size_x, 
                detection.bbox.size_y
            );

            // Draw the bounding box around the detected object
            cv::rectangle(m_rgbFrame, box, cv::Scalar(0, 255, 0), 2);
            
            // Create strings to display the confidence score and depth (Z) position
            std::string prob = "Confidence: " + std::to_string(detection.results[0].score) + "%";
            std::string depthZ = "Z: " + std::to_string(detection.position.z * 100.0f) + " cm";
            cv::putText(m_rgbFrame, prob, cv::Point(box.x, box.y - 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(m_rgbFrame, depthZ, cv::Point(box.x, box.y - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(m_rgbFrame, fpsStr.str(), cv::Point(2,  2), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255));

        // Publish the image with detections overlayed
        sensor_msgs::msg::Image::SharedPtr outputMsg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", m_rgbFrame).toImageMsg();
        m_rgbWithDetectionsPub->publish(*outputMsg);
    }

    // Subscription to the raw RGB image stream
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_rgbSub;

    // Subscription to the spatial detections stream
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detectionsSub;

    // Publisher to output the RGB image with detections overlayed
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgbWithDetectionsPub;

    // CV Frame from RGB stream
    cv::Mat m_rgbFrame;

    // FPS handling
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionsDisplayPublisher>());
    rclcpp::shutdown();
    
    return 0;
}
