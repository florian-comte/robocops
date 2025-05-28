#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

class DetectionsDisplayPublisher : public rclcpp::Node {
public:
    DetectionsDisplayPublisher() : Node("duplo_detection_viewer") {
        // Subscribe to the compressed raw RGB image stream
        m_rgbSub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/raw_rgb/compressed", 30, std::bind(&DetectionsDisplayPublisher::rgbCallback, this, std::placeholders::_1));
        
        // Subscribe to the spatial detections stream
        m_detectionsSub = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/camera/detections", 30, std::bind(&DetectionsDisplayPublisher::detectionsCallback, this, std::placeholders::_1));
        
        // Publisher to output the RGB image with detections overlayed
        m_rgbWithDetectionsPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb_with_detections", 30);
    }

private:
    void rgbCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            // Decompress the image using cv_bridge
            m_rgbFrame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Could not convert using cv_bridge: %s", 
                e.what());
            return;
        }
    }

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
        if (elapsed > std::chrono::seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        // Loop through each detection and overlay it on the image
        for (const auto& detection : detections->detections) {
            // Check if bounding box data exists
            if (detection.bbox.center.position.x == 0 && detection.bbox.center.position.y == 0) {
                continue;
            }

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
            std::string prob = "Confidence: " + std::to_string(detection.results[0].score * 100.0f) + "%";
            std::string depthZ = "Z: " + std::to_string(detection.position.z * 100.0f) + " cm";
            cv::putText(m_rgbFrame, prob, cv::Point(box.x, box.y - 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(m_rgbFrame, depthZ, cv::Point(box.x, box.y - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // Display FPS in top-right corner
        std::stringstream fpsStr;
        fpsStr << "FPS: " << std::fixed << std::setprecision(1) << fps;
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(fpsStr.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point fpsPosition(m_rgbFrame.cols - textSize.width - 10, textSize.height + 10);
        cv::putText(m_rgbFrame, fpsStr.str(), fpsPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // Publish the image with detections overlaid
        sensor_msgs::msg::Image::SharedPtr outputMsg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", m_rgbFrame).toImageMsg();
        m_rgbWithDetectionsPub->publish(*outputMsg);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_rgbSub;
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detectionsSub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgbWithDetectionsPub;

    cv::Mat m_rgbFrame; // CV Frame from RGB stream

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
