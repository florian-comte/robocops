#include <rclcpp/rclcpp.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class DetectionsViewer : public rclcpp::Node {
public:
    DetectionsViewer()
    : Node("detections_viewer") {
        // Create subscribers for detection and RGB image topics
        m_detections_subscriber = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/camera/detections", 4, std::bind(&DetectionsViewer::detections_callback, this, std::placeholders::_1));
        m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/raw_rgb", 4, std::bind(&DetectionsViewer::image_callback, this, std::placeholders::_1));
    }

private:
    void detections_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) {
        if (!m_image) return; // Wait for the image to be available

        // Visualize detections on the RGB image
        cv::Mat img = cv_bridge::toCvShare(m_image, "bgr8")->image;

        for (const auto& detection : msg->detections) {
            cv::Rect box(
                detection.bbox.center.position.x - detection.bbox.size_x / 2, 
                detection.bbox.center.position.y - detection.bbox.size_y / 2,
                detection.bbox.size_x, 
                detection.bbox.size_y
            );

            // Draw the bounding box around the detected object
            cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
            
            // Create strings to display the confidence score and depth (Z) position
            std::string prob = "Confidence: " + std::to_string(detection.results[0].score) + "%";
            std::string depthZ = "Z: " + std::to_string(detection.position.z * 100.0f) + " cm";
            cv::putText(img, prob, cv::Point(box.x, box.y - 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, depthZ, cv::Point(box.x, box.y - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // Show the image with detections
        cv::imshow("Detected Objects", img);
        cv::waitKey(1); // Refresh the window
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        m_image = msg; // Store the received image
    }

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detections_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    sensor_msgs::msg::Image::SharedPtr m_image;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionsViewer>());
    rclcpp::shutdown();
    return 0;
}
