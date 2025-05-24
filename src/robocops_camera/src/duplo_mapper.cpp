#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DetectionTransformer : public rclcpp::Node
{
public:
    DetectionTransformer()
        : Node("duplo_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        detection_sub_ = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/camera/detections", 10,
            std::bind(&DetectionTransformer::callback, this, std::placeholders::_1));
    }

private:
    void callback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg)
    {
        for (const auto &detection : msg->detections)
        {
            geometry_msgs::msg::PointStamped camera_point;
            camera_point.header = detection.header;
            camera_point.point.x = detection.position.x;
            camera_point.point.y = detection.position.y;
            camera_point.point.z = detection.position.z;

            try
            {
                geometry_msgs::msg::PointStamped map_point =
                    tf_buffer_.transform(camera_point, "map", tf2::durationFromSec(0.1));

                RCLCPP_INFO(this->get_logger(), "Transformed position: (%.2f, %.2f, %.2f)",
                            map_point.point.x, map_point.point.y, map_point.point.z);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF2 transform failed: %s", ex.what());
            }
        }
    }

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detection_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
