#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/logging.hpp"

class MessageSizeSubscriber : public rclcpp::Node {
public:
    MessageSizeSubscriber() : Node("test") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/raw_rgb", 10, std::bind(&MessageSizeSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Calculate the message size using the size of the image data
        size_t message_size = sizeof(sensor_msgs::msg::Image) + msg->data.size();
        RCLCPP_INFO(this->get_logger(), "Received message size: %zu bytes", message_size);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageSizeSubscriber>());
    rclcpp::shutdown();
    return 0;
}
