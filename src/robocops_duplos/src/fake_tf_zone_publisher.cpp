#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class FakeTFAndZonePublisher : public rclcpp::Node
{
public:
    FakeTFAndZonePublisher() : Node("fake_tf_and_zone_publisher"), zone_(1)
    {
        zone_pub_ = this->create_publisher<std_msgs::msg::Int32>("/zone", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&FakeTFAndZonePublisher::publish, this));
    }

private:
    void publish()
    {
        // Publish zone message
        std_msgs::msg::Int32 zone_msg;
        zone_msg.data = zone_;
        zone_pub_->publish(zone_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing zone: %d", zone_);

        // Cycle through zones 1-4
        zone_ = zone_ % 4 + 1;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zone_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int zone_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeTFAndZonePublisher>());
    rclcpp::shutdown();
    return 0;
}
