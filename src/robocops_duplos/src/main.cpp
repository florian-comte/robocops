#include "rclcpp/rclcpp.hpp"
#include "duplos_mapper.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DuplosMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
