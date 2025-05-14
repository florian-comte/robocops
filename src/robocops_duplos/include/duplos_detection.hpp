#ifndef ROBOCOPS_DUPLOS_DETECTION
#define ROBOCOPS_DUPLOS_DETECTION

#include <rclcpp/rclcpp.hpp>
#include "robocops_msgs/msg/duplo.hpp"
#include "robocops_msgs/msg/duplo_array.hpp"
#include "robocops_msgs/srv/get_duplos.hpp"
#include "robocops_msgs/srv/remove_duplo.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <deque>
#include <unordered_set>
#include <vector>

struct DuploObservation
{
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Point total_position;
    int count;
};

class DuplosDetection : public rclcpp::Node
{
public:
    DuplosDetection();

private:
    void poseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    geometry_msgs::msg::Point getInterpolatedPose(const rclcpp::Time &stamp);
    bool isNear(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b, double threshold);
    void detectionCallback(const robocops_msgs::msg::Duplo::SharedPtr msg);
    void publishDuplos();
    void handleGetDuplos(const std::shared_ptr<robocops_msgs::srv::GetDuplos::Request> request,
                         std::shared_ptr<robocops_msgs::srv::GetDuplos::Response> response);
    void handleRemoveDuplo(const std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Request> request,
                           std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Response> response);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<robocops_msgs::msg::Duplo>::SharedPtr detection_sub_;
    rclcpp::Publisher<robocops_msgs::msg::DuploArray>::SharedPtr duplo_pub_;
    rclcpp::Service<robocops_msgs::srv::GetDuplos>::SharedPtr get_duplos_srv_;
    rclcpp::Service<robocops_msgs::srv::RemoveDuplo>::SharedPtr remove_duplo_srv_;

    std::deque<geometry_msgs::msg::PointStamped> pose_buffer_;
    std::vector<DuploObservation> observed_duplos_;
    std::vector<robocops_msgs::msg::Duplo> confirmed_duplos_;
    std::unordered_set<std::string> blacklist_;
    double proximity_threshold_ = 0.1; // meters
    int detection_threshold_ = 5;
    size_t max_pose_buffer_size_ = 10; // max size of robot pose history buffer
};

#endif // ROBOCOPS_DUPLOS_DETECTION
