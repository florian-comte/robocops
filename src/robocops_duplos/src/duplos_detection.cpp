#include "duplos_detection.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

DuplosDetection::DuplosDetection() : Node("duplos_detection")
{
    m_detectionsSub = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
        "/camera/detections", 20, std::bind(&DetectionsDisplayPublisher::detectionsCallback, this, std::placeholders::_1));

    // pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //     "/robot/position", 20, std::bind(&Robocops::poseCallback, this, std::placeholders::_1));

    duplo_pub_ = this->create_publisher<robocops_msgs::msg::DuploArray>("/duplos", 10);

    get_duplos_srv_ = this->create_service<robocops_msgs::srv::GetDuplos>(
        "get_duplos", std::bind(&Robocops::handleGetDuplos, this, std::placeholders::_1, std::placeholders::_2));

    remove_duplo_srv_ = this->create_service<robocops_msgs::srv::RemoveDuplo>(
        "remove_duplo", std::bind(&Robocops::handleRemoveDuplo, this, std::placeholders::_1, std::placeholders::_2));
}

// void Robocops::poseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
// {
//     pose_buffer_.emplace_back(*msg);
//     while (pose_buffer_.size() > max_pose_buffer_size_)
//     {
//         pose_buffer_.pop_front();
//     }
// }

geometry_msgs::msg::Point Robocops::getInterpolatedPose(const rclcpp::Time &stamp)
{
    if (pose_buffer_.empty())
        return geometry_msgs::msg::Point();

    geometry_msgs::msg::PointStamped closest_pose;
    rclcpp::Duration min_delta = rclcpp::Duration::from_nanoseconds(std::numeric_limits<int64_t>::max());

    for (const auto &pose : pose_buffer_)
    {
        auto delta = rclcpp::Time(pose.header.stamp) - stamp;
        if (std::abs(delta.nanoseconds()) < min_delta.nanoseconds())
        {
            min_delta = rclcpp::Duration(std::abs(delta.nanoseconds()));
            closest_pose = pose;
        }
    }
    return closest_pose.point;
}

bool Robocops::isNear(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b, double threshold)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < threshold;
}

void Robocops::detectionCallback(const robocops_msgs::msg::Duplo::SharedPtr msg)
{
    geometry_msgs::msg::Point robot_pose_at_detection = getInterpolatedPose(rclcpp::Time(msg->header.stamp));

    geometry_msgs::msg::Point mapped_position;
    mapped_position.x = robot_pose_at_detection.x + msg->position.x;
    mapped_position.y = robot_pose_at_detection.y + msg->position.y;
    mapped_position.z = robot_pose_at_detection.z + msg->position.z;

    for (auto &d : observed_duplos_)
    {
        if (isNear(d.position, mapped_position, proximity_threshold_))
        {
            d.count++;
            d.total_position.x += mapped_position.x;
            d.total_position.y += mapped_position.y;
            d.total_position.z += mapped_position.z;

            if (d.count >= detection_threshold_)
            {
                std::string key = std::to_string((int)(d.position.x * 100)) + "," +
                                  std::to_string((int)(d.position.y * 100));
                if (blacklist_.count(key) == 0)
                {
                    robocops_msgs::msg::Duplo confirmed;
                    confirmed.position.x = d.total_position.x / d.count;
                    confirmed.position.y = d.total_position.y / d.count;
                    confirmed.position.z = d.total_position.z / d.count;
                    confirmed_duplos_.push_back(confirmed);
                    publishDuplos();
                    d.count = -9999; // prevent re-adding
                }
            }
            return;
        }
    }

    // New detection
    DuploObservation new_obs;
    new_obs.position = mapped_position;
    new_obs.total_position = mapped_position;
    new_obs.count = 1;
    observed_duplos_.push_back(new_obs);
}

void Robocops::publishDuplos()
{
    robocops_msgs::msg::DuploArray msg;
    msg.duplos = confirmed_duplos_;
    duplo_pub_->publish(msg);
}

void Robocops::handleGetDuplos(const std::shared_ptr<robocops_msgs::srv::GetDuplos::Request>,
                               std::shared_ptr<robocops_msgs::srv::GetDuplos::Response> response)
{
    response->duplos = confirmed_duplos_;
}

void Robocops::handleRemoveDuplo(const std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Request> request,
                                 std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Response> response)
{
    auto it = std::remove_if(confirmed_duplos_.begin(), confirmed_duplos_.end(), [&](const auto &d)
                             { return isNear(d.position, request->position, proximity_threshold_); });
    if (it != confirmed_duplos_.end())
    {
        confirmed_duplos_.erase(it, confirmed_duplos_.end());
        std::string key = std::to_string((int)(request->position.x * 100)) + "," +
                          std::to_string((int)(request->position.y * 100));
        blacklist_.insert(key);
        response->success = true;
        publishDuplos();
    }
    else
    {
        response->success = false;
    }
}
