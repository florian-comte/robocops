#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <robocops_msgs/msg/duplo.hpp>
#include <robocops_msgs/msg/duplo_array.hpp>
#include <robocops_msgs/srv/remove_duplo.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define BUFFER_SIZE 100
#define TOLERANCE_CM 20
#define MIN_COUNT 20

#define DROPPING_ZONE_MIN_X 0.0
#define DROPPING_ZONE_MAX_X 1.0
#define DROPPING_ZONE_MIN_Y 0.0
#define DROPPING_ZONE_MAX_Y 1.0

class DuploProcessor : public rclcpp::Node
{
public:
    DuploProcessor() : Node("duplo_processor"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        m_detectionsSub = this->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>(
            "/camera/detections", 20,
            std::bind(&DuploProcessor::detectionsCallback, this, std::placeholders::_1));

        m_zoneSub = this->create_subscription<std_msgs::msg::Int32>(
            "/zone", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                current_zone_ = msg->data;
            });

        for (int i = 0; i < 4; ++i)
        {
            std::string topic = "/duplos/zone" + std::to_string(i + 1);
            m_duploPubs.push_back(this->create_publisher<robocops_msgs::msg::DuploArray>(topic, 10));
        }

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&DuploProcessor::publishOfficialDuplos, this));

        remove_duplo_service_ = this->create_service<robocops_msgs::srv::RemoveDuplo>(
            "remove_duplo",
            std::bind(&DuploProcessor::handleRemoveDuplo, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr m_detectionsSub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_zoneSub;
    std::vector<rclcpp::Publisher<robocops_msgs::msg::DuploArray>::SharedPtr> m_duploPubs;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<robocops_msgs::srv::RemoveDuplo>::SharedPtr remove_duplo_service_;

    int current_zone_ = 1;

    int current_duplo_id_ = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<robocops_msgs::msg::Duplo> zone1_duplos_buffer_;
    std::vector<robocops_msgs::msg::Duplo> zone2_duplos_buffer_;
    std::vector<robocops_msgs::msg::Duplo> zone3_duplos_buffer_;
    std::vector<robocops_msgs::msg::Duplo> zone4_duplos_buffer_;

    std::vector<int> already_official_;

    std::vector<robocops_msgs::msg::Duplo> zone1_duplos_;
    std::vector<robocops_msgs::msg::Duplo> zone2_duplos_;
    std::vector<robocops_msgs::msg::Duplo> zone3_duplos_;
    std::vector<robocops_msgs::msg::Duplo> zone4_duplos_;

    double calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    bool is_in_dropping_zone(const geometry_msgs::msg::Point &pt)
    {
        return pt.x >= DROPPING_ZONE_MIN_X && pt.x <= DROPPING_ZONE_MAX_X &&
               pt.y >= DROPPING_ZONE_MIN_Y && pt.y <= DROPPING_ZONE_MAX_Y;
    }

    std::vector<robocops_msgs::msg::Duplo> &get_buffer(int zone)
    {
        switch (zone)
        {
        case 1:
            return zone1_duplos_buffer_;
        case 2:
            return zone2_duplos_buffer_;
        case 3:
            return zone3_duplos_buffer_;
        case 4:
            return zone4_duplos_buffer_;
        default:
            throw std::invalid_argument("Invalid zone number");
        }
    }

    std::vector<robocops_msgs::msg::Duplo> &get_official_list(int zone)
    {
        switch (zone)
        {
        case 1:
            return zone1_duplos_;
        case 2:
            return zone2_duplos_;
        case 3:
            return zone3_duplos_;
        case 4:
            return zone4_duplos_;
        default:
            throw std::invalid_argument("Invalid zone number");
        }
    }

    int add_duplo_in_buffer(int zone, robocops_msgs::msg::Duplo &duplo)
    {
        // if (is_in_dropping_zone(duplo.position.point))
        // {
        //     return -1;
        // }

        auto &buffer = get_buffer(zone);
        auto &official = get_official_list(zone);

        for (auto &existing : buffer)
        {
            if (calculate_distance(existing.position.point, duplo.position.point) < TOLERANCE_CM / 100.0)
            {
                existing.count += 1;
                if (existing.count >= MIN_COUNT)
                {
                    if (std::find(already_official_.begin(), already_official_.end(), existing.id) == already_official_.end())
                    {
                        official.push_back(existing);
                        already_official_.push_back(existing.id);
                        RCLCPP_INFO(this->get_logger(), "Duplo %d became official in zone %d", existing.id, zone);
                    }
                }
                return 1;
            }
        }

        if (buffer.size() >= BUFFER_SIZE)
        {
            RCLCPP_WARN(this->get_logger(), "Buffer full for zone %d, discarding duplo", zone);
            return -2;
        }

        duplo.count = 1;
        duplo.id = current_duplo_id_++;
        buffer.push_back(duplo);
        RCLCPP_INFO(this->get_logger(), "Added new duplo to buffer in zone %d", zone);
        return 0;
    }

    int transform_coordinates(
        const std_msgs::msg::Header &header,
        const depthai_ros_msgs::msg::SpatialDetection &untransformed_duplo,
        robocops_msgs::msg::Duplo &transformed_duplo)
    {
        geometry_msgs::msg::PointStamped camera_point;
        camera_point.header = header;
        camera_point.point.x = untransformed_duplo.position.x;
        camera_point.point.y = untransformed_duplo.position.y;
        camera_point.point.z = untransformed_duplo.position.z;

        // geometry_msgs::msg::PointStamped map_point;
        // try
        // {
        //     RCLCPP_INFO(this->get_logger(), "Camera point frame: %s", camera_point.header.frame_id.c_str());
        //     map_point = tf_buffer_.transform(camera_point, "base_link", tf2::durationFromSec(0.1));
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "TF2 transform failed: %s", ex.what());
        //     return -1;
        // }

        // transformed_duplo.position = map_point;

        transformed_duplo.position = camera_point;
        transformed_duplo.score = untransformed_duplo.results[0].score;
        transformed_duplo.count = 1;

        transformed_duplo.id = -1;

        return 0;
    }

    void detectionsCallback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg)
    {
        for (const auto &detection : msg->detections)
        {
            robocops_msgs::msg::Duplo duplo;
            if (transform_coordinates(msg->header, detection, duplo) == 0)
            {
                add_duplo_in_buffer(current_zone_, duplo);
            }
        }
    }

    void publishOfficialDuplos()
    {
        for (int zone = 1; zone <= 4; ++zone)
        {
            auto &official_list = get_official_list(zone);
            if (!official_list.empty())
            {
                robocops_msgs::msg::DuploArray array_msg;
                array_msg.duplos = official_list;
                m_duploPubs[zone - 1]->publish(array_msg);
            }
        }
    }

    void handleRemoveDuplo(
        const std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Request> request,
        std::shared_ptr<robocops_msgs::srv::RemoveDuplo::Response> response)
    {
        bool found = false;

        for (int zone = 1; zone <= 4; ++zone)
        {
            auto &official = get_official_list(zone);
            auto it = std::remove_if(official.begin(), official.end(),
                                     [&](const robocops_msgs::msg::Duplo &d)
                                     {
                                         return d.id == request->id;
                                     });
            if (it != official.end())
            {
                official.erase(it, official.end());
                found = true;
            }

            auto &buffer = get_buffer(zone);
            it = std::remove_if(buffer.begin(), buffer.end(),
                                [&](const robocops_msgs::msg::Duplo &d)
                                {
                                    return d.id == request->id;
                                });
            if (it != buffer.end())
            {
                buffer.erase(it, buffer.end());
                found = true;
            }
        }

        if (found)
        {
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Removed Duplo with ID %d", request->id);
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Failed to remove Duplo with ID %d: not found", request->id);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DuploProcessor>());
    rclcpp::shutdown();
    return 0;
}
