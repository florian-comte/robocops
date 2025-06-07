#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// DepthAI includes
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImuConverter.hpp"

dai::Pipeline createPipeline()
{
    dai::Pipeline pipeline;

    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutImu->setStreamName("imu");

    // Configure IMU
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->enableFirmwareUpdate(true);
    imu->setMaxBatchReports(20);

    imu->out.link(xoutImu->input);

    return pipeline;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robocops_camera_imu");

    // Declare parameters
    node->declare_parameter<int>("imu_mode", 1);
    node->declare_parameter<double>("linear_accel_covariance", 0.02);
    node->declare_parameter<double>("angular_vel_covariance", 0.0);
    node->declare_parameter<bool>("enable_ros_base_time_update", true);
    node->declare_parameter<bool>("enableRosBaseTimeUpdate", true);

    // Get parameters
    double linearAccelCovariance = node->get_parameter("linear_accel_covariance").as_double();
    double angularVelCovariance = node->get_parameter("angular_vel_covariance").as_double();
    int imuModeParam = node->get_parameter("imu_mode").as_int();
    bool enableRosBaseTimeUpdate = node->get_parameter("enable_ros_base_time_update").as_bool();

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);

    // Setup pipeline and device
    dai::Pipeline pipeline = createPipeline();
    dai::Device device(pipeline);

    auto imuQueue = device.getOutputQueue("imu", 30, false);

    dai::rosBridge::ImuConverter imuConverter("camera", imuMode, linearAccelCovariance, angularVelCovariance);
    if (enableRosBaseTimeUpdate)
    {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        imuQueue,
        node,
        std::string("imu/data_raw"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "camera");

    imuPublish.addPublisherCallback();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
