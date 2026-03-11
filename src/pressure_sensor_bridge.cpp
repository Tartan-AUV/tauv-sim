#include "tauv_sim/pressure_sensor_bridge.h"

#include <sensors/scalar/Pressure.h>

PressureSensorBridge::PressureSensorBridge(
    sf::Pressure* sensor,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
    std::string frame_id)
    : sensor_pressure_(sensor), frame_id_(std::move(frame_id)), pub_(pub) {}

void PressureSensorBridge::on_step(const Context& ctx) {
    if (sensor_pressure_->isNewDataAvailable()) {
        // std::cout << "Pressure: " << sensor_pressure_->getLastValue(0) << " Pa" << std::endl;
        const float pressure = sensor_pressure_->getLastValue(0);
        const float stddev = sensor_pressure_->getSensorChannelDescription(0).stdDev / 9806.65;  // Convert pressure stddev to depth stddev

        nav_msgs::msg::Odometry depth_msg;
        depth_msg.header.stamp = ctx.get_ros_time();
        depth_msg.header.frame_id = "odom";
        depth_msg.child_frame_id = "depth_link";
        depth_msg.pose.pose.position.z = -pressure / 9806.65;
        depth_msg.pose.covariance.fill(1e6);
        depth_msg.twist.covariance.fill(1e6);
        depth_msg.pose.covariance[14] = stddev * stddev; // Convert from variance to covariance
        pub_->publish(depth_msg);
    }
}
