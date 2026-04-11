/**
 * @file pressure_sensor_bridge.h
 * @brief Declares the bridge that publishes simulated pressure/depth measurements.
 */

#pragma once

#include <sensors/scalar/Pressure.h>

#include <nav_msgs/msg/odometry.hpp>
#include <string>

#include "tauv_sim/context.h"

class PressureSensorBridge {
   public:
    /**
     * @brief Constructs a pressure-to-odometry ROS publisher bridge.
     */
    PressureSensorBridge(sf::Pressure* sensor,
                         rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
                         std::string frame_id);

    /**
     * @brief Publishes a new depth estimate when pressure data is available.
     */
    void on_step(const Context& ctx);

   private:
    sf::Pressure* sensor_pressure_;
    const std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};
