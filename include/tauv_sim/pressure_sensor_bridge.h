#pragma once

#include <sensors/scalar/Pressure.h>

#include <nav_msgs/msg/odometry.hpp>

#include "tauv_sim/context.h"

class PressureSensorBridge {
   public:
    PressureSensorBridge(sf::Pressure* sensor,
                         rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
                         std::string frame_id);

    void on_step(const Context& ctx);

   private:
    sf::Pressure* sensor_pressure_;
    const std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};
