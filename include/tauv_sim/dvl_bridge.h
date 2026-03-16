#pragma once

#include <sensors/scalar/DVL.h>

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "tauv_sim/config.h"
#include "tauv_sim/context.h"

class DvlBridge {
   public:
    DvlBridge(sf::DVL* sensor,
              rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub,
              std::string frame_id,
              const config::osprey::sensors::Dvl& cfg);

    void on_step(const Context& ctx);

   private:
    sf::DVL* sensor_;
    const std::string frame_id_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;

    double linear_velocity_percent_noise_;
    double linear_velocity_stddev_noise_;
};
