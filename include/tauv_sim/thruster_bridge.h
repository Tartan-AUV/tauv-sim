/**
 * @file thruster_bridge.h
 * @brief Declares the bridge between ROS thruster commands and Stonefish thrusters.
 */

#pragma once

#include <actuators/Thruster.h>

#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>

#include "tauv_sim/config.h"
#include "tauv_msgs/msg/esc_telemetry.hpp"
#include "tauv_msgs/msg/thruster_setpoint.hpp"
#include "tauv_sim/context.h"

/**
 * @brief Applies incoming ESC commands and publishes simulated ESC telemetry.
 */
class ThrusterBridge {
   public:
    /**
     * @brief Constructs a bridge for one thruster/ESC channel.
     */
    ThrusterBridge(sf::Thruster* thruster,
                   rclcpp::Publisher<tauv_msgs::msg::EscTelemetry>::SharedPtr pub,
                   double telemetry_rate,
                   uint8_t thruster_esc_id,
                   const config::osprey::actuators::Thrusters& cfg);

    /**
     * @brief Publishes ESC telemetry at the configured rate.
     */
    void on_step(const Context& ctx);

    void set_speed(float rad_per_sec);

   private:
    sf::Thruster* thruster_;
    rclcpp::Publisher<tauv_msgs::msg::EscTelemetry>::SharedPtr pub_;
    const std::chrono::duration<double, std::nano> period_ns_;
    const uint8_t thruster_esc_id_;
    SimTime prev_pub_time_ = std::chrono::seconds{0};
    const config::osprey::actuators::Thrusters& c_;
};
