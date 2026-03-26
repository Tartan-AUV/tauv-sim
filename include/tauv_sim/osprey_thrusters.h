#pragma once

#include <rclcpp/rclcpp.hpp>
#include <StonefishCommon.h>
#include <core/FeatherstoneRobot.h>
#include <memory>
#include <string>
#include <array>

#include "tauv_sim/config.h"
#include "tauv_sim/thruster_bridge.h"
// Include your unified message type here
#include "tauv_msgs/msg/thruster_setpoint.hpp"


class ThrusterController {

    public:
        ThrusterController(const std::string& prefix,
                        const std::string& assets_path,
                        rclcpp::Node::SharedPtr node,
                        sf::FeatherstoneRobot* sf_robot,
                        const config::osprey::actuators::Thrusters& thruster_config,
                        const sf::Transform& body_T_cad);

        void on_step(const Context& ctx);

    private:
        rclcpp::Subscription<tauv_msgs::msg::ThrusterSetpoint>::SharedPtr forces_sub_;
        std::array<std::unique_ptr<ThrusterBridge>, 8> thruster_bridges_;

        // Might need to keep a reference to the config or node if used later in the callback
        config::osprey::actuators::Thrusters thruster_config_; // TODO

        void callback(const tauv_msgs::msg::ThrusterSetpoint::SharedPtr msg);
};
