/**
 * @file osprey.h
 * @brief Declares the dynamic Stonefish model for the Osprey vehicle.
 */

#pragma once

#include <core/FeatherstoneRobot.h>
#include <entities/solids/Polyhedron.h>

#include <array>
#include <memory>
#include <string>

#undef Max

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "tauv_msgs/msg/thruster_setpoint.hpp"
#include "tauv_sim/config.h"
#include "tauv_sim/config_loader.h"
#include "tauv_sim/context.h"
#include "tauv_sim/osprey_sensors.h"
#include "tauv_sim/thruster_bridge.h"

/**
 * @brief Owns the simulated Osprey robot, including sensors and thruster interfaces.
 */
class Osprey {
   public:
    /**
     * @brief Builds the Stonefish Osprey model and wires ROS actuator/sensor bridges.
     */
    Osprey(const std::string prefix,
           const std::string& assets_path,
           rclcpp::Node::SharedPtr node,
           std::shared_ptr<ConfigLoader> config_loader,
           bool enable_cameras = true);
    ~Osprey() = default;

    /**
     * @brief Returns the Stonefish robot object to register in the scenario.
     */
    sf::FeatherstoneRobot* get_stonefish_robot();

    /**
     * @brief Runs per-step sensor publication and actuator telemetry updates.
     */
    void on_step(const Context& ctx);

   private:
    std::string prefix_;
    sf::Polyhedron* base_link_;
    std::shared_ptr<sf::FeatherstoneRobot> construct_robot();

    sf::FeatherstoneRobot* sf_robot_;
    std::unique_ptr<OspreySensors> sensors_;

    std::array<std::unique_ptr<ThrusterBridge>, 8> thruster_bridges_;
    std::array<std::shared_ptr<rclcpp::Subscription<tauv_msgs::msg::ThrusterSetpoint>>, 8>
        thruster_setpoint_subs_{};

    // Configuration
    config::osprey::actuators::Thrusters thruster_config_;

    sf::Matrix3 compute_principal_inertia_axes(const config::osprey::InertialBuoyancy& cfg);
};
