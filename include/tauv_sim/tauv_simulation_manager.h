/**
 * @file tauv_simulation_manager.h
 * @brief Declares the Stonefish simulation manager that owns ROS integration.
 */

#pragma once

#include <core/SimulationManager.h>
#include <entities/statics/Obstacle.h>

#undef Max

#include <optional>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

#include "tauv_sim/config_loader.h"
#include "tauv_sim/context.h"
#include "tauv_sim/kinematic_osprey.h"
#include "tauv_sim/osprey.h"
#include "tauv_sim/scenario.h"

/**
 * @brief Builds and steps the tauv_sim Stonefish world while coordinating ROS.
 */
class TauvSimulationManager : public sf::SimulationManager {
   public:
    /**
     * @brief Constructs the simulation manager with optional kinematic playback mode.
     * @param pool pool model to build
     * @param scenario props to place and, for comp2026A-D, where the robot starts
     */
    TauvSimulationManager(std::string assets_path,
                          float step_per_second,
                          std::optional<std::string> kinematic_trajectory_path = std::nullopt,
                          bool enable_cameras = true,
                          PoolModel pool = PoolModel::Basic,
                          Scenario scenario = Scenario::None);

    /**
     * @brief Builds the pool scenario, robot, materials, and sensors.
     */
    void BuildScenario() override;

    /**
     * @brief Runs per-step ROS spinning and bridge updates after physics.
     */
    void SimulationStepCompleted(sf::Scalar time_step) override;

   private:
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<ConfigLoader> config_loader_;

    std::unique_ptr<Osprey> robot_ = nullptr;
    std::unique_ptr<KinematicOsprey> kinematic_robot_ = nullptr;
    const std::string assets_path;
    std::vector<sf::StaticEntity*> props_;  // static prop entities (owned by Stonefish)
    Context ctx_;
    std::optional<std::string> kinematic_trajectory_path_;
    bool enable_cameras_;
    PoolModel pool_;
    Scenario scenario_;
};
