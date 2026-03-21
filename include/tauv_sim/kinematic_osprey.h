/**
 * @file kinematic_osprey.h
 * @brief Declares a trajectory-driven Osprey model used for kinematic playback mode.
 */

#pragma once

#include <entities/animation/PWLTrajectory.h>

#include <memory>
#include <string>

#undef Max

#include "tauv_sim/config_loader.h"
#include "tauv_sim/context.h"
#include "tauv_sim/osprey_sensors.h"
#include "tauv_sim/trajectory_loader.h"

namespace sf {
class SimulationManager;
class AnimatedEntity;
}  // namespace sf

class KinematicOsprey {
   public:
    /**
     * @brief Builds a kinematic Osprey entity and its sensor stack.
     */
    KinematicOsprey(std::string prefix,
                    const std::string& assets_path,
                    rclcpp::Node::SharedPtr node,
                    std::shared_ptr<ConfigLoader> config_loader,
                    const trajectory::Spec& trajectory_spec,
                    bool enable_cameras = true);

    /**
     * @brief Registers the animated body and sensors with the simulation manager.
     */
    void add_to_simulation(sf::SimulationManager* sim_manager);

    /**
     * @brief Advances ROS-side sensor publication for the current sim step.
     */
    void on_step(const Context& ctx);

    /**
     * @brief Returns the underlying animated entity used in Stonefish.
     */
    sf::AnimatedEntity* get_entity();

   private:
    /**
     * @brief Converts a loaded trajectory spec into a Stonefish PWL trajectory.
     */
    void build_trajectory(const trajectory::Spec& spec);
    std::string prefix_;
    std::unique_ptr<sf::AnimatedEntity> animated_body_;
    std::unique_ptr<sf::PWLTrajectory> trajectory_;
    std::unique_ptr<OspreySensors> sensors_;
};
