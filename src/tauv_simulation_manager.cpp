/**
 * @file tauv_simulation_manager.cpp
 * @brief Implements scenario setup and per-step ROS integration for tauv_sim.
 */

#include "tauv_sim/tauv_simulation_manager.h"

#include <entities/statics/Obstacle.h>
#include <graphics/OpenGLDataStructs.h>

#include <cmath>
#include <iostream>

#include "tauv_sim/comp.h"
#include "tauv_sim/registry.h"
#include "tauv_sim/spawn.h"
#include "tauv_sim/static_prop.h"
#include "tauv_sim/trajectory_loader.h"

namespace {

/** @brief The simple Irvine pool (single mesh, no textures, exact triangle-mesh collision). */
props::StaticPropSpec BasicPool() {
    return props::StaticPropSpec{
        "pool",
        {{"pool/irvine_pool.obj", ""}},
        props::collision::VisualMesh{},
        materials::POOL.name,
    };
}

}  // namespace

TauvSimulationManager::TauvSimulationManager(std::string assets_path,
                                             float step_per_second,
                                             std::optional<std::string> kinematic_trajectory_path,
                                             bool enable_cameras,
                                             PoolModel pool,
                                             Scenario scenario)
    : SimulationManager(step_per_second),
      assets_path(std::move(assets_path) + "/"),
      kinematic_trajectory_path_(std::move(kinematic_trajectory_path)),
      enable_cameras_(enable_cameras),
      pool_(pool),
      scenario_(scenario) {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    node_ = std::make_shared<rclcpp::Node>("tauv_sim", options);
    config_loader_ = std::make_shared<ConfigLoader>(node_);

    executor_.add_node(node_);
}

/**
 * @brief Builds the simulated pool world and either dynamic or kinematic Osprey.
 */
void TauvSimulationManager::BuildScenario() {
    // Ocean configuration
    getMaterialManager()->CreateFluid("OceanWater", 1000.0, 0.002, 1.33);
    EnableOcean(0.0, getMaterialManager()->getFluid("OceanWater"));

    // Materials
    for (const auto& m : materials::all_materials) {
        CreateMaterial(m.name, m.density, m.restitution);
    }

    // Looks
    for (const auto& l : looks::all_looks) {
        create_look(*this, l, assets_path);
    }
    // Competition looks are created on first use by any comp pool or scenario
    bool comp_looks_created = false;
    const auto ensure_comp_looks = [&] {
        if (!comp_looks_created) {
            comp::create_looks(*this, assets_path);
            comp_looks_created = true;
        }
    };

    // Solid colors
    CreateLook("osprey_red_hull", sf::Color::RGB(1.0f, 0.0f, 0.0f), 0.8f, 0.8f, 0.1f);

    const auto addProps = [this](const std::vector<sf::StaticEntity*>& added) {
        props_.insert(props_.end(), added.begin(), added.end());
    };

    // Pool. Pool models are +Z up with their origin on the water surface; rotating by pi flips them
    // into NED (z down).
    const sf::Transform world_T_pool(sf::Quaternion(0.0, M_PI, 0.0));
    switch (pool_) {
        case PoolModel::Basic:
            addProps(props::AddStaticProp(*this, BasicPool(), assets_path, world_T_pool));
            break;
        case PoolModel::Comp:
            ensure_comp_looks();
            addProps(comp::setup_woollett_pool(*this, assets_path, world_T_pool));
            break;
    }

    // Scenario: props, and optionally where the robot starts (overrides the configured x, y, yaw)
    std::optional<sf::Transform> world_T_start;
    switch (scenario_) {
        case Scenario::None:
            break;
        case Scenario::Comp2026:
        case Scenario::Comp2026A:
        case Scenario::Comp2026B:
        case Scenario::Comp2026C:
        case Scenario::Comp2026D: {
            ensure_comp_looks();
            auto setup = comp::setup_2026_scenario(*this, assets_path, world_T_pool, scenario_);
            addProps(setup.entities);
            world_T_start = setup.world_T_start;
            break;
        }
    }

    if (kinematic_trajectory_path_) {
        auto trajectory_spec = trajectory::load_from_yaml(*kinematic_trajectory_path_);
        kinematic_robot_ = std::make_unique<KinematicOsprey>("os",
                                                             assets_path,
                                                             node_,
                                                             config_loader_,
                                                             trajectory_spec,
                                                             enable_cameras_);
        kinematic_robot_->add_to_simulation(this);
        if (world_T_start) {
            std::cerr << "Warning: the scenario's start pose is ignored in kinematic mode "
                         "(the trajectory sets the pose).\n";
        }
    } else {
        robot_ =
            std::make_unique<Osprey>("os", assets_path, node_, config_loader_, enable_cameras_);
        auto world_T_body_initial = config_loader_->get_initial_pose().world_T_body_initial;
        btVector3 origin = world_T_body_initial.getOrigin();
        origin.setZ(origin.z() + 2);
        world_T_body_initial.setOrigin(origin);
        if (world_T_start) {
            world_T_body_initial =
                AlignStartPose(world_T_body_initial, *world_T_start, Osprey::kFrontInBody);
        }
        AddRobot(robot_->get_stonefish_robot(), world_T_body_initial);
    }

    std::cout << "Done building scenario!\n";
}

void TauvSimulationManager::SimulationStepCompleted(sf::Scalar time_step) {
    (void)time_step;

    // Update context
    ctx_.sim_time_ = std::chrono::nanoseconds(getSimulationClock() * 1000);

    // Spin the ROS node
    executor_.spin_some();

    // Publish available sensor date and propagate control inputs into the stonefish model
    if (kinematic_robot_) {
        kinematic_robot_->on_step(ctx_);
    } else if (robot_) {
        robot_->on_step(ctx_);
    }
}
