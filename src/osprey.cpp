/**
 * @file osprey.cpp
 * @brief Implements dynamic Osprey model construction, sensors, and thruster wiring.
 */

#include "tauv_sim/osprey.h"

#undef Max  // stonefish opengl Max conflicts with ROS

#include <StonefishCommon.h>
#include <core/FeatherstoneRobot.h>
#include <entities/Entity.h>
#include <entities/SolidEntity.h>
#include <entities/solids/Polyhedron.h>

#include <Eigen/Dense>

#include "tauv_sim/config.h"
#include "tauv_sim/registry.h"
#include "tauv_sim/util.h"


using namespace config::osprey;

Osprey::Osprey(const std::string prefix,
               const std::string& assets_path,
               rclcpp::Node::SharedPtr node,
               std::shared_ptr<ConfigLoader> config_loader,
               bool enable_cameras)
    : prefix_(prefix) {
    sf_robot_ = new sf::FeatherstoneRobot("osprey");

    auto frames = config_loader->get_frames();
    auto body_T_cad = frames.cad_T_body.inverse();

    auto inertial_buoyancy_params = config_loader->get_inertial_buoyancy_params();

    sf::PhysicsSettings base_physics;
    // Use the low-res physics mesh to estimate drag and added mass
    base_physics.estimateHydrodynamics = false;
    // Instead of relying the mesh for CoB and Volume, use values from config
    base_physics.useCustomVolume = true;
    base_physics.useCustomCB = true;

    auto t_hull_cob_B = body_T_cad * inertial_buoyancy_params.t_hull_cob_C;
    base_physics.customCB = t_hull_cob_B;

    base_physics.customVolume = inertial_buoyancy_params.volume;

    base_link_ = new sf::Polyhedron(links::OSPREY_BASE,
                                    base_physics,
                                    assets_path + "osprey/hull_visual.stl",
                                    1.0F,
                                    body_T_cad,
                                    assets_path + "osprey/hull_physical.stl",
                                    1.0F,
                                    body_T_cad,
                                    materials::ALUMINUM.name,
                                    looks::OSPREY_RED_HULL.name);

    auto body_R_cad = sf::Matrix3{frames.cad_T_body.getRotation().inverse()};
    auto [body_T_CG, I_CG] = get_sf_inertia(inertial_buoyancy_params, body_R_cad);

    base_link_->SetArbitraryPhysicalProperties(inertial_buoyancy_params.mass, I_CG, body_T_CG);

    sf_robot_->DefineLinks(base_link_);
    sf_robot_->BuildKinematicStructure();

    sensors_ = std::make_unique<OspreySensors>(prefix_,
                                               node,
                                               config_loader,
                                               frames,
                                               body_T_cad,
                                               enable_cameras);



    sensors_->attach_to_robot(sf_robot_); //TODO: same thing but with thrusters

    /* Actuators */
    /** Thrusters **/
    thruster_config_ = config_loader->get_thrusters();

    // make the osprey thrusters
    thruster_controller_ = std::make_unique<OspreyThrusters>(
        prefix_,
        assets_path,
        node,
        sf_robot_,
        thruster_config_,
        body_T_cad
    );

    // TODO: should be using Bessa model, but we don't have rotor inertia rn
    // auto rotor_dynamics = std::make_shared<sf::Bessa>(thruster_config_.J_msp,
    //                                                   thruster_config_.K_v1,
    //                                                   thruster_config_.K_v2,
    //                                                   thruster_config_.K_t,
    //                                                   thruster_config_.R_m);



}

/**
 * @brief Runs all sensor and thruster bridge updates for the current step.
 */
void Osprey::on_step(const Context& ctx) {
    sensors_->on_step(ctx);

    if (thruster_controller_) {
        thruster_controller_->on_step(ctx);
    }
}

sf::FeatherstoneRobot* Osprey::get_stonefish_robot() { return sf_robot_; }
