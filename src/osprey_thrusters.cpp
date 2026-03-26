#include "tauv_sim/osprey_thrusters.h"

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

ThrusterController::ThrusterController(const std::string& prefix,
                                       const std::string& assets_path,
                                       rclcpp::Node::SharedPtr node,
                                       sf::FeatherstoneRobot* sf_robot,
                                       const config::osprey::actuators::Thrusters& thruster_config,
                                       const sf::Transform& body_T_cad)
    : thruster_config_(thruster_config)
{
    // single subscriber for all the 8 forces
    //TODO
    auto forces_topic_name = prefix + "/actuators/thrusters/setpoint"; // Adjust topic name as needed

    forces_sub_ = node->create_subscription<tauv_msgs::msg::ThrusterSetpoint>(
        forces_topic_name,
        10,
        // Bind the callback
        [this](const tauv_msgs::msg::ThrusterSetpoint::SharedPtr msg) { this->callback(msg); }
    );

    // physics
    auto prop_physics = sf::PhysicsSettings{};
    prop_physics.mode = sf::PhysicsMode::DISABLED;
    prop_physics.estimateHydrodynamics = false;

    //rotor and thruster model
    auto rotor_dynamics = std::make_shared<sf::FirstOrder>(thruster_config_.kd1, thruster_config_.kd2);
    std::vector<sf::Scalar> thrust_in = {-1.0, 0.0, 1.0};

    // Define the output forces corresponding to those points
    std::vector<sf::Scalar> thrust_out = {
        static_cast<sf::Scalar>(thruster_config_.K_F_rev),
        0.0,
        static_cast<sf::Scalar>(thruster_config_.K_F_fwd)
    };

    // Create the model using the two vectors
    auto thrust_model = std::make_shared<sf::InterpolatedThrust>(thrust_in, thrust_out);

    //builds the 8 thruster bridges
    for (size_t i = 0; i < actuators::Thrusters::N_THRUSTERS; ++i) {
        auto prop = std::make_shared<sf::Polyhedron>("thruster_prop_" + std::to_string(i),
                                                     prop_physics,
                                                     assets_path + "/osprey/t200_cw_prop.obj",
                                                     1.0F,
                                                     sf::I4(),
                                                     materials::PLASTIC.name,
                                                     looks::OSPREY_BLUE_PROP.name);

        auto thruster = new sf::Thruster("thruster" + std::to_string(i),
                                         prop,
                                         rotor_dynamics,
                                         thrust_model,
                                         0.1F,
                                         thruster_config_.right_handed[i],
                                         16.0F, // Hardcoded constant voltage of 16V instead of v_bat
                                         false,
                                         true);  // normalized

        auto body_T_thruster = body_T_cad * thruster_config_.cad_T_thrusters[i];
        sf_robot->AddLinkActuator(thruster, links::OSPREY_BASE, body_T_thruster); // Fixed sf_robot typo

        // Telemetry topic (still individual per thruster)
        auto telemetry_topic_name = prefix + "/actuators/thruster_" + std::to_string(i) + "/telemetry"; // Fixed prefix typo

        auto pub = node->create_publisher<tauv_msgs::msg::EscTelemetry>(telemetry_topic_name, 10);

        // Create the bridge and store it in the array
        //make a thrustersetpoint message?


        thruster_bridges_[i] =
            std::make_unique<ThrusterBridge>(thruster,
                                             pub,
                                             thruster_config_.telemetry_rate,
                                             thruster_config_.esc_thruster_ids[i],
                                             thruster_config_);
    }
    // build the 8 thruster to their bridge
}


void ThrusterController::callback(const tauv_msgs::msg::ThrusterSetpoint::SharedPtr msg)
{
    // loop through all the forces
    //TODO: add something for armed

    for (size_t i = 0; i < actuators::Thrusters::N_THRUSTERS; ++i) {
        // force
        float force = msg->thrust[i];

        //linear interpolation
        // TODO: convert force to rpm
        float rpm = force;

        float rad_per_sec = rpm * (2.0f * 3.14159 / 60.0f);


        if (thruster_bridges_[i]) {
            thruster_bridges_[i]->set_speed(rad_per_sec);
        }

    }
}

void ThrusterController::on_step(const Context& ctx) {
    // Pass the simulator tick down to all the active bridges
    for (auto& bridge : thruster_bridges_) {
        if (bridge) {
            bridge->on_step(ctx);
        }
    }
}
