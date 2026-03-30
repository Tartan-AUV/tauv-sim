#include "tauv_sim/osprey_thrusters.h"

#undef Max  // stonefish opengl Max conflicts with ROS

#include <StonefishCommon.h>
#include <core/FeatherstoneRobot.h>
#include <entities/Entity.h>
#include <entities/SolidEntity.h>
#include <entities/solids/Polyhedron.h>
#include <fstream>
#include <sstream>
#include <cmath>

#include <Eigen/Dense>

#include "tauv_sim/config.h"
#include "tauv_sim/registry.h"
#include "tauv_sim/util.h"


using namespace config::osprey;

OspreyThrusters::OspreyThrusters(const std::string& prefix,
                                       const std::string& assets_path,
                                       rclcpp::Node::SharedPtr node,
                                       sf::FeatherstoneRobot* sf_robot,
                                       const config::osprey::actuators::Thrusters& thruster_config,
                                       const sf::Transform& body_T_cad) //TODO: change config.h to add tau and change config.cpp to add tau
                                       //%ODO also config_loader.cpp line 150
    : thruster_config_(thruster_config)
{
    // single subscriber for all the 8 forces
    //TODO
    auto forces_topic_name = prefix + "/actuators/thrusters/thruster_rpms"; // Adjust topic name as needed

    forces_sub_ = node->create_subscription<tauv_msgs::msg::ThrusterSetpoint>(
        forces_topic_name,
        10,
        // Bind the callback
        [this](const tauv_msgs::msg::ThrusterSetpoint::SharedPtr msg)
        {
            this->ThrusterCallback(msg);
        }
    );

    // physics
    auto prop_physics = sf::PhysicsSettings{};
    prop_physics.mode = sf::PhysicsMode::SUBMERGED;
    // prop_physics.estimateHydrodynamics = true;

    // Setting up the interpolated thrust
    std::vector<sf::Scalar> thrust_in;
    std::vector<sf::Scalar> thrust_out;

    // Build path to the CSV file
    std::string csv_path = assets_path + "/osprey/t200_thrust_data.csv";
    std::ifstream file(csv_path);

    // checks that the file is there
    if (!file.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open T200 thrust data CSV at: %s", csv_path.c_str());
        throw std::runtime_error("Missing thruster data file.");
    }

    std::string line;
    // Read and discard the header row so we don't try to parse words as numbers
    std::getline(file, line);

    // The way the rotor and thruster models work, each thruster requires its own rotor model,
    // but they can all share the same thrust model.

    // Parse the CSV line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> columns;

        // Split the row by commas and store each cell in the 'columns' vector
        while (std::getline(ss, token, ',')) {
            columns.push_back(token);
        }

        // Check if the row has all 3 columns to avoid out-of-bounds crashes
        if (columns.size() >= 3) {
            try {
                // Grab Column 1 (RPM) and convert to rad/s
                float rpm = std::stof(columns[0]);
                float rad_per_sec = rpm * (2.0f * M_PI / 60.0f);

                // Grab Column 5 (Force in Kg f) and convert to Newtons
                float force_kgf = std::stof(columns[1]);
                float force_n = force_kgf * 9.80665f;

                thrust_in.push_back(rad_per_sec);
                thrust_out.push_back(force_n);

            } catch (const std::invalid_argument& e) {
                RCLCPP_WARN(node->get_logger(), "Could not parse number in line: %s", line.c_str());
            }
        }
    }
    file.close();

    // Create the model using the populated and converted vectors
    auto thrust_model = std::make_shared<sf::InterpolatedThrust>(thrust_in, thrust_out);
    std::cout << thrust_model->Update(0.0f).first << " N" << std::endl;

    //builds the 8 thruster bridges
    for (size_t i = 0; i < actuators::Thrusters::N_THRUSTERS; ++i) {
        //rotor and thruster model
        auto rotor_dynamics = std::make_shared<sf::FirstOrder>(thruster_config_.tau);
        // auto rotor_dynamics = std::make_shared<sf::ZeroOrder>();

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
                                         0.076,
                                         thruster_config_.right_handed[i],
                                         370, // Max radians/second
                                         false,
                                         false);  // normalized

        auto body_T_thruster = body_T_cad * thruster_config_.cad_T_thrusters[i];
        sf_robot->AddLinkActuator(thruster, links::OSPREY_BASE, body_T_thruster);

        // Telemetry topic (still individual per thruster)
        auto telemetry_topic_name = prefix + "/actuators/thruster_" + std::to_string(i) + "/telemetry";

        auto pub = node->create_publisher<tauv_msgs::msg::EscTelemetry>(telemetry_topic_name, 10);

        thruster_bridges_[i] =
            std::make_unique<ThrusterBridge>(thruster,
                                             pub,
                                             thruster_config_.telemetry_rate,
                                             thruster_config_.esc_thruster_ids[i],
                                             thruster_config_);
        thruster_bridges_[i]->set_speed(0.0f); // Initialize thrusters to 0 speed
    }
}


void OspreyThrusters::ThrusterCallback(const tauv_msgs::msg::ThrusterSetpoint::SharedPtr msg)
{
    //TODO: add something for armed
    for (size_t i = 0; i < actuators::Thrusters::N_THRUSTERS; ++i) {
        float rpm = msg->thrust[i];
        float rad_per_sec = rpm * (2.0f * M_PI / 60.0f);
        if (thruster_bridges_[i]) {
            thruster_bridges_[i]->set_speed(rad_per_sec);
        }

    }
}

void OspreyThrusters::on_step(const Context& ctx) {
    // Pass the simulator tick down to all the active bridges
    for (auto& bridge : thruster_bridges_) {
        if (bridge) {
            bridge->on_step(ctx);
        }
    }
}
