/**
 * @file config_loader.h
 * @brief Declares the ROS-parameter loader for tauv_sim configuration structs.
 */

#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "tauv_sim/config.h"

/**
 * @brief Loads typed simulation configuration from ROS parameters.
 */
class ConfigLoader {
   public:
    /**
     * @brief Constructs a loader that reads from the provided ROS node.
     */
    explicit ConfigLoader(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

    /**
     * @brief Loads the initial world pose for the robot body.
     */
    config::world::InitialPose get_initial_pose();

    /**
     * @brief Loads static CAD/body/sensor frame transforms.
     */
    config::osprey::Frames get_frames();

    /**
     * @brief Loads hull mass, volume, CoM, CoB, and inertia parameters.
     */
    config::osprey::InertialBuoyancy get_inertial_buoyancy_params();

    /**
     * @brief Loads depth sensor parameters.
     */
    config::osprey::sensors::Depth get_depth_params();

    /**
     * @brief Loads all configured IMU parameter blocks.
     */
    std::array<config::osprey::sensors::Imu, config::osprey::sensors::Imu::N_IMUS>
    get_imu_params();

    /**
     * @brief Loads DVL parameters.
     */
    config::osprey::sensors::Dvl get_dvl_params();

    /**
     * @brief Loads fisheye camera parameters for all configured cameras.
     */
    std::array<config::osprey::sensors::FisheyeCamera, config::osprey::sensors::FisheyeCamera::N_CAMERAS>
    get_fisheye_cameras();

    /**
     * @brief Loads thruster and ESC model parameters.
     */
    config::osprey::actuators::Thrusters get_thrusters();

   private:
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief Computes parameter key names used to load a transform pair.
     */
    std::pair<std::string, std::string> get_transform_name(const std::string& ns,
                                                           const std::string& to,
                                                           const std::string& from,
                                                           bool expect_euler);

    /**
     * @brief Loads a vector-valued parameter as a dynamic array.
     */
    std::vector<double> get_vector(const std::string& ns, const std::string& name);

    /**
     * @brief Loads a fixed-size array parameter and validates its element count.
     */
    template <typename T, size_t N>
    std::array<T, N> get_array(const std::string& ns, const std::string& name);

    /**
     * @brief Loads a 3x3 matrix parameter.
     */
    sf::Matrix3 get_matrix3(const std::string& ns, const std::string& name);

    /**
     * @brief Loads a 3D vector parameter.
     */
    sf::Vector3 get_vector3(const std::string& ns, const std::string& name);

    /**
     * @brief Loads a transform from rotation and translation parameter fields.
     */
    sf::Transform get_transform(const std::string& ns,
                                const std::string& to,
                                const std::string& from,
                                bool expect_euler = true);

    /**
     * @brief Loads a scalar parameter.
     */
    template <typename T>
    T get_scalar(const std::string& ns, const std::string& name);
};
