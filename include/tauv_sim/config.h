/**
 * @file config.h
 * @brief Defines the typed runtime configuration schema consumed by tauv_sim.
 */

#pragma once

#include <StonefishCommon.h>

#include <array>
#include <cstdint>
#include <string_view>

namespace config {

namespace world {

/**
 * @brief Initial world pose for the simulated vehicle body.
 */
struct InitialPose {
    static constexpr std::string_view NS = "world_T_body_initial";

    sf::Transform world_T_body_initial;
};

}  // namespace world

namespace osprey {

/**
 * @brief Static frame transforms used to attach sensors and actuators to Osprey.
 */
struct Frames {
    static constexpr std::string_view NS = "osprey.frames";

    // Transform from the CAD frame into the body frame
    sf::Transform cad_T_body;

    // Depth sensor in the body frame
    // TODO: express in CAD frame
    sf::Vector3 t_depth_B;

    // Transform from the DVL frame into the CAD frame
    sf::Transform cad_T_dvl;

    // Transform from the IMU frame into the CAD frame
    sf::Transform cad_T_imu0;
    sf::Transform cad_T_imu1;

    // Transforms from the camera frames into the CAD frame
    sf::Transform cad_T_cam0;
    sf::Transform cad_T_cam1;
};

/**
 * @brief Mass, volume, and rigid-body inertial parameters for the Osprey hull.
 */
struct InertialBuoyancy {
    static constexpr std::string_view NS = "osprey.inertial_buoyancy";

    // Mass of the hull in kg
    double mass;

    // Volume of the hull in m^3
    double volume;

    // Center of mass of the hull expressed in the CAD frame
    sf::Vector3 t_hull_com_C;

    // Inertia tensor of the hull at the center of mass with axes aligned with the CAD frame
    sf::Matrix3 hull_inertia_COM_C;

    // Center of buoyancy of the hull expressed in the CAD frame
    sf::Vector3 t_hull_cob_C;
};

namespace sensors {

/**
 * @brief Depth sensor noise and update-rate configuration.
 */
struct Depth {
    static constexpr std::string_view NS = "osprey.sensors.depth";

    double noise_std;
    double update_rate;
};

/**
 * @brief IMU noise, range, and update-rate configuration.
 */
struct Imu {
    static constexpr std::string_view NS = "osprey.sensors.imus";
    static constexpr size_t N_IMUS = 2;

    double update_rate;
    sf::Vector3 angle_std;
    sf::Vector3 angular_velocity_std;
    double yaw_angle_drift;
    sf::Vector3 linear_acceleration_std;
    sf::Vector3 angular_velocity_range;
    sf::Vector3 linear_acceleration_range;
};

/**
 * @brief DVL noise, velocity-range, and update-rate configuration.
 */
struct Dvl {
    static constexpr std::string_view NS = "osprey.sensors.dvl";

    double update_rate;
    double linear_velocity_percent_noise;
    double linear_velocity_stddev_noise;
    sf::Vector3 linear_velocity_range;
};

/**
 * @brief Fisheye camera rendering and publication configuration.
 */
struct FisheyeCamera {
    static constexpr std::string_view NS = "osprey.sensors.cameras";
    static constexpr size_t N_CAMERAS = 2;

    double update_rate;
    double horizontal_fov_deg;
    double exposure;
    std::array<uint32_t, 2> resolution;
    bool display_on_screen;
    std::array<uint32_t, 2> screen_offset;
    double screen_scale;
};

}  // namespace sensors

namespace actuators {

/**
 * Thruster model parameters.
 * Corresponds to the Bessa rotor dynamics model and the deadband thrust model in Stonefish.
 *
 * Bessa model: J_msp * Omega_dot + K_v1 * Omega + K_v2 * Omega * |Omega| = K_t / R_m * V_effective
 * Omega:           Angular velocity
 * J_msp:           Rotor inertia (effective)
 * K_v1, K_v2, K_t:      Torque coefficients
 * R_m:             Winding resistance
 * V_effective:     Effective motor voltage. ESC input * V_bat seems to work well enough.
 *
 * Deadband model:
 *     { K_F_fwd * Omega^2,     if Omega > deadband_max
 * F = { 0                      if deadband_min < Omega < deadband_max
 *     { -K_F_rev * Omega^2     if Omega < deadband_min
 *
 * Note: Stonefish is weird in that deadband is in the thrust model rather than rotor dynamics
 * model. This is physically incorrect. Therefore, we use deadband 0, and just don't issue
 * ESC commands for RPMs within the deadband.
 *
 * Note: All constants are using rad / s for angular velocity.
 */
struct Thrusters {
    static constexpr std::string_view NS = "osprey.actuators.thrusters";

    static constexpr size_t N_THRUSTERS = 8;

    // Time constant for rotor first order model
    double tau;

    // Right-handedness
    std::array<bool, N_THRUSTERS> right_handed;

    // ESC thruster IDs
    std::array<uint8_t, N_THRUSTERS> esc_thruster_ids;

    // Frames
    std::array<sf::Transform, N_THRUSTERS> cad_T_thrusters;

    // Telemetry rate [Hz]
    double telemetry_rate;
};

}  // namespace actuators

}  // namespace osprey

}  // namespace config
