/**
 * @file osprey_sensors.h
 * @brief Declares sensor construction, attachment, and ROS publication for Osprey.
 */

#pragma once

#include <core/FeatherstoneRobot.h>
#include <core/SimulationManager.h>
#include <entities/AnimatedEntity.h>
#include <entities/animation/PWLTrajectory.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/vision/FisheyeCamera.h>

#include <array>
#include <memory>
#include <string>

#undef Max

#include "tauv_sim/config_loader.h"
#include "tauv_sim/context.h"
#include "tauv_sim/dvl_bridge.h"
#include "tauv_sim/fisheye_camera_bridge.h"
#include "tauv_sim/imu_bridge.h"
#include "tauv_sim/pressure_sensor_bridge.h"
#include "tauv_sim/registry.h"

class OspreySensors {
   public:
    /**
     * @brief Creates and configures all enabled Osprey sensors and ROS bridges.
     */
    OspreySensors(std::string prefix,
                  rclcpp::Node::SharedPtr node,
                  std::shared_ptr<ConfigLoader> config_loader,
                  const config::osprey::Frames& frames,
                  const sf::Transform& body_T_cad,
                  bool enable_cameras);

    /**
     * @brief Attaches sensors to the Featherstone robot model.
     */
    void attach_to_robot(sf::FeatherstoneRobot* robot);

    /**
     * @brief Attaches sensors to an animated body for kinematic playback mode.
     */
    void attach_to_animated(sf::AnimatedEntity* entity, sf::SimulationManager* sim_manager);

    /**
     * @brief Publishes all available sensor outputs for the current sim step.
     */
    void on_step(const Context& ctx);

   private:
    /**
     * @brief Returns the transform from body frame to depth sensor frame.
     */
    sf::Transform body_T_depth() const;

    /**
     * @brief Returns the transform from body frame to IMU frame by index.
     */
    sf::Transform body_T_imu(size_t idx) const;

    /**
     * @brief Returns the transform from body frame to DVL frame.
     */
    sf::Transform body_T_dvl() const;

    /**
     * @brief Returns the transform from body frame to camera frame by index.
     */
    sf::Transform body_T_cam(size_t idx) const;

    std::string prefix_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ConfigLoader> config_loader_;
    config::osprey::Frames frames_;
    sf::Transform body_T_cad_;
    bool cameras_enabled_;

    std::unique_ptr<sf::Pressure> pressure_sensor_;
    std::array<std::unique_ptr<sf::IMU>, config::osprey::sensors::Imu::N_IMUS> imu_sensors_;
    std::unique_ptr<sf::DVL> dvl_sensor_;
    std::array<std::unique_ptr<sf::FisheyeCamera>, config::osprey::sensors::FisheyeCamera::N_CAMERAS>
        cameras_;

    std::unique_ptr<PressureSensorBridge> pressure_bridge_;
    std::array<std::unique_ptr<ImuBridge>, config::osprey::sensors::Imu::N_IMUS> imu_bridges_;
    std::unique_ptr<DvlBridge> dvl_bridge_;
    std::array<std::unique_ptr<FisheyeCameraBridge>, config::osprey::sensors::FisheyeCamera::N_CAMERAS>
        camera_bridges_;
};
