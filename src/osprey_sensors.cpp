#include "tauv_sim/osprey_sensors.h"

#include <core/FeatherstoneRobot.h>
#include <core/SimulationManager.h>
#include <entities/AnimatedEntity.h>
#include <entities/FeatherstoneEntity.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

OspreySensors::OspreySensors(std::string prefix,
                             rclcpp::Node::SharedPtr node,
                             std::shared_ptr<ConfigLoader> config_loader,
                             const config::osprey::Frames& frames,
                             const sf::Transform& body_T_cad,
                             bool enable_cameras)
    : prefix_(std::move(prefix)),
      node_(std::move(node)),
      config_loader_(std::move(config_loader)),
      frames_(frames),
      body_T_cad_(body_T_cad),
      cameras_enabled_(enable_cameras) {
    const auto depth_params = config_loader_->get_depth_params();
    pressure_sensor_ = std::make_unique<sf::Pressure>("pressure_sensor", depth_params.update_rate);
    pressure_sensor_->setNoise(depth_params.noise_std);
    pressure_sensor_->setRange(200'000);
    auto pressure_pub =
        node_->create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/sensors/depth", 10);
    pressure_bridge_ = std::make_unique<PressureSensorBridge>(pressure_sensor_.get(),
                                                              pressure_pub,
                                                              prefix_ + "/depth_link");

    const auto imu_params = config_loader_->get_imu_params();
    const auto& imu_cfg = imu_params[0];
    imu_sensors_[0] = std::make_unique<sf::IMU>("imu_xsens", imu_cfg.update_rate);
    imu_sensors_[0]->setRange(imu_cfg.angular_velocity_range, imu_cfg.linear_acceleration_range);
    imu_sensors_[0]->setNoise(imu_cfg.angle_std,
                              imu_cfg.angular_velocity_std,
                              imu_cfg.yaw_angle_drift,
                              imu_cfg.linear_acceleration_std);
    auto imu_pub =
        node_->create_publisher<sensor_msgs::msg::Imu>(prefix_ + "/sensors/imu_xsens", 10);
    imu_bridges_[0] = std::make_unique<ImuBridge>(imu_sensors_[0].get(),
                                                  imu_pub,
                                                  prefix_ + "/imu_link_xsens",
                                                  imu_params[0]);

    const auto dvl_params = config_loader_->get_dvl_params();
    dvl_sensor_ = std::make_unique<sf::DVL>("dvl",
                                            50,
                                            true,
                                            dvl_params.update_rate);

    dvl_sensor_->setRange(dvl_params.linear_velocity_range, 0.01, 10);
    dvl_sensor_->setNoise(dvl_params.linear_velocity_percent_noise,
                          dvl_params.linear_velocity_stddev_noise,
                          0,
                          0,
                          0);
    auto dvl_pub =
        node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(prefix_ +
                                                                                    "/sensors/dvl",
                                                                                10);
    dvl_bridge_ =
        std::make_unique<DvlBridge>(dvl_sensor_.get(), dvl_pub, prefix_ + "/dvl_link", dvl_params);

    if (cameras_enabled_) {
        auto camera_params = config_loader_->get_fisheye_cameras();
        for (size_t i = 0; i < camera_params.size(); ++i) {
            const auto& cam_cfg = camera_params[i];
            cameras_[i] = std::make_unique<sf::FisheyeCamera>("fisheye_" + std::to_string(i),
                                                              cam_cfg.resolution[0],
                                                              cam_cfg.resolution[1],
                                                              cam_cfg.horizontal_fov_deg,
                                                              cam_cfg.update_rate);
            cameras_[i]->setExposure(cam_cfg.exposure);
            cameras_[i]->setDisplayOnScreen(cam_cfg.display_on_screen,
                                            cam_cfg.screen_offset[0],
                                            cam_cfg.screen_offset[1],
                                            static_cast<float>(cam_cfg.screen_scale));

            auto image_pub =
                node_->create_publisher<sensor_msgs::msg::Image>(prefix_ + "/sensors/cam" +
                                                                     std::to_string(i) +
                                                                     "/image_raw",
                                                                 10);
            std::string frame_id = prefix_ + "/cam" + std::to_string(i) + "_optical";
            camera_bridges_[i] =
                std::make_unique<FisheyeCameraBridge>(cameras_[i].get(), image_pub, frame_id);
            cameras_[i]->InstallNewDataHandler([this, i](sf::FisheyeCamera* cam) {
                if (camera_bridges_[i]) {
                    camera_bridges_[i]->handle_frame(cam);
                }
            });
        }
    }

    // // Make sure the tf broadcaster is only made once
    // static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
    // if (!tf_broadcaster)
    //     tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    // std::vector<geometry_msgs::msg::TransformStamped> tfs;
    // rclcpp::Time now = node_->get_clock()->now();

    // // Turn all the stonefish transforms into tf transforms
    // auto add_tf = [&](const sf::Transform& T, const std::string& child_suffix) {
    //     geometry_msgs::msg::TransformStamped t;
    //     t.header.stamp = now;
    //     t.header.frame_id = prefix_ + "/base_link_ned";
    //     t.child_frame_id = prefix_ + child_suffix;
    //     t.transform.translation.x = T.getOrigin().x();
    //     t.transform.translation.y = T.getOrigin().y();
    //     t.transform.translation.z = T.getOrigin().z();
    //     t.transform.rotation.x = T.getRotation().x();
    //     t.transform.rotation.y = T.getRotation().y();
    //     t.transform.rotation.z = T.getRotation().z();
    //     t.transform.rotation.w = T.getRotation().w();
    //     tfs.push_back(t);
    // };

    // add_tf(body_T_depth(), "/pressure_link");
    // for (size_t i = 0; i < imu_sensors_.size(); ++i) {
    //     add_tf(body_T_imu(i), "/imu" + std::to_string(i) + "_link");
    // }
    // add_tf(body_T_dvl(), "/dvl_link");

    // if (cameras_enabled_) {
    //     for (size_t i = 0; i < cameras_.size(); ++i) {
    //         add_tf(body_T_cam(i), "/cam" + std::to_string(i) + "_optical");
    //     }
    // }

    // // The body is in the NED frame, we add a transform in between to change to ENU for ROS
    // geometry_msgs::msg::TransformStamped enu_t_ned;
    // enu_t_ned.header.stamp = now;
    // enu_t_ned.header.frame_id = prefix_ + "/base_link";      // ENU is Parent
    // enu_t_ned.child_frame_id = prefix_ + "/base_link_ned";  // NED is Child
    // enu_t_ned.transform.translation.x = 0.0;
    // enu_t_ned.transform.translation.y = 0.0;
    // enu_t_ned.transform.translation.z = 0.0;
    // enu_t_ned.transform.rotation.w = 0.0;
    // enu_t_ned.transform.rotation.x = 1/std::sqrt(2);
    // enu_t_ned.transform.rotation.y = 1/std::sqrt(2);
    // enu_t_ned.transform.rotation.z = 0.0;
    // tfs.push_back(enu_t_ned);

    // tf_broadcaster->sendTransform(tfs);
}

void OspreySensors::attach_to_robot(sf::FeatherstoneRobot* robot) {
    if (!robot) {
        return;
    }

    robot->AddLinkSensor(pressure_sensor_.get(), links::OSPREY_BASE, body_T_depth());
    for (size_t i = 0; i < imu_sensors_.size(); ++i) {
        if (imu_sensors_[i]) {
            robot->AddLinkSensor(imu_sensors_[i].get(), links::OSPREY_BASE, body_T_imu(i));
        }
    }
    robot->AddLinkSensor(dvl_sensor_.get(), links::OSPREY_BASE, body_T_dvl());
    if (cameras_enabled_) {
        for (size_t i = 0; i < cameras_.size(); ++i) {
            if (cameras_[i]) {
                robot->AddVisionSensor(cameras_[i].get(), links::OSPREY_BASE, body_T_cam(i));
            }
        }
    }
}

void OspreySensors::attach_to_animated(sf::AnimatedEntity* entity,
                                       sf::SimulationManager* sim_manager) {
    if (!entity || !sim_manager) {
        return;
    }

    pressure_sensor_->AttachToSolid(entity, body_T_depth());
    for (size_t i = 0; i < imu_sensors_.size(); ++i) {
        if (imu_sensors_[i]) {
            imu_sensors_[i]->AttachToSolid(entity, body_T_imu(i));
        }
    }
    dvl_sensor_->AttachToSolid(entity, body_T_dvl());

    sim_manager->AddSensor(pressure_sensor_.get());
    for (auto& imu : imu_sensors_) {
        if (imu) {
            sim_manager->AddSensor(imu.get());
        }
    }
    sim_manager->AddSensor(dvl_sensor_.get());

    if (cameras_enabled_) {
        for (size_t i = 0; i < cameras_.size(); ++i) {
            if (!cameras_[i]) {
                continue;
            }
            cameras_[i]->AttachToSolid(entity, body_T_cam(i));
            sim_manager->AddSensor(cameras_[i].get());
        }
    }
}

void OspreySensors::on_step(const Context& ctx) {
    if (pressure_bridge_) {
        pressure_bridge_->on_step(ctx);
    }
    for (auto& imu_bridge : imu_bridges_) {
        if (imu_bridge) {
            imu_bridge->on_step(ctx);
        }
    }
    if (dvl_bridge_) {
        dvl_bridge_->on_step(ctx);
    }
    if (cameras_enabled_) {
        for (auto& bridge : camera_bridges_) {
            if (bridge) {
                bridge->on_step(ctx);
            }
        }
    }
}

sf::Transform OspreySensors::body_T_depth() const {
    return sf::Transform{sf::I3(), frames_.t_depth_B};
}

sf::Transform OspreySensors::body_T_dvl() const { return body_T_cad_ * frames_.cad_T_dvl; }

sf::Transform OspreySensors::body_T_imu(size_t idx) const {
    if (idx == 0) {
        return body_T_cad_ * frames_.cad_T_imu0;
    }
    return body_T_cad_ * frames_.cad_T_imu1;
}

sf::Transform OspreySensors::body_T_cam(size_t idx) const {
    if (idx == 0) {
        return body_T_cad_ * frames_.cad_T_cam0;
    }
    return body_T_cad_ * frames_.cad_T_cam1;
}
