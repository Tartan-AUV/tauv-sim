#include "tauv_sim/dvl_bridge.h"

#include <Eigen/Geometry>
#include <algorithm>

namespace {

std::array<double, 9> diagonal_from_stddev(const sf::Vector3& stddev) {
    const double sx = static_cast<double>(stddev.x());
    const double sy = static_cast<double>(stddev.y());
    const double sz = static_cast<double>(stddev.z());

    return {sx * sx, 0.0, 0.0, 0.0, sy * sy, 0.0, 0.0, 0.0, sz * sz};
}

}  // namespace

DvlBridge::DvlBridge(sf::DVL* sensor,
                     rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub,
                     std::string frame_id,
                     const config::osprey::sensors::Dvl& cfg)
    : sensor_(sensor),
      frame_id_(std::move(frame_id)),
      pub_(pub),
      linear_velocity_percent_noise_(cfg.linear_velocity_percent_noise),
      linear_velocity_stddev_noise_(cfg.linear_velocity_stddev_noise) {}

void DvlBridge::on_step(const Context& ctx) {
    if (!sensor_->isNewDataAvailable()) {
        return;
    }

    if (sensor_->getLastValue(7) == 3) {  // No ping at all
        // std::cout << "DVL: No ping available, skipping publish." << std::endl;
        return;
    }

    // 1. Get raw velocities in Stonefish FRD (Forward-Right-Down) frame
    const double raw_lin_vel_x = sensor_->getLastValue(0);
    const double raw_lin_vel_y = sensor_->getLastValue(1);
    const double raw_lin_vel_z = sensor_->getLastValue(2);

    // 2. Convert to ROS standard FLU (Forward-Left-Up) frame
    const double flu_lin_vel_x = raw_lin_vel_x;
    const double flu_lin_vel_y = -raw_lin_vel_y;
    const double flu_lin_vel_z = -raw_lin_vel_z;

    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.frame_id = "dvl_link";
    msg.header.stamp = ctx.get_ros_time();

    // 3. Assign the converted velocities to the ROS message
    msg.twist.twist.linear.x = flu_lin_vel_x;
    msg.twist.twist.linear.y = flu_lin_vel_y;
    msg.twist.twist.linear.z = flu_lin_vel_z;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    msg.twist.covariance.fill(1e6);  // Large default covariance for unmeasured variables

    // 4. Calculate covariance (std::abs handles the negative signs safely)
    const auto proportional_cov = diagonal_from_stddev(sf::Vector3(
        linear_velocity_percent_noise_ * std::abs(flu_lin_vel_x),
        linear_velocity_percent_noise_ * std::abs(flu_lin_vel_y),
        linear_velocity_percent_noise_ * std::abs(flu_lin_vel_z)));
    const auto absolute_cov = diagonal_from_stddev(sf::Vector3(
        linear_velocity_stddev_noise_,
        linear_velocity_stddev_noise_,
        linear_velocity_stddev_noise_));
    std::array<double, 9> lin_vel_cov;
    for (size_t i = 0; i < 9; ++i) {
        lin_vel_cov[i] = proportional_cov[i] + absolute_cov[i];
    }

    for (size_t r = 0; r < 3; ++r) {
        std::copy(lin_vel_cov.begin() + r * 3, lin_vel_cov.begin() + r * 3 + 3, msg.twist.covariance.begin() + r * 6);
    }

    pub_->publish(msg);
}
