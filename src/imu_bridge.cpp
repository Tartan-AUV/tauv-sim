#include "tauv_sim/imu_bridge.h"

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

ImuBridge::ImuBridge(sf::IMU* sensor,
                     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
                     std::string frame_id,
                     const config::osprey::sensors::Imu& cfg)
    : sensor_(sensor),
      frame_id_(std::move(frame_id)),
      pub_(pub),
      orientation_covariance_(diagonal_from_stddev(cfg.angle_std)),
      angular_velocity_covariance_(diagonal_from_stddev(cfg.angular_velocity_std)),
      linear_acceleration_covariance_(diagonal_from_stddev(cfg.linear_acceleration_std)) {}

void ImuBridge::on_step(const Context& ctx) {
    if (!sensor_->isNewDataAvailable()) {
        return;
    }

    const double roll = sensor_->getLastValue(0);
    const double pitch = sensor_->getLastValue(1);
    const double yaw = sensor_->getLastValue(2);

    const double ang_vel_x = sensor_->getLastValue(3);
    const double ang_vel_y = sensor_->getLastValue(4);
    const double ang_vel_z = sensor_->getLastValue(5);

    const double lin_accel_x = sensor_->getLastValue(6);
    const double lin_accel_y = sensor_->getLastValue(7);
    const double lin_accel_z = sensor_->getLastValue(8);

    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    // Okay, so since the CAD is in ENU, but Stonefish uses NED, we rotate the robot by
    // (1/sqrt(2), 1/sqrt(2), 0, 0) quaternion to convert the frame. The IMU has the same
    // orientation as the body, so its output is modified by the same (1/sqrt(2), 1/sqrt(2), 0, 0)
    // rotation. We need to undo this to get the correct ouputs of the IMU relative to the robot.
    // After that, we have to change everything from the NED frame to the ENU frame for ROS.
    Eigen::Quaterniond orientation_offset(0, -1/std::sqrt(2), -1/std::sqrt(2), 0); // Conjugate of the (1/sqrt(2), 1/sqrt(2), 0, 0) rotation

    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ctx.get_ros_time();

    Eigen::Quaterniond orientation_raw = yaw_angle * pitch_angle * roll_angle;
    Eigen::Quaternion orientation_ned = orientation_offset * orientation_raw;
    Eigen::Quaterniond enu_T_ned(0, 1/std::sqrt(2), 1/std::sqrt(2), 0);
    Eigen::Quaterniond orientation_enu = enu_T_ned * orientation_ned;
    msg.orientation.w = orientation_ned.w();
    msg.orientation.x = orientation_ned.x();
    msg.orientation.y = orientation_ned.y();
    msg.orientation.z = orientation_ned.z();
    std::copy(orientation_covariance_.begin(),
              orientation_covariance_.end(),
              msg.orientation_covariance.begin());

    Eigen::Vector3d ang_vel_raw(ang_vel_x, ang_vel_y, ang_vel_z);
    Eigen::Vector3d ang_vel = orientation_offset * ang_vel_raw;
    msg.angular_velocity.x = ang_vel.y();
    msg.angular_velocity.y = ang_vel.x();
    msg.angular_velocity.z = -ang_vel.z();
    std::copy(angular_velocity_covariance_.begin(),
              angular_velocity_covariance_.end(),
              msg.angular_velocity_covariance.begin());

    Eigen::Vector3d lin_accel_raw(lin_accel_x, lin_accel_y, lin_accel_z);
    Eigen::Vector3d lin_accel = orientation_offset * lin_accel_raw;
    msg.linear_acceleration.x = lin_accel.y();
    msg.linear_acceleration.y = lin_accel.x();
    msg.linear_acceleration.z = -lin_accel.z();
    std::copy(linear_acceleration_covariance_.begin(),
              linear_acceleration_covariance_.end(),
              msg.linear_acceleration_covariance.begin());

    pub_->publish(msg);
}
