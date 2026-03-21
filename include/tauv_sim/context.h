/******************************************************************************
 * @file context.h
 * @brief Defines per-step simulation context shared by tauv_sim components.
 *****************************************************************************/

/******************************************************************************
 *  TartanAUV - Carnegie Mellon University
 *
 *  Author:      root
 *  Date:        1/3/26
 *****************************************************************************/

#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

using SimTime = std::chrono::nanoseconds;

struct Context {
    Context() = default;

    SimTime sim_time_;

    /**
     * @brief Converts simulation time into an rclcpp-compatible timestamp.
     */
    rclcpp::Time get_ros_time() const;
};
