/******************************************************************************
 * @file context.cpp
 * @brief Implements conversion helpers for simulation-time context.
 *****************************************************************************/

/******************************************************************************
 *  TartanAUV - Carnegie Mellon University
 *
 *  Author:      root
 *  Date:        1/4/26
 *****************************************************************************/

#include "tauv_sim/context.h"

/**
 * @brief Converts nanosecond simulation time to `rclcpp::Time`.
 */
rclcpp::Time Context::get_ros_time() const { return rclcpp::Time(sim_time_.count()); }
