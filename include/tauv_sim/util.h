/**
 * @file util.h
 * @brief Declares math conversion helpers shared across tauv_sim model code.
 */

#pragma once

#include <StonefishCommon.h>

#include <Eigen/Dense>

#undef Max

#include "tauv_sim/config.h"

/**
 * @brief Converts a Stonefish 3x3 matrix to Eigen format.
 */
Eigen::Matrix3d sf_to_eigen_matrix(const sf::Matrix3& m);

/**
 * @brief Converts an Eigen 3x3 matrix to Stonefish format.
 */
sf::Matrix3 eigen_to_sf_matrix(const Eigen::Matrix3d& m);

/**
 * @brief Computes principal inertial frame transform and moments from config.
 */
std::pair<sf::Transform, sf::Vector3> get_sf_inertia(const config::osprey::InertialBuoyancy& cfg,
                                                     sf::Matrix3 body_R_cad);
