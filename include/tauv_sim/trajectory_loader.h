/**
 * @file trajectory_loader.h
 * @brief Declares YAML trajectory loading for kinematic Osprey playback.
 */

#pragma once

#include <StonefishCommon.h>
#include <entities/animation/PWLTrajectory.h>

#include <string>
#include <vector>

namespace trajectory {

/**
 * @brief Parsed trajectory specification for Stonefish piecewise-linear playback.
 */
struct Spec {
    sf::PlaybackMode playback_mode;
    std::vector<sf::KeyPoint> keypoints;
};

/**
 * @brief Loads a trajectory specification from a YAML file.
 */
Spec load_from_yaml(const std::string& path);

}  // namespace trajectory
