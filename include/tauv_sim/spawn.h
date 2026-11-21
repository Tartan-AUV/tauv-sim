/**
 * @file spawn.h
 * @brief Helpers for choosing where a robot starts.
 */

#pragma once

#include <StonefishCommon.h>

#include <cmath>

/**
 * @brief Moves a default robot pose to a start frame.
 *
 * The start frame gives the position (x, y) and, through its +X axis, the direction the robot's
 * front should face. Depth and roll/pitch are kept from `world_T_body_default`; the robot is only
 * translated horizontally and yawed about world Z so that `body_front` points along the start
 * frame's +X (both projected onto the horizontal plane).
 *
 * A plain yaw is used instead of Bullet's shortestArcQuat, which for opposite directions may pick
 * a tilted rotation axis and turn the robot upside down.
 *
 * @param world_T_body_default pose the robot would otherwise start at (e.g. from params.yaml)
 * @param world_T_start start frame: origin = start position, +X = heading
 * @param body_front the robot's front direction in its body frame
 */
inline sf::Transform AlignStartPose(const sf::Transform& world_T_body_default,
                                    const sf::Transform& world_T_start,
                                    const sf::Vector3& body_front) {
    sf::Vector3 position = world_T_start.getOrigin();
    position.setZ(world_T_body_default.getOrigin().z());

    sf::Vector3 want = world_T_start.getBasis().getColumn(0);
    sf::Vector3 front = world_T_body_default.getBasis() * body_front;
    want.setZ(0);
    front.setZ(0);
    sf::Scalar yaw = 0;
    if (want.length2() > 1e-12 && front.length2() > 1e-12) {
        want.normalize();
        front.normalize();
        yaw = std::atan2(front.cross(want).z(), front.dot(want));
    }
    const sf::Matrix3 world_R_yaw(sf::Quaternion(sf::Vector3(0, 0, 1), yaw));
    return sf::Transform(world_R_yaw * world_T_body_default.getBasis(), position);
}
