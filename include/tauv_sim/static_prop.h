/**
 * @file static_prop.h
 * @brief Static props built from per-material visual meshes plus an independent collision shape.
 *
 * Stonefish gives each mesh a single look and ignores .mtl files, so multi-material models are
 * exported as one OBJ per material (assets/.../by_material/). A prop is a list of those visual parts
 * and one collision description, chosen per prop:
 *
 *  - collision::VisualMesh: every visual part is also an exact triangle-mesh collider (pool basin).
 *  - collision::Mesh:       a separate proxy mesh (triangle mesh or convex hull).
 *  - collision::Box / Cylinder / Sphere / Capsule: an invisible analytic primitive (cheap and
 *    stable; e.g. a capsule around a lane rope instead of ~140k triangles).
 *  - collision::None:       visual only.
 *
 * Visual-only parts are kept out of collision detection and sensor raycasts (DVL etc.) by putting
 * their rigid bodies in collision group 0 with mask 0. Collision primitives are not rendered.
 */

#pragma once

#include <core/SimulationManager.h>
#include <entities/StaticEntity.h>

#include <string>
#include <variant>
#include <vector>

namespace props {

/** @brief One visual mesh (usually one material of a by_material export) and its look. */
struct VisualPart {
    std::string mesh;  ///< OBJ path relative to the assets directory
    std::string look;  ///< name of a look created with CreateLook
};

namespace collision {

/** @brief No collision: the prop is visual only. */
struct None {};

/** @brief Use every visual part as an exact (non-convex) triangle-mesh collider. */
struct VisualMesh {};

/** @brief A separate proxy mesh, relative to the assets directory. */
struct Mesh {
    std::string mesh;
    bool convex_hull = true;  ///< convex hull of the vertices (fast) or the raw triangle mesh
    sf::Transform origin = sf::I4();  ///< prop frame -> proxy mesh frame
};

/** @brief Box with full edge lengths `size`, centred on `origin`. */
struct Box {
    sf::Vector3 size;
    sf::Transform origin = sf::I4();
};

/** @brief Cylinder along the local Z axis of `origin`. */
struct Cylinder {
    sf::Scalar radius;
    sf::Scalar height;
    sf::Transform origin = sf::I4();
};

/** @brief Sphere centred on `origin`. */
struct Sphere {
    sf::Scalar radius;
    sf::Transform origin = sf::I4();
};

/** @brief Capsule along the local Z axis of `origin`; `length` is the tip-to-tip length. */
struct Capsule {
    sf::Scalar radius;
    sf::Scalar length;
    sf::Transform origin = sf::I4();
};

}  // namespace collision

using Collision = std::variant<collision::None,
                               collision::VisualMesh,
                               collision::Mesh,
                               collision::Box,
                               collision::Cylinder,
                               collision::Sphere,
                               collision::Capsule>;

/** @brief A static prop: visual parts in a common prop frame plus one collision description. */
struct StaticPropSpec {
    std::string name;
    std::vector<VisualPart> parts;
    Collision collision;
    std::string material;  ///< Stonefish material used for contacts
};

/** @brief A prop spec together with its pose in some parent frame (e.g. a pool or course frame). */
struct PlacedProp {
    StaticPropSpec spec;
    sf::Transform parent_T_prop;
};

/**
 * @brief Adds a prop to the simulation at `world_T_prop`.
 * @param assets_path assets directory, with a trailing slash
 * @return every entity created (owned by Stonefish), visual parts first
 */
std::vector<sf::StaticEntity*> AddStaticProp(sf::SimulationManager& sim,
                                             const StaticPropSpec& spec,
                                             const std::string& assets_path,
                                             const sf::Transform& world_T_prop);

/** @brief Invisible static capsule collider (Stonefish's Obstacle has no capsule primitive). */
class CapsuleCollider : public sf::StaticEntity {
   public:
    CapsuleCollider(const std::string& unique_name,
                    sf::Scalar radius,
                    sf::Scalar length,
                    const std::string& material);

    sf::StaticEntityType getStaticType() override;
};

}  // namespace props
