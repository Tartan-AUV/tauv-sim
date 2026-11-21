/**
 * @file static_prop.cpp
 * @brief Implements static props with independent visual parts and collision shapes.
 */

#include "tauv_sim/static_prop.h"

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <entities/statics/Obstacle.h>
#include <graphics/OpenGLContent.h>

#include <algorithm>
#include <stdexcept>

namespace props {

CapsuleCollider::CapsuleCollider(const std::string& unique_name,
                                 sf::Scalar radius,
                                 sf::Scalar length,
                                 const std::string& material)
    : sf::StaticEntity(unique_name, material, "") {
    if (radius <= 0 || length < 2 * radius) {
        throw std::invalid_argument("CapsuleCollider '" + unique_name +
                                    "': need radius > 0 and length >= 2 * radius");
    }
    // Only drawn in Stonefish's PHYSICAL display mode; a cylinder is close enough for that.
    phyMesh = sf::OpenGLContent::BuildCylinder(static_cast<GLfloat>(radius),
                                               static_cast<GLfloat>(length));
    auto* shape = new btCapsuleShapeZ(radius, length - 2 * radius);  // height excludes the caps
    BuildRigidBody(shape);
}

sf::StaticEntityType CapsuleCollider::getStaticType() {
    return sf::StaticEntityType::OBSTACLE;
}

namespace {

/** Keeps an entity out of collision detection and raycasts (group 0 never matches any mask). */
void MakeVisualOnly(sf::SimulationManager& sim, sf::StaticEntity* ent) {
    btRigidBody* body = ent->getRigidBody();
    if (body == nullptr) {
        return;
    }
    auto* world = sim.getDynamicsWorld();
    world->removeRigidBody(body);
    world->addRigidBody(body, 0, 0);
}

/** Builds the collision entity for every collision type except None and VisualMesh. */
struct ColliderBuilder {
    const std::string& name;
    const std::string& material;
    const std::string& assets_path;

    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::Mesh& c) const {
        const std::string mesh = assets_path + c.mesh;
        return {new sf::Obstacle(name, mesh, 1.0, sf::I4(), mesh, 1.0, sf::I4(), c.convex_hull, material),
                c.origin};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::Box& c) const {
        return {new sf::Obstacle(name, c.size, sf::I4(), material), c.origin};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::Cylinder& c) const {
        return {new sf::Obstacle(name, c.radius, c.height, sf::I4(), material), c.origin};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::Sphere& c) const {
        return {new sf::Obstacle(name, c.radius, sf::I4(), material), c.origin};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::Capsule& c) const {
        return {new CapsuleCollider(name, c.radius, c.length, material), c.origin};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::None&) const {
        return {nullptr, sf::I4()};
    }
    std::pair<sf::StaticEntity*, sf::Transform> operator()(const collision::VisualMesh&) const {
        return {nullptr, sf::I4()};
    }
};

/** Lowercase file stem with separators, used to give each part a unique entity name. */
std::string PartName(const std::string& prop, const std::string& mesh) {
    std::string stem = mesh.substr(mesh.find_last_of('/') + 1);
    stem = stem.substr(0, stem.rfind('.'));
    return prop + "/" + stem;
}

}  // namespace

std::vector<sf::StaticEntity*> AddStaticProp(sf::SimulationManager& sim,
                                             const StaticPropSpec& spec,
                                             const std::string& assets_path,
                                             const sf::Transform& world_T_prop) {
    std::vector<sf::StaticEntity*> created;
    const bool parts_collide = std::holds_alternative<collision::VisualMesh>(spec.collision);

    for (const auto& part : spec.parts) {
        const std::string mesh = assets_path + part.mesh;
        // Visual-only parts still need some shape for Stonefish; a convex hull of the existing
        // vertices is cheap to build (no BVH) and is never tested since the body is filtered out.
        auto* obstacle = new sf::Obstacle(PartName(spec.name, part.mesh),
                                          mesh,
                                          1.0,
                                          sf::I4(),
                                          /*convexHull=*/!parts_collide,
                                          spec.material,
                                          part.look);
        sim.AddStaticEntity(obstacle, world_T_prop);
        if (!parts_collide) {
            MakeVisualOnly(sim, obstacle);
        }
        created.push_back(obstacle);
    }

    const auto [collider, prop_T_collider] = std::visit(
        ColliderBuilder{spec.name + "/collision", spec.material, assets_path}, spec.collision);
    if (collider != nullptr) {
        collider->setRenderable(false);
        sim.AddStaticEntity(collider, world_T_prop * prop_T_collider);
        created.push_back(collider);
    }
    return created;
}

}  // namespace props
