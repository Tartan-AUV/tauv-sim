/**
 * @file comp.h
 * @brief RoboSub competition environment: looks, prop specs and layouts for the models in
 *        assets/comp_common and assets/comp_2026.
 *
 * Models are +Z up with their origin on the water surface. Layout transforms here are in that
 * "pool frame"; the caller passes world_T_pool to map it into Stonefish's NED world. The setup_*
 * functions at the bottom are what TauvSimulationManager::BuildScenario calls.
 */

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "tauv_sim/registry.h"
#include "tauv_sim/scenario.h"
#include "tauv_sim/static_prop.h"

namespace comp {

namespace looks {

// Woollett competition pool (assets/comp_common/woollett). Values mirror the model's .mtl files
// (Ks 0.05, Ns 10 -> rough, non-metallic); textured looks use a white base color.
static const std::string WOOLLETT_TEX = "comp_common/woollett/textures/";

static const auto POOL_STONE = Look{"pool_stone",
                                    sf::Color::RGB(1.0F, 1.0F, 1.0F),
                                    0.9F,
                                    0.0F,
                                    0.0F,
                                    WOOLLETT_TEX + "pool_stone.png",
                                    WOOLLETT_TEX + "pool_stone_normal.png"};

static const auto POOL_TILE_BAND_5IN = Look{"pool_tile_band_5in",
                                            sf::Color::RGB(1.0F, 1.0F, 1.0F),
                                            0.9F,
                                            0.0F,
                                            0.0F,
                                            WOOLLETT_TEX + "tile_band_5in.png",
                                            WOOLLETT_TEX + "tile_band_5in_normal.png"};

static const auto POOL_TILE_DARK_1IN = Look{"pool_tile_dark_1in",
                                            sf::Color::RGB(1.0F, 1.0F, 1.0F),
                                            0.9F,
                                            0.0F,
                                            0.0F,
                                            WOOLLETT_TEX + "tile_dark_1in.png",
                                            WOOLLETT_TEX + "tile_dark_1in_normal.png"};

static const auto POOL_TILE_MEDIUM_1IN = Look{"pool_tile_medium_1in",
                                              sf::Color::RGB(1.0F, 1.0F, 1.0F),
                                              0.9F,
                                              0.0F,
                                              0.0F,
                                              WOOLLETT_TEX + "tile_medium_1in.png",
                                              WOOLLETT_TEX + "tile_medium_1in_normal.png"};

static const auto POOL_DECK_CONCRETE =
    Look{"pool_deck_concrete", sf::Color::RGB(0.72F, 0.70F, 0.66F), 0.9F, 0.0F, 0.0F};

// Lane ropes (assets/comp_common/lane_dividers): plastic float discs plus steel and tensioner parts.
// Colours mirror the models' .mtl files; every colour variant reuses these five looks.
static const auto LANE_RED = Look{"lane_red", sf::Color::RGB(0.86F, 0.07F, 0.09F), 0.5F, 0.0F, 0.05F};
static const auto LANE_WHITE =
    Look{"lane_white", sf::Color::RGB(0.95F, 0.95F, 0.95F), 0.5F, 0.0F, 0.05F};
static const auto LANE_BLUE = Look{"lane_blue", sf::Color::RGB(0.05F, 0.22F, 0.72F), 0.5F, 0.0F, 0.05F};
static const auto LANE_STEEL =
    Look{"lane_steel", sf::Color::RGB(0.62F, 0.63F, 0.65F), 0.4F, 1.0F, 0.2F};
static const auto LANE_TENSIONER =
    Look{"lane_tensioner", sf::Color::RGB(0.90F, 0.90F, 0.88F), 0.6F, 0.0F, 0.05F};

static const auto all_looks = std::array{
    POOL_STONE,
    POOL_TILE_BAND_5IN,
    POOL_TILE_DARK_1IN,
    POOL_TILE_MEDIUM_1IN,
    POOL_DECK_CONCRETE,
    LANE_RED,
    LANE_WHITE,
    LANE_BLUE,
    LANE_STEEL,
    LANE_TENSIONER,
};

}  // namespace looks

// Woollett basin, inside the walls (50 m x 25 yd); pool frame origin = centre of the water surface.
constexpr double WOOLLETT_LENGTH = 50.0;    // along X
constexpr double WOOLLETT_WIDTH = 22.86;    // along Y
constexpr int WOOLLETT_SECTIONS = 4;        // equal sections along X, split by lane dividers

/** @brief Woollett competition pool: exact triangle-mesh collision on every part. */
inline props::StaticPropSpec WoollettPool() {
    const std::string dir = "comp_common/woollett/by_material/Woollett_CompetitionPool_";
    return props::StaticPropSpec{
        "pool",
        {
            {dir + "pool_stone.obj", looks::POOL_STONE.name},
            {dir + "tile_band_5in.obj", looks::POOL_TILE_BAND_5IN.name},
            {dir + "deck_concrete.obj", looks::POOL_DECK_CONCRETE.name},
            {dir + "tile_dark_1in.obj", looks::POOL_TILE_DARK_1IN.name},
            {dir + "tile_medium_1in.obj", looks::POOL_TILE_MEDIUM_1IN.name},
        },
        props::collision::VisualMesh{},
        materials::POOL.name,
    };
}

/** @brief Tip-to-tip length of the lane rope model [m] (75 ft). */
constexpr double LANE_ROPE_LENGTH = 75 * 0.3048;

/**
 * @brief Antiwave-style 75 ft tricolor lane rope (assets/comp_common/lane_dividers), rope along X,
 *        centred on the origin, axis on the water surface.
 *
 * The visual mesh is ~64k triangles, so collision is a single capsule of the float radius instead.
 */
inline props::StaticPropSpec LaneRope(const std::string& name) {
    constexpr double kFloatRadius = 4.75 * 0.0254 / 2;  // 4.75" float discs

    const std::string R = looks::LANE_RED.name, W = looks::LANE_WHITE.name,
                      B = looks::LANE_BLUE.name;
    // Slot colours in slot order lead, sec0..sec5 (from the model's .mtl file)
    const std::array<std::string, 7> slots = {R, W, B, R, W, B, R};

    const std::string stem =
        "comp_common/lane_dividers/by_material/LaneRope_75ft_LOD1_Tricolor_Cycle_";
    props::StaticPropSpec spec{name, {}, {}, materials::PLASTIC.name};
    spec.parts.push_back({stem + "lead.obj", slots[0]});
    for (int i = 0; i < 6; ++i) {
        spec.parts.push_back({stem + "sec" + std::to_string(i) + ".obj", slots[i + 1]});
    }
    spec.parts.push_back({stem + "steel.obj", looks::LANE_STEEL.name});
    spec.parts.push_back({stem + "tensioner_body.obj", looks::LANE_TENSIONER.name});

    // Capsule axis is local Z; rotate +90 deg about Y so it runs along the rope (X).
    spec.collision = props::collision::Capsule{
        kFloatRadius,
        LANE_ROPE_LENGTH,
        sf::Transform(sf::Quaternion(sf::Vector3(0, 1, 0), M_PI / 2), sf::Vector3(0, 0, 0))};
    return spec;
}

/**
 * @brief Three lane ropes splitting the Woollett pool into four equal sections, in the pool frame.
 *
 * Each is a 75 ft (22.86 m) rope parallel to the short (25 yd) edge, spanning wall to wall along
 * Y, placed at x = -12.5, 0 and +12.5 m. The 50 m length is cut into four 12.5 m sections. The ropes
 * float on the surface.
 */
inline std::vector<props::PlacedProp> WoollettQuarterDividers() {
    constexpr int kDividers = WOOLLETT_SECTIONS - 1;
    const double section = WOOLLETT_LENGTH / (kDividers + 1);  // 12.5 m
    // Rope models run along X; rotate +90 deg about Z so they run along Y (across the pool).
    const sf::Quaternion across(sf::Vector3(0, 0, 1), M_PI / 2);

    std::vector<props::PlacedProp> dividers;
    for (int i = 0; i < kDividers; ++i) {
        const double x = -WOOLLETT_LENGTH / 2 + section * (i + 1);  // -12.5, 0, +12.5
        dividers.push_back(
            {LaneRope("lane_divider_" + std::to_string(i + 1)),
             sf::Transform(across, sf::Vector3(x, 0, 0))});
    }
    return dividers;
}

/**
 * @brief Every prop of the RoboSub 2026 scenarios, in the Woollett pool frame.
 *
 * Shared by all comp2026 variants (they differ only in the robot start); add new course props here.
 */
inline std::vector<props::PlacedProp> Comp2026Props() {
    std::vector<props::PlacedProp> placed = WoollettQuarterDividers();
    return placed;
}

// Robot starts are against the "front" long wall (y = -WOOLLETT_WIDTH / 2), facing away from it
// into the pool (+Y). Sections A..D run along +X.
// [m] front wall to robot centre: Osprey's hull reaches 0.295 m from its centre, so ~5 cm gap.
constexpr double COMP2026_START_WALL_OFFSET = 0.35;

/**
 * @brief Start frame at the centre of section `section` (0 = A ... 3 = D), next to the front wall.
 * @return pool_T_start: origin = start position on the surface, +X = heading (into the pool, +Y)
 */
inline sf::Transform Comp2026StartPose(int section) {
    const double width = WOOLLETT_LENGTH / WOOLLETT_SECTIONS;  // 12.5 m
    const double x = -WOOLLETT_LENGTH / 2 + width * (section + 0.5);  // -18.75, -6.25, 6.25, 18.75
    const double y = -WOOLLETT_WIDTH / 2 + COMP2026_START_WALL_OFFSET;
    const sf::Quaternion face_plus_y(sf::Vector3(0, 0, 1), M_PI / 2);  // start +X -> pool +Y
    return sf::Transform(face_plus_y, sf::Vector3(x, y, 0));
}

// ---------------------------------------------------------------------------------------------
// Setup entry points (implemented in comp.cpp). `assets_path` has a trailing slash; world_T_pool
// maps the pool frame (+Z up, origin on the water surface) into the simulation world.

/** @brief Creates every competition look. Call once before any other comp setup. */
void create_looks(sf::SimulationManager& sim, const std::string& assets_path);

/** @brief Adds the Woollett pool and sets competition water (no suspended particles). */
std::vector<sf::StaticEntity*> setup_woollett_pool(sf::SimulationManager& sim,
                                                   const std::string& assets_path,
                                                   const sf::Transform& world_T_pool);

/** @brief What a comp2026 scenario added, and where the robot should start (comp2026A-D). */
struct Scenario2026 {
    std::vector<sf::StaticEntity*> entities;     ///< created props (owned by Stonefish)
    std::optional<sf::Transform> world_T_start;  ///< start frame for AlignStartPose, if any
};

/**
 * @brief Adds the RoboSub 2026 course props (Comp2026Props) and picks the robot start.
 * @param scenario must be one of the comp2026 scenarios
 */
Scenario2026 setup_2026_scenario(sf::SimulationManager& sim,
                                 const std::string& assets_path,
                                 const sf::Transform& world_T_pool,
                                 Scenario scenario);

}  // namespace comp
