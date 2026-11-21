/**
 * @file comp.cpp
 * @brief RoboSub competition setup: looks, the Woollett pool and the 2026 scenarios.
 */

#include "tauv_sim/comp.h"

#include <entities/forcefields/Ocean.h>

#include <stdexcept>

namespace comp {

namespace {

/** @brief Start section for comp2026A-D (0 = A ... 3 = D); nullopt keeps the configured pose. */
std::optional<int> StartSection(Scenario scenario) {
    switch (scenario) {
        case Scenario::Comp2026A: return 0;
        case Scenario::Comp2026B: return 1;
        case Scenario::Comp2026C: return 2;
        case Scenario::Comp2026D: return 3;
        case Scenario::Comp2026:
        case Scenario::None:
            return std::nullopt;
    }
    return std::nullopt;
}

}  // namespace

void create_looks(sf::SimulationManager& sim, const std::string& assets_path) {
    for (const auto& look : looks::all_looks) {
        create_look(sim, look, assets_path);
    }
}

std::vector<sf::StaticEntity*> setup_woollett_pool(sf::SimulationManager& sim,
                                                   const std::string& assets_path,
                                                   const sf::Transform& world_T_pool) {
    // Clear, filtered competition water: no suspended particles. Only has an effect with graphics
    // (the ocean's graphics exist once EnableOcean has run).
    if (sim.getOcean() != nullptr) {
        sim.getOcean()->setParticles(false);
    }
    return props::AddStaticProp(sim, WoollettPool(), assets_path, world_T_pool);
}

Scenario2026 setup_2026_scenario(sf::SimulationManager& sim,
                                 const std::string& assets_path,
                                 const sf::Transform& world_T_pool,
                                 Scenario scenario) {
    if (!scenario::IsComp2026(scenario)) {
        throw std::invalid_argument("comp::setup_2026_scenario called with a non-comp2026 scenario");
    }

    Scenario2026 result;
    for (const auto& placed : Comp2026Props()) {
        auto added =
            props::AddStaticProp(sim, placed.spec, assets_path, world_T_pool * placed.parent_T_prop);
        result.entities.insert(result.entities.end(), added.begin(), added.end());
    }
    if (const auto section = StartSection(scenario)) {
        result.world_T_start = world_T_pool * Comp2026StartPose(*section);
    }
    return result;
}

}  // namespace comp
