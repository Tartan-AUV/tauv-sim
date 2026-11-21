/**
 * @file scenario.h
 * @brief World options selected on the command line: which pool model and which scenario to build.
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>

/** @brief Pool model (`--pool`). */
enum class PoolModel {
    Basic,  ///< simple Irvine pool (assets/pool/irvine_pool.obj)
    Comp,   ///< Woollett competition pool (assets/comp_common/woollett)
};

/** @brief Scenario (`--scenario`): props placed in the pool and, optionally, the robot start. */
enum class Scenario {
    None,       ///< empty pool, robot at the configured initial pose
    Comp2026,   ///< RoboSub 2026 props, robot at the configured initial pose
    Comp2026A,  ///< RoboSub 2026 props, robot starts in section A
    Comp2026B,  ///< ... section B
    Comp2026C,  ///< ... section C
    Comp2026D,  ///< ... section D
};

namespace scenario {

inline std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

/** @brief Parses a `--pool` value (case-insensitive). */
inline std::optional<PoolModel> ParsePoolModel(const std::string& value) {
    const std::string v = ToLower(value);
    if (v == "basic") return PoolModel::Basic;
    if (v == "comp") return PoolModel::Comp;
    return std::nullopt;
}

/** @brief Parses a `--scenario` value (case-insensitive). */
inline std::optional<Scenario> ParseScenario(const std::string& value) {
    const std::string v = ToLower(value);
    if (v == "none") return Scenario::None;
    if (v == "comp2026") return Scenario::Comp2026;
    if (v == "comp2026a") return Scenario::Comp2026A;
    if (v == "comp2026b") return Scenario::Comp2026B;
    if (v == "comp2026c") return Scenario::Comp2026C;
    if (v == "comp2026d") return Scenario::Comp2026D;
    return std::nullopt;
}

/** @brief True for every RoboSub 2026 scenario (they all share the same props). */
inline bool IsComp2026(Scenario s) {
    switch (s) {
        case Scenario::Comp2026:
        case Scenario::Comp2026A:
        case Scenario::Comp2026B:
        case Scenario::Comp2026C:
        case Scenario::Comp2026D:
            return true;
        case Scenario::None:
            return false;
    }
    return false;
}

/** @brief True when the scenario only makes sense in the given pool (comp props need Woollett). */
inline bool IsCompatible(PoolModel pool, Scenario s) {
    return !IsComp2026(s) || pool == PoolModel::Comp;
}

}  // namespace scenario
