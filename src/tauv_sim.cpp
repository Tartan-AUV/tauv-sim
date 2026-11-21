/**
 * @file tauv_sim.cpp
 * @brief Entry point for the tauv_sim executable and CLI argument parsing.
 */

#include <core/GraphicalSimulationApp.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#undef Max

#include "tauv_sim/scenario.h"
#include "tauv_sim/tauv_simulation_manager.h"

namespace {

struct ParsedArgs {
    std::vector<std::string> ros_args;
    std::optional<std::string> kinematic_trajectory_path;
    bool enable_cameras{true};
    bool enable_caustics{false};
    PoolModel pool{PoolModel::Basic};
    Scenario scenario{Scenario::None};
};

/**
 * @brief Matches `--name=value` or `--name value`; returns the value, advancing `i` for the latter.
 */
std::optional<std::string> option_value(const std::string& name, int argc, char** argv, int& i) {
    const std::string arg(argv[i]);
    const std::string flag = "--" + name;
    if (arg.rfind(flag + "=", 0) == 0) {
        return arg.substr(flag.size() + 1);
    }
    if (arg == flag) {
        if (i + 1 >= argc) {
            throw std::runtime_error(flag + " requires a value");
        }
        return std::string(argv[++i]);
    }
    return std::nullopt;
}

/**
 * @brief Parses tauv_sim custom CLI flags and forwards remaining args to ROS.
 */
ParsedArgs parse_args(int argc, char** argv) {
    ParsedArgs result;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--kinematic" || arg == "-k") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--kinematic requires a file path");
            }
            result.kinematic_trajectory_path = std::string(argv[i + 1]);
            ++i;
            continue;
        }

        if (arg == "--no-cameras" || arg == "--headless") {
            result.enable_cameras = false;
            continue;
        }

        if (arg == "--caustics") {
            result.enable_caustics = true;
            continue;
        }
        if (arg.rfind("--caustics=", 0) == 0) {
            const std::string value = arg.substr(std::string("--caustics=").size());
            if (value == "true" || value == "True" || value == "1") {
                result.enable_caustics = true;
            } else if (value == "false" || value == "False" || value == "0") {
                result.enable_caustics = false;
            } else {
                throw std::runtime_error("--caustics expects true or false, got '" + value + "'");
            }
            continue;
        }

        if (const auto value = option_value("pool", argc, argv, i)) {
            const auto pool = scenario::ParsePoolModel(*value);
            if (!pool) {
                throw std::runtime_error("--pool expects basic or comp, got '" + *value + "'");
            }
            result.pool = *pool;
            continue;
        }
        if (const auto value = option_value("scenario", argc, argv, i)) {
            const auto parsed_scenario = scenario::ParseScenario(*value);
            if (!parsed_scenario) {
                throw std::runtime_error(
                    "--scenario expects none, comp2026, comp2026A, comp2026B, comp2026C or "
                    "comp2026D, got '" + *value + "'");
            }
            result.scenario = *parsed_scenario;
            continue;
        }

        result.ros_args.emplace_back(arg);
    }

    if (!scenario::IsCompatible(result.pool, result.scenario)) {
        throw std::runtime_error("comp2026 scenarios are laid out for the competition pool; use --pool=comp");
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    // Parse tauv_sim flags before initializing ROS to avoid unknown ROS arguments.
    ParsedArgs parsed;
    try {
        parsed = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Argument error: %s\n", e.what());
        return 1;
    }

    // Can't just use init(argc, argv) because we don't want to pass our custom args to ROS (e.g. --kinematic)
    std::vector<char*> ros_argv;
    ros_argv.reserve(parsed.ros_args.size() + 1);
    ros_argv.push_back(argv[0]);
    for (auto& arg : parsed.ros_args) {
        ros_argv.push_back(const_cast<char*>(arg.c_str()));
    }
    int ros_argc = static_cast<int>(ros_argv.size());
    rclcpp::init(ros_argc, ros_argv.data());

    const auto share_path = ament_index_cpp::get_package_share_directory("tauv_sim");
    const std::string assets_path = (std::filesystem::path(share_path) / "assets").string();

    sf::RenderSettings s;
    s.caustics = parsed.enable_caustics;
    sf::HelperSettings h;
    h.showActuators = true;
    h.showSensors = true;
    TauvSimulationManager manager(assets_path,
                                  100.0f,
                                  parsed.kinematic_trajectory_path,
                                  parsed.enable_cameras,
                                  parsed.pool,
                                  parsed.scenario);

    sf::GraphicalSimulationApp app("TAUV Simulator", assets_path, s, h, &manager);
    app.Run();

    return 0;
}
