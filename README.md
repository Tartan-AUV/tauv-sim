# TAUV Sim

`tauv_sim` is the ROS 2 + Stonefish simulation package for TartanAUV’s Osprey vehicle. It builds
the pool world, spawns either a dynamic or kinematic vehicle model, and bridges simulated sensors
and actuators to ROS topics used by the rest of the autonomy stack.

## What The Package Does

- Builds a Stonefish scenario with pool geometry, materials, and visual looks.
- Spawns Osprey in one of two runtime modes:
  - Dynamic mode: full physics with thruster actuation.
  - Kinematic mode: pose playback from a YAML keyframe trajectory.
- Simulates and publishes vehicle sensors:
  - Depth (`nav_msgs/Odometry`)
  - IMU (`sensor_msgs/Imu`)
  - DVL (`geometry_msgs/TwistWithCovarianceStamped`)
  - Fisheye cameras (`sensor_msgs/Image`)
- Receives thruster setpoints from ROS and publishes simulated ESC telemetry.

## Package Organization

### `src/tauv_sim.cpp`
- Executable entry point.
- Parses package-specific CLI flags (`--kinematic`, `--no-cameras` / `--headless`, `--caustics[=true|false]`).
- World selection:
  - `--pool=basic|comp` (default `basic`): the Irvine pool or the Woollett competition pool.
  - `--scenario=none|comp2026|comp2026A|comp2026B|comp2026C|comp2026D` (default `none`). Every
    `comp2026*` scenario places the same course props (`comp::Comp2026Props()`, currently the lane
    dividers) and requires `--pool=comp`. `comp2026A`-`D` also start the robot in section A-D (sections
    run along +X), at the section centre with its back ~5 cm from the front long wall and its camera
    side facing into the pool
    (`comp::Comp2026StartPose`). Depth and roll/pitch come from `world_T_body_initial`. Ignored in
    kinematic mode.
  - Launch: `ros2 launch tauv_sim main.launch.py pool:=comp scenario:=comp2026A`.
- Caustics (animated sunlight focusing on underwater surfaces) are off by default; enable with `--caustics`,
  or `ros2 launch tauv_sim main.launch.py caustics:=true`. They also appear in simulated camera images.
- Creates `TauvSimulationManager` and starts the Stonefish app loop.

### `include/tauv_sim/config.h`, `config_loader.h`, `src/config_loader.cpp`
- Defines typed simulation configuration structures.
- Loads ROS parameters into those structures.
- Handles frame transforms, inertial properties, sensor settings, and thruster settings.

### `include/tauv_sim/tauv_simulation_manager.h`, `src/tauv_simulation_manager.cpp`
- Owns scenario construction and per-step integration.
- Creates the pool world and either `Osprey` (dynamic) or `KinematicOsprey` (trajectory playback).
- Spins the ROS executor and calls per-step bridge updates.

### `include/tauv_sim/osprey.h`, `src/osprey.cpp`
- Builds the dynamic Stonefish robot model.
- Applies inertial/buoyancy settings and adds thruster actuators.
- Owns sensor/actuator bridges for dynamic mode.

### `include/tauv_sim/kinematic_osprey.h`, `src/kinematic_osprey.cpp`
- Builds animated Osprey entity for trajectory playback.
- Loads and plays a piecewise-linear Stonefish trajectory.
- Reuses the same sensor bridge stack as dynamic mode.

### `include/tauv_sim/osprey_sensors.h`, `src/osprey_sensors.cpp`
- Constructs simulated sensors and associated ROS bridges.
- Supports both robot-attached (dynamic) and animated-entity-attached (kinematic) paths.
- Centralizes per-step sensor publication.

### Sensor bridge modules
- `pressure_sensor_bridge.*`: pressure sensor to depth message conversion.
- `imu_bridge.*`: IMU orientation and motion conversion into ROS conventions.
- `dvl_bridge.*`: DVL velocity publication and covariance handling.
- `fisheye_camera_bridge.*`: asynchronous camera frame transfer and image publication.

### Actuator bridge module
- `thruster_bridge.*`: maps ROS thrust commands to Stonefish thruster setpoints and emits ESC telemetry.

### Trajectory and utility modules
- `trajectory_loader.*`: parses YAML keyframe trajectories used in kinematic mode.
- `util.*`: matrix and inertia conversion helpers between Stonefish and Eigen.
- `registry.h`: shared material/look/link registries used when building the scenario. Looks may
  reference albedo/normal-map textures (paths relative to `assets/`).

### World setup: `scenario.h`, `spawn.h`, `static_prop.*`, `comp.*`
- `scenario.h`: the `PoolModel` / `Scenario` options and their CLI parsing. `BuildScenario` switches
  on each one, so a new pool or scenario is a new enum value plus a new `case`.
- `spawn.h`: `AlignStartPose` moves a default robot pose to a start frame (origin = position,
  +X = heading), keeping depth and roll/pitch. Osprey's front axis is `Osprey::kFrontInBody`.
- `comp.*` (namespace `comp`): RoboSub setup entry points called by `BuildScenario`:
  `create_looks`, `setup_woollett_pool` (pool + no suspended particles) and `setup_2026_scenario`
  (course props + optional start frame).

### Static props: `static_prop.*` and competition data: `comp.h`
- Stonefish gives each mesh one look and ignores `.mtl` files, so multi-material models are used as
  their per-material exports (`<model dir>/by_material/<model>_<material>.obj`).
- `props::AddStaticProp` adds a prop's visual parts plus one collision description, chosen per prop:
  `VisualMesh` (exact triangle mesh, e.g. the pool), `Mesh` (proxy mesh / convex hull),
  `Box`, `Cylinder`, `Sphere`, `Capsule` (invisible primitives), or `None`.
- Visual-only parts are excluded from contacts and sensor raycasts (DVL) via Bullet collision group 0.
- `static_prop.*` (namespace `props`) is generic and can build any prop.
- `comp.h` (namespace `comp`) holds the RoboSub-specific data: competition looks, `WoollettPool()`,
  `LaneRope(...)` (capsule collider), `WoollettQuarterDividers()` (three 75 ft ropes parallel to
  the short edge at x = -12.5, 0, +12.5 m, splitting the pool into four 12.5 m sections),
  `Comp2026Props()` (every comp2026 prop) and `Comp2026StartPose(section)`.

### `config/`
- `params.yaml`: runtime parameters for frames, dynamics, sensors, and actuators.
- `trajectories/*.yaml`: trajectory files for kinematic playback.

### `launch/`
- ROS launch entry points for simulation workflows.
- `desktop_sim.launch.py`: starts `tauv_sim` in default dynamic mode with `config/params.yaml`.
- `kinematic_square.launch.py`: starts `tauv_sim` in kinematic mode using `config/trajectories/osprey_square.yaml`.
- `state_estimator.launch.py`: starts kinematic simulation plus `robot_localization` EKF and optional rosbag recording for estimator debugging.

### `assets/`
- Pool and Osprey meshes/textures referenced by scenario construction.
- `comp_common/`, `comp_2026/`: RoboSub competition props (meters, +Z up, origin on the water
  surface). Multi-material models also have a `by_material/` folder with one OBJ per material.

## Runtime Modes

### Dynamic mode (default)
- Uses Stonefish rigid-body dynamics.
- Subscribes to thruster setpoint topics.
- Publishes sensor outputs and actuator telemetry.

### Kinematic mode
- Ignores physics and thruster commands.
- Drives body pose from a YAML trajectory.
- Still publishes sensors from the animated body.

Run with:

```bash
ros2 run tauv_sim tauv_sim --kinematic path/to/trajectory.yaml
```

Pass `--no-cameras` (or `--headless`) to disable fisheye camera sensor creation and publication.

### Launch Examples

```bash
# Dynamic simulation
ros2 launch tauv_sim desktop_sim.launch.py

# Kinematic trajectory playback
ros2 launch tauv_sim kinematic_square.launch.py

# Kinematic playback + EKF (and optional rosbag recording)
ros2 launch tauv_sim state_estimator.launch.py
ros2 launch tauv_sim state_estimator.launch.py record:=true
```

## Frames Of Reference

- Vehicle body frame: centered around the bottom-most point of the body.
- Vehicle inertial frame: origin at the hull/static-attachment CoM with principal inertia axes.

Trajectories are specified in a NED world frame (same origin as ENU). Example:

```yaml
playback_mode: repeat  # onetime|repeat|boomerang (optional, defaults to onetime)
keyframes:
  - t: 0.0
    position: [0.0, 0.0, 1.0]
    rpy: [0.0, 0.0, 0.0]  # degrees, roll/pitch/yaw in NED
  - t: 5.0
    position: [2.0, 0.0, 1.0]
    rpy: [0.0, 0.0, 0.0]
```
