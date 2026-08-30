"""
Vehicle and simulator parameter loading.

The vehicle's physical description is read from the existing ROS parameter file
`tauv_sim/config/params.yaml` rather than being copied into this package, so the
Stonefish and MuJoCo simulators share one source of truth for the vehicle.
MuJoCo-only settings come from `tauv_sim/mujoco/config/osprey.yaml`.

Everything this module hands out is already expressed in the body frame (B),
NED. The CAD-frame values in `params.yaml` are converted on load so that no
downstream code has to think about the CAD frame again.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from spatialmath import SO3

from .frames import orthonormalize, rotate_inertia, rotation_from_rpy_degrees

# Package layout anchors.
PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TAUV_SIM_ROOT = PACKAGE_ROOT.parent
ROS_PARAMS_PATH = TAUV_SIM_ROOT / "config" / "params.yaml"
MUJOCO_CONFIG_PATH = PACKAGE_ROOT / "config" / "osprey.yaml"
STONEFISH_ASSET_DIR = TAUV_SIM_ROOT / "assets" / "osprey"
GENERATED_ASSET_DIR = PACKAGE_ROOT / "models" / "assets"

N_THRUSTERS = 8

# Names follow the repo's Front/Aft, Left/Right, Vertical/Horizontal scheme, in
# the thruster index order used by `params.yaml` and the ESC mapping.
THRUSTER_NAMES = ("frh", "flh", "brh", "blh", "frv", "flv", "brv", "blv")


@dataclass(frozen=True)
class InertialParams:
    """Rigid-body and buoyancy description of the hull, in the body frame."""

    mass: float               # kg
    volume: float             # m^3 of displaced fluid
    r_com_B: np.ndarray       # (3,) centre of mass in body frame
    r_cob_B: np.ndarray       # (3,) centre of buoyancy in body frame
    inertia_com_B: np.ndarray  # (3,3) inertia about the COM, body axes

    @property
    def r_com_cob_B(self) -> np.ndarray:
        """Translation from the COM to the COB, expressed in the body frame."""
        return self.r_cob_B - self.r_com_B


@dataclass(frozen=True)
class ThrusterParams:
    """Geometry and handedness of a single T200 thruster."""

    index: int
    name: str
    esc_id: int
    right_handed: bool
    r_thruster_B: np.ndarray   # (3,) thruster origin in body frame
    body_R_thruster: SO3       # thruster frame orientation in body frame

    @property
    def thrust_axis_B(self) -> np.ndarray:
        """
        Unit vector along which a POSITIVE command pushes the vehicle.

        Stonefish generates thrust along the thruster frame's +x axis and then
        negates it for left-handed propellers (`Thruster.cpp`), because a
        counter-rotating prop reverses its thrust for the same shaft direction.
        Folding that sign into the axis here keeps the convention in one place.
        """
        axis_B = self.body_R_thruster.A @ np.array([1.0, 0.0, 0.0])
        return axis_B if self.right_handed else -axis_B


@dataclass(frozen=True)
class SolverParams:
    """MuJoCo integration settings."""

    timestep: float
    integrator: str


@dataclass(frozen=True)
class MeshParams:
    """Resolved absolute paths to the collision and visual hull meshes."""

    collision_path: Path
    visual_path: Path
    visual_source_path: Path
    visual_max_faces: int


@dataclass(frozen=True)
class VehicleParams:
    """Everything needed to emit the MJCF and run the simulator."""

    inertial: InertialParams
    thrusters: tuple
    solver: SolverParams
    meshes: MeshParams
    gravity: float
    reference_floor_depth: float
    reference_floor_size: float
    cad_R_body: SO3

    @property
    def body_R_cad(self) -> SO3:
        return self.cad_R_body.inv()


def _read_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _osprey_section(ros_params: dict) -> dict:
    """Digs the Osprey block out of the ROS parameter file's nested layout."""
    return ros_params["/tauv_sim"]["ros__parameters"]["osprey"]


def load_params(
    ros_params_path: Path = ROS_PARAMS_PATH,
    mujoco_config_path: Path = MUJOCO_CONFIG_PATH,
) -> VehicleParams:
    """
    Loads the vehicle description and MuJoCo settings into body-frame values.

    Raises FileNotFoundError if either config file is missing, and KeyError with
    the offending key if a required field is absent, so a bad config fails at
    load time rather than producing silently wrong physics.
    """
    osprey = _osprey_section(_read_yaml(ros_params_path))
    mj_config = _read_yaml(mujoco_config_path)

    cad_R_body = orthonormalize(osprey["frames"]["cad_R_body"])
    body_R_cad = cad_R_body.inv()

    inertial = _load_inertial(osprey["inertial_buoyancy"], body_R_cad)
    thrusters = _load_thrusters(osprey["actuators"]["thrusters"], body_R_cad)
    meshes = _load_meshes(mj_config["meshes"])

    solver = SolverParams(
        timestep=float(mj_config["solver"]["timestep"]),
        integrator=str(mj_config["solver"]["integrator"]),
    )

    return VehicleParams(
        inertial=inertial,
        thrusters=thrusters,
        solver=solver,
        meshes=meshes,
        gravity=float(mj_config["world"]["gravity"]),
        reference_floor_depth=float(mj_config["world"]["reference_floor_depth"]),
        reference_floor_size=float(mj_config["world"]["reference_floor_size"]),
        cad_R_body=cad_R_body,
    )


def _load_inertial(config: dict, body_R_cad: SO3) -> InertialParams:
    """Converts the CAD-frame inertial block into body-frame quantities."""
    r_com_C = np.asarray(config["t_hull_com_C"], dtype=float)
    r_cob_C = np.asarray(config["t_hull_cob_C"], dtype=float)
    inertia_com_C = np.asarray(config["hull_inertia_COM_C"], dtype=float).reshape(3, 3)

    # Symmetrize before rotating: the config writes all nine entries by hand, so
    # a tensor can arrive very slightly asymmetric.
    inertia_com_C = 0.5 * (inertia_com_C + inertia_com_C.T)

    return InertialParams(
        mass=float(config["mass"]),
        volume=float(config["volume"]),
        r_com_B=body_R_cad.A @ r_com_C,
        r_cob_B=body_R_cad.A @ r_cob_C,
        inertia_com_B=rotate_inertia(body_R_cad, inertia_com_C),
    )


def _load_thrusters(config: dict, body_R_cad: SO3) -> tuple:
    """Converts the eight CAD-frame thruster poses into body-frame poses."""
    right_handed = [bool(v) for v in config["right_handed"]]
    esc_ids = [int(v) for v in config["esc_thruster_ids"]]

    thrusters = []
    for i in range(N_THRUSTERS):
        r_thruster_C = np.asarray(config[f"t_cad__thruster_{i}"], dtype=float)
        cad_R_thruster = rotation_from_rpy_degrees(config[f"rpy_cad__thruster_{i}"])

        thrusters.append(
            ThrusterParams(
                index=i,
                name=THRUSTER_NAMES[i],
                esc_id=esc_ids[i],
                right_handed=right_handed[i],
                r_thruster_B=body_R_cad.A @ r_thruster_C,
                body_R_thruster=body_R_cad * cad_R_thruster,
            )
        )
    return tuple(thrusters)


def _load_meshes(config: dict) -> MeshParams:
    return MeshParams(
        collision_path=STONEFISH_ASSET_DIR / config["collision"],
        visual_path=GENERATED_ASSET_DIR / config["visual"],
        visual_source_path=STONEFISH_ASSET_DIR / config["visual_source"],
        visual_max_faces=int(config["visual_max_faces"]),
    )
