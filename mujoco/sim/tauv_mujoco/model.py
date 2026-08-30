"""
MJCF generation and loading for the Osprey rigid-body model.

The MJCF is generated from `params.py` rather than hand-written so that the
inertia tensor, COM/COB offsets and eight thruster sites cannot drift away from
`tauv_sim/config/params.yaml`. The generated file is committed for readability;
`tests/test_rigid_body.py` fails if it goes stale.

The model deliberately contains NO fluid physics. `<option density>` and
`<option viscosity>` are pinned to zero, so MuJoCo applies gravity and nothing
else. Buoyancy, drag and thrust are all injected through `data.xfrc_applied`
by the hydrodynamics module.
"""

from pathlib import Path

import mujoco
import numpy as np

from .frames import quaternion_wxyz
from .params import PACKAGE_ROOT, VehicleParams, load_params

MJCF_PATH = PACKAGE_ROOT / "models" / "osprey.xml"

BODY_NAME = "osprey"
# The vehicle is the only non-world body, so it is always index 1.
BODY_ID_HINT = 1
COB_SITE_NAME = "cob"
COM_SITE_NAME = "com"


def _format_vector(v) -> str:
    return " ".join(f"{x:.9g}" for x in np.asarray(v, dtype=float).ravel())


def _thruster_site_block(params: VehicleParams) -> str:
    """
    Emits one site per thruster, oriented so the site's +x axis is the direction
    a POSITIVE command pushes.

    Stonefish stores the raw propeller frame and applies the left-handed sign
    flip at force-application time. Baking the flip into the site orientation
    instead means the thruster model can always push along +x of its own site,
    which removes a persistent source of sign confusion.
    """
    lines = []
    for thruster in params.thrusters:
        # Rebuild an orientation whose +x is the signed thrust axis. For
        # right-handed thrusters this is exactly body_R_thruster.
        body_R_site = thruster.body_R_thruster
        if not thruster.right_handed:
            # A pi rotation about the propeller frame's own z axis reverses x
            # (and y) while keeping the frame right-handed.
            body_R_site = body_R_site * _rotation_pi_about_z()

        quat = quaternion_wxyz(body_R_site)
        handedness = "right-handed" if thruster.right_handed else "left-handed"
        lines.append(
            f'      <!-- {thruster.name.upper()}: esc {thruster.esc_id}, {handedness}. '
            f"+x is the positive-command thrust direction. -->"
        )
        lines.append(
            f'      <site name="thruster_{thruster.name}" '
            f'pos="{_format_vector(thruster.r_thruster_B)}" '
            f'quat="{_format_vector(quat)}" '
            f'type="cylinder" size="0.005 0.03" rgba="0.1 0.5 0.9 0.6"/>'
        )
    return "\n".join(lines)


def _rotation_pi_about_z():
    from spatialmath import SO3

    return SO3.Rz(np.pi)


def build_mjcf_string(params: VehicleParams) -> str:
    """
    Renders the complete MJCF for the vehicle.

    Takes the fully-resolved params so tests can perturb a value (for example
    substituting an anisotropic inertia tensor) and re-render without touching
    the config files on disk.
    """
    inertial = params.inertial
    ixx, iyy, izz = np.diag(inertial.inertia_com_B)
    ixy = inertial.inertia_com_B[0, 1]
    ixz = inertial.inertia_com_B[0, 2]
    iyz = inertial.inertia_com_B[1, 2]

    # The hull meshes are authored in the CAD frame, so each mesh geom carries
    # the CAD -> body rotation. t_cad__body is zero, so no offset is needed.
    body_R_cad_quat = quaternion_wxyz(params.body_R_cad)

    return f"""<?xml version="1.0" encoding="utf-8"?>
<!--
  Osprey AUV rigid-body model.

  GENERATED FILE - do not edit by hand.
  Regenerate with:  python tools/generate_mjcf.py
  Source of truth:  tauv_sim/config/params.yaml + tauv_sim/mujoco/config/osprey.yaml

  World frame is NED (x north, y east, z DOWN), so gravity is +z.
  Body frame is NED (x forward, y right, z down), origin at the CAD origin.

  There is no fluid model here on purpose: density and viscosity are pinned to
  zero and all hydrodynamic and thruster wrenches are injected externally
  through data.xfrc_applied.
-->
<mujoco model="osprey">
  <compiler angle="radian" autolimits="true" inertiafromgeom="false" balanceinertia="false"/>

  <option timestep="{params.solver.timestep:.9g}"
          integrator="{params.solver.integrator}"
          gravity="0 0 {params.gravity:.9g}"
          density="0"
          viscosity="0"
          wind="0 0 0"/>

  <!-- Offscreen framebuffer must be sized up front; it bounds the resolution of
       both tools/view_model.py stills and the Step 4 camera sensors. -->
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <scale framelength="0.25" framewidth="0.012"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7" specular="0.2 0.2 0.2"/>
    <rgba haze="0.13 0.24 0.33 1"/>
  </visual>

  <asset>
    <mesh name="hull_collision" file="{params.meshes.collision_path.name}"
          content_type="model/stl"/>
    <mesh name="hull_visual" file="{params.meshes.visual_path.name}"
          content_type="model/stl"/>
    <texture name="pool_floor" type="2d" builtin="checker"
             rgb1="0.10 0.18 0.24" rgb2="0.15 0.26 0.33" width="512" height="512"/>
    <material name="pool_floor" texture="pool_floor" texrepeat="12 12"/>
    <texture name="water" type="skybox" builtin="gradient"
             rgb1="0.09 0.24 0.34" rgb2="0.02 0.06 0.11" width="256" height="256"/>
    <material name="hull_red" rgba="0.75 0.12 0.12 1"/>
    <material name="hull_collision_material" rgba="0.2 0.8 0.2 0.25"/>
  </asset>

  <default>
    <!-- Visual geoms carry no mass and never collide; the explicit <inertial>
         below is the sole source of the body's inertial properties. -->
    <default class="visual">
      <geom type="mesh" group="2" contype="0" conaffinity="0" mass="0"
            material="hull_red"/>
    </default>
    <default class="collision">
      <geom type="mesh" group="3" contype="1" conaffinity="1" mass="0"
            material="hull_collision_material"/>
    </default>
  </default>

  <worldbody>
    <light name="key" pos="0 0 -4" dir="0 0 1" directional="true"/>

    <!-- Cameras are given explicitly because the world is z-down; MuJoCo's
         default free camera would otherwise start underneath the scene. -->
    <!-- A MuJoCo camera looks along its own -z with +y up. In NED the world's
         "up" is -z, so every camera here uses y_cam = (0,0,-1). -->
    <camera name="chase" mode="trackcom" pos="-2 0 -0.3" xyaxes="0 1 0 0 0 -1"/>
    <camera name="side" mode="trackcom" pos="0 -2.5 -0.4" xyaxes="-1 0 0 0 0 -1"/>
    <camera name="above" mode="trackcom" pos="0 0 -2.5" xyaxes="0 1 0 1 0 0"/>

    <!-- Static visual reference. The cameras track the vehicle, so without a
         fixed object in the scene there is nothing to see it move against.
         Rotated 180 deg about x so its visible face points up in NED, and
         non-colliding because the pool is not modelled yet. -->
    <geom name="reference_floor" type="plane" pos="0 0 {params.reference_floor_depth:.9g}"
          quat="0 1 0 0" size="{params.reference_floor_size:.9g} {params.reference_floor_size:.9g} 0.1"
          material="pool_floor" group="2" contype="0" conaffinity="0"/>

    <body name="{BODY_NAME}" pos="0 0 0">
      <freejoint name="{BODY_NAME}_free"/>

      <!-- Mass, COM and full inertia tensor come straight from params.yaml,
           rotated from CAD axes into body axes. -->
      <inertial pos="{_format_vector(inertial.r_com_B)}"
                mass="{inertial.mass:.9g}"
                fullinertia="{ixx:.9g} {iyy:.9g} {izz:.9g} {ixy:.9g} {ixz:.9g} {iyz:.9g}"/>

      <geom name="hull_visual" class="visual" mesh="hull_visual"
            quat="{_format_vector(body_R_cad_quat)}"/>
      <geom name="hull_collision" class="collision" mesh="hull_collision"
            quat="{_format_vector(body_R_cad_quat)}"/>

      <!-- Centre of buoyancy: where the restoring force is applied. -->
      <site name="{COB_SITE_NAME}" pos="{_format_vector(inertial.r_cob_B)}"
            type="sphere" size="0.02" rgba="0.1 0.4 1.0 0.7"/>
      <!-- Centre of mass, for visual comparison against the COB. -->
      <site name="{COM_SITE_NAME}" pos="{_format_vector(inertial.r_com_B)}"
            type="sphere" size="0.02" rgba="1.0 0.6 0.1 0.7"/>

{_thruster_site_block(params)}
    </body>
  </worldbody>

  <!--
    No meshdir: the collision mesh lives in tauv_sim/assets/osprey/ while the
    visual mesh is generated into mujoco/models/assets/. MJCF allows only one
    meshdir, so load_model() supplies both through an in-memory asset dict.
  -->
</mujoco>
"""


def load_model(params: VehicleParams = None, mjcf: str = None) -> mujoco.MjModel:
    """
    Compiles the MJCF into an MjModel.

    The collision mesh is referenced from the Stonefish asset directory while
    the visual mesh is a generated file in `models/assets/`. MJCF has a single
    `meshdir`, so both are handed to the compiler through an in-memory asset
    dict keyed by filename.
    """
    if params is None:
        params = load_params()
    if mjcf is None:
        mjcf = build_mjcf_string(params)

    assets = {}
    for path in (params.meshes.collision_path, params.meshes.visual_path):
        if not path.exists():
            raise FileNotFoundError(
                f"Missing mesh {path}. Run `python tools/prepare_assets.py` to "
                "generate the converted visual mesh."
            )
        assets[path.name] = path.read_bytes()

    return mujoco.MjModel.from_xml_string(mjcf, assets)


def write_mjcf(path: Path = MJCF_PATH, params: VehicleParams = None) -> Path:
    """Renders the MJCF and writes it to disk, returning the path written."""
    if params is None:
        params = load_params()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_mjcf_string(params))
    return path
