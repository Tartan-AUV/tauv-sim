"""
Step 1 validation: the MJCF is a faithful, force-free rigid body.

Two things are checked here, and nothing else:

1.  The compiled model's inertial properties match `params.yaml` exactly. If
    mass, COM or the inertia tensor is wrong, every later hydrodynamics result
    is wrong in a way that is very hard to attribute.

2.  In vacuum (`density=0`, `viscosity=0`, no applied wrench) the body behaves
    as an ideal rigid body: it free-falls along the analytic parabola, does not
    spontaneously rotate, and under torque-free rotation conserves angular
    momentum and kinetic energy.

The world is NED, so gravity is +z and "falling" means z increases.
"""

import dataclasses

import mujoco
import numpy as np
import pytest

from tauv_mujoco.model import BODY_NAME, MJCF_PATH, build_mjcf_string, load_model
from tauv_mujoco.params import N_THRUSTERS, load_params

# The body is the only non-world body in the model.
BODY_ID = 1


@pytest.fixture(scope="module")
def params():
    return load_params()


@pytest.fixture(scope="module")
def model(params):
    return load_model(params)


def make_data(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return data


def body_inertia_matrix(model, body_id=BODY_ID):
    """
    Reassembles the full inertia tensor in body-frame axes.

    MuJoCo stores inertia diagonalized: `body_inertia` holds the principal
    moments and `body_iquat` the body->principal rotation.
    """
    body_R_principal = np.zeros(9)
    mujoco.mju_quat2Mat(body_R_principal, model.body_iquat[body_id])
    body_R_principal = body_R_principal.reshape(3, 3)
    return body_R_principal @ np.diag(model.body_inertia[body_id]) @ body_R_principal.T


def angular_momentum_world(model, data, body_id=BODY_ID):
    """Angular momentum about the COM, expressed in the world frame."""
    omega_body = data.qvel[3:6]
    inertia_body = body_inertia_matrix(model, body_id)
    world_R_body = data.xmat[body_id].reshape(3, 3)
    return world_R_body @ (inertia_body @ omega_body)


def rotational_energy(model, data, body_id=BODY_ID):
    omega_body = data.qvel[3:6]
    return 0.5 * omega_body @ (body_inertia_matrix(model, body_id) @ omega_body)


# --- 1. Inertial properties round-trip ---------------------------------------


def test_mass_matches_config(model, params):
    assert model.body_mass[BODY_ID] == pytest.approx(params.inertial.mass, abs=1e-12)


def test_com_matches_config(model, params):
    np.testing.assert_allclose(
        model.body_ipos[BODY_ID], params.inertial.r_com_B, atol=1e-12
    )


def test_full_inertia_tensor_matches_config(model, params):
    np.testing.assert_allclose(
        body_inertia_matrix(model), params.inertial.inertia_com_B, atol=1e-9
    )


def test_geoms_contribute_no_mass(model, params):
    """
    The explicit <inertial> element must be the only source of inertia.

    If a mesh geom were ever given a density, MuJoCo would silently add its mass
    and the model would no longer match the config.
    """
    assert model.body_mass[BODY_ID] == pytest.approx(params.inertial.mass, abs=1e-12)
    assert np.all(model.geom_group[model.body_geomadr[BODY_ID] :] >= 2)


# --- 2. Model structure -------------------------------------------------------


def test_visual_and_collision_geoms_are_separate(model):
    visual = model.geom(name="hull_visual")
    collision = model.geom(name="hull_collision")

    # The visual mesh is full-resolution and must never take part in contacts.
    assert visual.contype[0] == 0 and visual.conaffinity[0] == 0
    # The collision mesh is the simplified one and is the only contact geometry.
    assert collision.contype[0] == 1 and collision.conaffinity[0] == 1
    assert visual.dataid[0] != collision.dataid[0]

    n_visual_verts = model.mesh_vertnum[visual.dataid[0]]
    n_collision_verts = model.mesh_vertnum[collision.dataid[0]]
    assert n_collision_verts < n_visual_verts


def test_cob_and_com_sites_match_config(model, params):
    np.testing.assert_allclose(
        model.site(name="cob").pos, params.inertial.r_cob_B, atol=1e-12
    )
    np.testing.assert_allclose(
        model.site(name="com").pos, params.inertial.r_com_B, atol=1e-12
    )


def test_thruster_sites_match_config(model, params):
    assert len(params.thrusters) == N_THRUSTERS

    for thruster in params.thrusters:
        site = model.site(name=f"thruster_{thruster.name}")
        np.testing.assert_allclose(site.pos, thruster.r_thruster_B, atol=1e-12)

        # The site's +x axis must be the direction a positive command pushes.
        site_R = np.zeros(9)
        mujoco.mju_quat2Mat(site_R, site.quat)
        np.testing.assert_allclose(
            site_R.reshape(3, 3)[:, 0], thruster.thrust_axis_B, atol=1e-9
        )


def test_free_joint_has_no_damping(model):
    """Any joint damping would masquerade as hydrodynamic drag in later steps."""
    np.testing.assert_allclose(model.dof_damping, 0.0, atol=0.0)
    np.testing.assert_allclose(model.dof_frictionloss, 0.0, atol=0.0)


def test_fluid_model_is_disabled(model):
    """MuJoCo's own fluid model must stay off; hydrodynamics are external."""
    assert model.opt.density == 0.0
    assert model.opt.viscosity == 0.0
    np.testing.assert_allclose(model.opt.wind, 0.0, atol=0.0)


def test_gravity_is_ned(model, params):
    np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, params.gravity], atol=1e-12)


def test_committed_mjcf_is_not_stale(params):
    """Guards against `models/osprey.xml` drifting from the config files."""
    assert MJCF_PATH.read_text() == build_mjcf_string(params), (
        "models/osprey.xml is out of date; run `python tools/generate_mjcf.py`"
    )


# --- 3. Vacuum rigid-body behaviour -------------------------------------------


def test_free_fall_follows_analytic_parabola(model, params):
    """With gravity as the only force, z(t) must be exactly z0 + g*t^2/2."""
    data = make_data(model)
    duration = 5.0
    n_steps = int(duration / model.opt.timestep)

    for _ in range(n_steps):
        mujoco.mj_step(model, data)

    t = n_steps * model.opt.timestep
    expected_z = 0.5 * params.gravity * t**2
    expected_vz = params.gravity * t

    assert data.qpos[2] == pytest.approx(expected_z, rel=1e-9)
    assert data.qvel[2] == pytest.approx(expected_vz, rel=1e-9)


def test_free_fall_has_no_lateral_drift_or_rotation(model):
    """
    A body released from rest must fall straight down and never start spinning,
    even though its COM is offset from the body origin.
    """
    data = make_data(model)
    for _ in range(2500):
        mujoco.mj_step(model, data)

    np.testing.assert_allclose(data.qpos[:2], 0.0, atol=1e-12)
    np.testing.assert_allclose(data.qvel[:2], 0.0, atol=1e-12)
    np.testing.assert_allclose(data.qvel[3:6], 0.0, atol=1e-12)
    np.testing.assert_allclose(data.qpos[3:7], [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_at_rest_in_zero_gravity_nothing_moves(model):
    model = mujoco.MjModel.from_xml_string(
        build_mjcf_string(load_params()), _mesh_assets()
    )
    model.opt.gravity[:] = 0.0
    data = make_data(model)

    for _ in range(2500):
        mujoco.mj_step(model, data)

    np.testing.assert_allclose(data.qpos[:3], 0.0, atol=1e-12)
    np.testing.assert_allclose(data.qvel, 0.0, atol=1e-12)


def test_torque_free_rotation_is_constant_for_isotropic_inertia(model):
    """
    The configured inertia is diag(10,10,10). An isotropic body has no
    gyroscopic coupling, so angular velocity must be exactly constant.
    """
    model = _zero_gravity_model()
    data = mujoco.MjData(model)

    omega_initial = np.array([0.7, -0.4, 1.1])
    _set_spin_about_com(model, data, omega_initial)
    com_initial = data.xipos[BODY_ID].copy()

    for _ in range(5000):
        mujoco.mj_step(model, data)

    np.testing.assert_allclose(data.qvel[3:6], omega_initial, rtol=1e-9, atol=1e-11)

    # With no external force the COM must not move. Note that qvel[0:3] is the
    # velocity of the BODY ORIGIN, which is 61 mm from the COM and therefore
    # legitimately non-zero while the vehicle spins.
    np.testing.assert_allclose(data.xipos[BODY_ID], com_initial, atol=1e-9)

    # The body origin's velocity must be exactly the rigid-body relation
    # v_origin = omega x (r_origin - r_com). This pins down the free-joint
    # velocity convention that the hydrodynamics module will rely on.
    world_R_body = data.xmat[BODY_ID].reshape(3, 3)
    omega_world = world_R_body @ data.qvel[3:6]
    r_com_origin_W = data.xpos[BODY_ID] - data.xipos[BODY_ID]
    np.testing.assert_allclose(
        data.qvel[:3], np.cross(omega_world, r_com_origin_W), atol=1e-9
    )


def test_torque_free_rotation_conserves_momentum_and_energy_when_anisotropic():
    """
    The isotropic test above cannot detect a wrongly-oriented inertia tensor,
    because every axis is equivalent. Repeating it with a deliberately
    anisotropic tensor forces MuJoCo to actually integrate Euler's equations,
    so angular momentum (world frame) and rotational energy must be conserved
    while body-frame angular velocity is free to wander.
    """
    params = load_params()
    anisotropic = np.diag([0.45, 0.80, 1.10])
    params = dataclasses.replace(
        params,
        inertial=dataclasses.replace(params.inertial, inertia_com_B=anisotropic),
    )

    model = mujoco.MjModel.from_xml_string(build_mjcf_string(params), _mesh_assets())
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)

    _set_spin_about_com(model, data, np.array([1.5, 0.2, 0.9]))

    momentum_initial = angular_momentum_world(model, data)
    energy_initial = rotational_energy(model, data)

    omega_history = []
    for _ in range(5000):
        mujoco.mj_step(model, data)
        omega_history.append(data.qvel[3:6].copy())

    # Tolerance is set by RK4 truncation error over 10 s of fast rotation, not
    # by anything physical; the observed drift is ~2e-6 relative.
    np.testing.assert_allclose(
        angular_momentum_world(model, data), momentum_initial, rtol=1e-5, atol=1e-8
    )
    assert rotational_energy(model, data) == pytest.approx(energy_initial, rel=1e-5)

    # Sanity check that the test is not vacuously passing: an anisotropic body
    # really must exchange angular velocity between body axes.
    omega_history = np.array(omega_history)
    assert np.ptp(omega_history[:, 0]) > 0.1


def _set_spin_about_com(model, data, omega_body):
    """
    Sets a pure spin about the COM.

    A free joint's qvel[0:3] is the velocity of the BODY ORIGIN, not the COM.
    Leaving it at zero while setting an angular velocity therefore launches the
    COM on a straight-line drift of omega x (r_com - r_origin), which is correct
    rigid-body behaviour but not the torque-free spin these tests want.
    """
    data.qvel[3:6] = omega_body
    mujoco.mj_forward(model, data)

    world_R_body = data.xmat[BODY_ID].reshape(3, 3)
    omega_world = world_R_body @ omega_body
    r_com_origin_W = data.xpos[BODY_ID] - data.xipos[BODY_ID]
    data.qvel[:3] = np.cross(omega_world, r_com_origin_W)
    mujoco.mj_forward(model, data)


def _mesh_assets():
    params = load_params()
    return {
        path.name: path.read_bytes()
        for path in (params.meshes.collision_path, params.meshes.visual_path)
    }


def _zero_gravity_model():
    model = mujoco.MjModel.from_xml_string(
        build_mjcf_string(load_params()), _mesh_assets()
    )
    model.opt.gravity[:] = 0.0
    return model
