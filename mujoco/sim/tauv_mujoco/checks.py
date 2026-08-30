"""
Derived sanity checks on the vehicle description.

These are computed from the loaded parameters rather than hard-coded, so they
keep telling the truth as the config is corrected. They are reported by
`tools/generate_mjcf.py` and asserted as warnings-not-errors, because every one
of them is a value that is *suspicious* rather than provably wrong.
"""

import numpy as np

from .params import VehicleParams

FRESH_WATER_DENSITY = 1000.0  # kg/m^3

# Rough upper bound on a sensible AUV ballast margin, used only to decide
# whether the net buoyancy is worth flagging.
TYPICAL_MAX_BALLAST_MARGIN_KG = 1.0


def net_buoyancy_newtons(params: VehicleParams, fluid_density=FRESH_WATER_DENSITY):
    """Net upward force in newtons; positive means the vehicle rises."""
    displaced_mass = params.inertial.volume * fluid_density
    return (displaced_mass - params.inertial.mass) * params.gravity


def open_questions(params: VehicleParams, fluid_density=FRESH_WATER_DENSITY):
    """Returns a list of human-readable warnings about questionable parameters."""
    inertial = params.inertial
    questions = []

    # A COB below the COM (positive body-z offset, since body z is down) gives a
    # capsizing rather than a righting moment.
    cob_offset_down = inertial.r_com_cob_B[2]
    if cob_offset_down > 0.0:
        questions.append(
            f"COB is {cob_offset_down * 1000:.0f} mm BELOW the COM (body +z is down), "
            "which is passively unstable in roll and pitch. Verify the sign of "
            "t_hull_cob_C in params.yaml."
        )

    # A uniform-density hull of this size would have a far smaller inertia; an
    # exactly isotropic tensor on an asymmetric hull is a placeholder tell.
    principal = np.linalg.eigvalsh(inertial.inertia_com_B)
    if np.allclose(principal, principal[0], rtol=1e-6):
        questions.append(
            f"Inertia tensor is exactly isotropic (diag {principal[0]:.3g} kg m^2), "
            "which no real asymmetric hull is. Likely a placeholder; a "
            "uniform-density hull_physical.stl at this mass gives ~0.5-0.7 kg m^2."
        )

    net_buoyancy = net_buoyancy_newtons(params, fluid_density)
    margin_kg = net_buoyancy / params.gravity
    if abs(margin_kg) > TYPICAL_MAX_BALLAST_MARGIN_KG:
        questions.append(
            f"Net buoyancy is {net_buoyancy:+.1f} N ({margin_kg:+.2f} kgf) at "
            f"{fluid_density:.0f} kg/m^3. Typical AUV ballast margin is under "
            f"{TYPICAL_MAX_BALLAST_MARGIN_KG:.1f} kgf. Verify mass and displaced volume."
        )

    # Compare the lever arms the vertical thrusters have in roll versus pitch.
    verticals = [t for t in params.thrusters if abs(t.thrust_axis_B[2]) > 0.9]
    if verticals:
        roll_arm = max(abs(t.r_thruster_B[1]) for t in verticals)
        pitch_arm = max(abs(t.r_thruster_B[0]) for t in verticals)
        if pitch_arm > 0.0 and roll_arm / pitch_arm > 2.0:
            questions.append(
                f"Vertical thrusters have a {roll_arm:.3f} m roll arm but only a "
                f"{pitch_arm:.3f} m pitch arm. Verify the lateral and fore-aft "
                "coordinates are not swapped in params.yaml."
            )

    return questions
