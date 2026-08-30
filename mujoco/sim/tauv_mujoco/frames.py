"""
Frame definitions and conversion helpers for the MuJoCo Osprey simulator.

Frames
------
cad   (C) : CAD frame. Right-Forward-Up, z up. Origin at the CAD origin.
body  (B) : Vehicle body frame. NED-style: x forward, y right, z down.
            Its origin is coincident with the CAD origin (t_cad__body is zero).
world (W) : World frame. NED: x north, y east, z down.

The MuJoCo world frame is configured to be `world` directly, so gravity is
applied along +z. This keeps every number in the simulator in the repo-wide NED
convention and removes a whole class of sign errors from the hydrodynamics,
which is the part of this project most likely to harbour them. The cost is
purely cosmetic: the MJCF ships explicit cameras so the viewer is right-way-up.
"""

import numpy as np
from spatialmath import SO3


def orthonormalize(R: np.ndarray) -> SO3:
    """
    Projects a nearly-orthonormal 3x3 matrix onto SO(3) and wraps it as an SO3.

    Rotations in `params.yaml` are written out by hand as flat 9-element lists,
    so they can carry small transcription errors. This mirrors the SVD
    projection that `config_loader.cpp` applies for the same reason, and lets
    SO3's validity check act as a real guard rather than a nuisance.
    """
    U, _, Vt = np.linalg.svd(np.asarray(R, dtype=float).reshape(3, 3))
    R_orthonormal = U @ Vt
    if np.linalg.det(R_orthonormal) < 0.0:
        U[:, 0] *= -1.0
        R_orthonormal = U @ Vt
    return SO3(R_orthonormal, check=True)


def rotation_from_rpy_degrees(rpy_degrees) -> SO3:
    """
    Builds a rotation from an intrinsic Z-Y-X (yaw-pitch-roll) triple in degrees.

    Matches the `Rz * Ry * Rx` composition used by `config_loader.cpp`, which is
    how the thruster orientations in `params.yaml` are meant to be read.
    """
    roll, pitch, yaw = np.radians(np.asarray(rpy_degrees, dtype=float))
    return SO3.Rz(yaw) * SO3.Ry(pitch) * SO3.Rx(roll)


def rotate_inertia(A_R_B: SO3, inertia_B: np.ndarray) -> np.ndarray:
    """
    Re-expresses an inertia tensor from frame B axes into frame A axes.

    Both tensors are taken about the same point; only the axes change.
    """
    R = A_R_B.A
    return R @ np.asarray(inertia_B, dtype=float).reshape(3, 3) @ R.T


def quaternion_wxyz(A_R_B: SO3) -> np.ndarray:
    """Returns the rotation as a (w, x, y, z) quaternion, the order MJCF uses."""
    return np.asarray(A_R_B.UnitQuaternion().A, dtype=float)
