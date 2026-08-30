"""
Opens the Osprey model in the MuJoCo viewer for visual inspection.

This is NOT the Step 5 simulation loop. There is no hydrodynamics and no
thruster model yet, so the only force acting is gravity. By default gravity is
disabled so the vehicle hovers and the geometry, sites and frames can be
inspected; pass --free-fall to watch it drop.

Run:
    python tools/view_model.py                 # hover, inspect geometry
    python tools/view_model.py --free-fall     # vacuum free-fall
    python tools/view_model.py --spin 0.6      # torque-free tumble
    python tools/view_model.py --camera chase  # side | chase | above | free
    python tools/view_model.py --wrench 50,0,0,0,0,0   # 50 N surge for 2 s
    python tools/view_model.py --wrench 0,0,0,0,0,20   # 20 N.m yaw for 2 s

Interaction:
    Double-click the vehicle to select it, then Ctrl+right-drag to push it and
    Ctrl+left-drag to twist it. The force is applied where you grabbed, not at
    the centre of mass, so an off-centre pull spins the vehicle as well as
    moving it. With no hydrodynamics yet the motion never decays.

Keys:
    space   pause / resume     (implemented here: launch_passive has no
                                built-in pause, the stepping loop is ours)
    right   single step while paused
    r       reset to the initial state
    2 / 3   toggle the visual and collision hulls
    tab     full control panel
"""

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402

from tauv_mujoco.checks import net_buoyancy_newtons  # noqa: E402
from tauv_mujoco.model import BODY_ID_HINT, load_model  # noqa: E402
from tauv_mujoco.params import load_params  # noqa: E402


def ensure_x_authority():
    """
    Points XAUTHORITY at GNOME's XWayland cookie when it is not already set.

    A GNOME Wayland session never writes ~/.Xauthority; mutter puts a per-login
    cookie in XDG_RUNTIME_DIR instead. The dev container mounts that directory
    but does not export XAUTHORITY, so GLFW cannot authenticate against :0 and
    window creation fails with "Failed to open display". The cookie's filename
    is regenerated on every login, so it has to be discovered rather than
    hard-coded.
    """
    if os.environ.get("XAUTHORITY") or not os.environ.get("DISPLAY"):
        return

    runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    cookies = sorted(
        runtime_dir.glob(".mutter-Xwaylandauth.*"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if cookies:
        os.environ["XAUTHORITY"] = str(cookies[0])
        print(f"using XWayland cookie {cookies[0]}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--free-fall",
        action="store_true",
        help="enable gravity; without hydrodynamics the vehicle just falls",
    )
    parser.add_argument(
        "--spin",
        type=float,
        default=0.0,
        help="initial angular velocity magnitude (rad/s) about a tilted axis",
    )
    parser.add_argument(
        "--wrench",
        default=None,
        metavar="FX,FY,FZ,TX,TY,TZ",
        help="constant wrench in the BODY frame (N and N.m) applied for "
             "--wrench-duration seconds, e.g. '50,0,0,0,0,0' to surge forward "
             "or '0,0,0,0,0,20' to yaw",
    )
    parser.add_argument(
        "--wrench-duration",
        type=float,
        default=2.0,
        help="seconds to apply --wrench for (default 2)",
    )
    parser.add_argument(
        "--camera",
        default="side",
        choices=("side", "chase", "above", "free"),
        help="starting camera; 'free' is MuJoCo's orbit camera, which starts "
             "upside-down because the world is z-down",
    )
    return parser.parse_args()


def set_spin_about_com(model, data, omega_body):
    """
    Sets a pure spin about the COM.

    A free joint's qvel[0:3] is the velocity of the body ORIGIN, not the COM, so
    leaving it zero would launch the COM on a straight-line drift.
    """
    data.qvel[3:6] = omega_body
    mujoco.mj_forward(model, data)

    world_R_body = data.xmat[BODY_ID_HINT].reshape(3, 3)
    r_com_origin_W = data.xpos[BODY_ID_HINT] - data.xipos[BODY_ID_HINT]
    data.qvel[:3] = np.cross(world_R_body @ omega_body, r_com_origin_W)
    mujoco.mj_forward(model, data)


class SimControl:
    """
    Pause, single-step and reset state for the viewer loop.

    launch_passive gives the stepping loop to the caller, so none of these
    exist until they are written here. The viewer only forwards key events.
    """

    KEY_SPACE = 32
    KEY_RIGHT = 262
    KEY_R = ord("R")

    def __init__(self, on_reset):
        self.paused = False
        self.step_once = False
        self._on_reset = on_reset

    def handle_key(self, keycode):
        if keycode == self.KEY_SPACE:
            self.paused = not self.paused
            print("paused" if self.paused else "running")
        elif keycode == self.KEY_RIGHT:
            self.step_once = True
        elif keycode == self.KEY_R:
            self._on_reset()
            print("reset")


def parse_wrench(text):
    """Parses a 'fx,fy,fz,tx,ty,tz' body-frame wrench into force and torque."""
    if text is None:
        return None, None
    values = np.array([float(v) for v in text.replace(" ", "").split(",")])
    if values.size != 6:
        raise ValueError(f"--wrench needs 6 comma-separated numbers, got {values.size}")
    return values[:3], values[3:]


def main() -> int:
    args = parse_args()
    ensure_x_authority()
    force_body, torque_body = parse_wrench(args.wrench)
    params = load_params()
    model = load_model(params)

    if not args.free_fall:
        # Step 2 has not happened yet, so there is no buoyancy to balance
        # gravity. Zeroing it is the only way to hold the vehicle still.
        model.opt.gravity[:] = 0.0

    data = mujoco.MjData(model)
    if args.spin:
        set_spin_about_com(model, data, args.spin * np.array([0.3, 0.2, 0.93]))
    mujoco.mj_forward(model, data)

    print(f"mass {params.inertial.mass} kg, displaced {params.inertial.volume} m^3")
    print(f"net buoyancy would be {net_buoyancy_newtons(params):+.1f} N "
          "(NOT simulated yet - no hydrodynamics until Step 2)")
    print(f"gravity {'ON - expect free-fall' if args.free_fall else 'OFF - hovering'}")
    print("world is NED: +z is DOWN, so falling means z increases")
    print(f"camera '{args.camera}'; sites and body axes are already enabled")
    print("keys: space=pause, right=step, r=reset, 2/3=visual/collision hull")

    def reset():
        mujoco.mj_resetData(model, data)
        if args.spin:
            set_spin_about_com(model, data, args.spin * np.array([0.3, 0.2, 0.93]))
        mujoco.mj_forward(model, data)

    control = SimControl(reset)

    with mujoco.viewer.launch_passive(
        model, data, key_callback=control.handle_key
    ) as viewer:
        # Show sites straight away; otherwise the whole point of opening the
        # viewer is buried in the control panel.
        viewer.opt.sitegroup[:] = 1
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY

        # The named cameras are built with y_cam = -z_world so they render
        # right-way-up despite the NED world.
        if args.camera != "free":
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera(args.camera).id

        while viewer.is_running():
            step_start = time.time()

            if control.paused and not control.step_once:
                # Dragging a paused body moves its pose directly rather than
                # pushing it, which is the easiest way to inspect an attitude.
                mujoco.mjv_applyPerturbPose(model, data, viewer.perturb, 1)
                mujoco.mj_forward(model, data)
            else:
                # xfrc_applied is the single channel for every external wrench,
                # and mjv_applyPerturbForce ADDS to it, so it must be cleared
                # each step or mouse forces would accumulate without bound.
                # From Step 2 the hydrodynamic wrench is summed into this same
                # slot.
                data.xfrc_applied[:] = 0.0
                mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)

                # A scripted body-frame wrench, rotated into the world frame
                # because xfrc_applied is global. This is exactly the injection
                # path the hydrodynamics and thrusters will use from Step 2.
                if force_body is not None and data.time < args.wrench_duration:
                    world_R_body = data.xmat[BODY_ID_HINT].reshape(3, 3)
                    data.xfrc_applied[BODY_ID_HINT, :3] += world_R_body @ force_body
                    data.xfrc_applied[BODY_ID_HINT, 3:] += world_R_body @ torque_body

                mujoco.mj_step(model, data)
                control.step_once = False

            viewer.sync()

            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
