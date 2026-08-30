# Osprey MuJoCo Simulator

A standalone (no ROS) MuJoCo simulator for the Osprey AUV, replacing the
Stonefish simulator in the parent `tauv_sim` package.

MuJoCo has no usable AUV hydrodynamics: its native fluid model (`<option
density>` / `<option viscosity>` plus ellipsoid drag) is far too crude. So the
fluid model is pinned **off** and a Fossen-style hydrodynamic wrench is computed
in Python and injected through `data.xfrc_applied` each step.

## Status

| Step | Description | State |
|---|---|---|
| 1 | Rigid body model (MJCF) | **done** |
| 2 | Hydrodynamics module | not started |
| 3 | Thrusters | not started |
| 4 | Sensors | not started |
| 5 | Standalone sim loop | not started |
| 6 | Validation vs Stonefish | not started |
| 7 | ROS integration | not started |

## Layout

```
config/osprey.yaml      MuJoCo-only settings (solver, meshes, gravity)
models/osprey.xml       Generated MJCF - do not hand-edit
models/assets/          Generated meshes (gitignored)
sim/tauv_mujoco/        The simulator package
tests/                  Per-step validation tests
tools/                  Asset conversion and MJCF generation
```

## Setup

The system Python on the dev box is 3.14 with no `pip` and no working
`ensurepip`, so the venv needs pip bootstrapped into it:

```bash
python3 -m venv --without-pip .venv
curl -sS https://bootstrap.pypa.io/get-pip.py | ./.venv/bin/python
./.venv/bin/pip install -r requirements.txt
```

If your Python already has pip, the usual `python3 -m venv .venv` is fine.

Generate the meshes and the MJCF. The visual mesh is gitignored, so this is
required on a fresh checkout:

```bash
./.venv/bin/python tools/prepare_assets.py
./.venv/bin/python tools/generate_mjcf.py
```

Run the tests:

```bash
./.venv/bin/python -m pytest tests/ -q
```

## Running it (dev container)

Everything here runs inside the `tauv-desktop` container, where the repo is
mounted at `/tauv-mono`. The venv is built against the container's Python 3.10,
so it will NOT work from the host (whose Python is a different version).

```bash
docker exec -it containers-tauv-desktop-1 bash
cd /tauv-mono/ros_ws/src/tauv_sim/mujoco
```

If `.venv/` is missing, or you rebuilt the container image, recreate it:

```bash
/usr/bin/python3 -m venv .venv
./.venv/bin/pip install --upgrade pip
./.venv/bin/pip install -r requirements.txt
./.venv/bin/python tools/prepare_assets.py
```

## Looking at the model

```bash
./.venv/bin/python tools/view_model.py
```

On a GNOME Wayland host there is no `~/.Xauthority`, so the container's
`DISPLAY=:0` has no cookie and GLFW fails with "Failed to open display".
`view_model.py` works around this by locating mutter's XWayland cookie in
`$XDG_RUNTIME_DIR` (which docker-compose already mounts) and exporting
`XAUTHORITY` itself. The cookie's name changes every login, so it is discovered
at runtime rather than hard-coded.

Gravity is off by default: there is no buoyancy until Step 2, so with gravity
on the vehicle simply falls forever.

```bash
./.venv/bin/python tools/view_model.py --camera chase   # side | chase | above | free
./.venv/bin/python tools/view_model.py --free-fall      # vacuum free-fall
./.venv/bin/python tools/view_model.py --spin 0.6       # torque-free tumble
```

The world is NED, so MuJoCo's free-orbit camera (which assumes z-up) renders
the vehicle upside-down. The `side`, `chase` and `above` cameras are built with
`y_cam = -z_world` and come out right-way-up; `side` is the default.

Rendering falls back to software (`llvmpipe`) on this machine, so expect a low
frame rate; the model itself is only 60k triangles.

### Interacting

Double-click the vehicle to select it, then `Ctrl`+right-drag to push and
`Ctrl`+left-drag to twist. The force is applied at the point you grabbed rather
than at the centre of mass, so an off-centre pull spins the vehicle as well as
translating it. Nothing decays, because there is no drag until Step 2.

| Key | Action |
|---|---|
| `space` | pause / resume |
| `right` | single step while paused |
| `r` | reset |
| `2` / `3` | toggle visual / collision hull |
| `tab` | full control panel |

`launch_passive` hands the stepping loop to the caller, so it has no built-in
pause or reset; both are implemented in `view_model.py` via `key_callback`.

### Applying a known wrench

Mouse dragging is imprecise. To apply an exact, repeatable body-frame wrench:

```bash
./.venv/bin/python tools/view_model.py --wrench 50,0,0,0,0,0    # 50 N surge
./.venv/bin/python tools/view_model.py --wrench 0,0,0,0,0,20    # 20 N.m yaw
```

The six numbers are `fx,fy,fz,tx,ty,tz` in the body frame, held for
`--wrench-duration` seconds (default 2). This uses the same `xfrc_applied`
injection path that the hydrodynamics and thrusters will use from Step 2, so it
doubles as a check that the path is wired correctly: 50 N on 22.3 kg gives
exactly 2.2422 m/s^2, and 20 N.m on Izz = 10 kg m^2 gives exactly 2 rad/s^2.

The scene includes a static checkered reference plane 3 m below the origin.
Both cameras use `mode="trackcom"` and follow the vehicle, so without a fixed
object in the scene even a 9.8 m/s^2 free-fall looks motionless.

## Conventions

Every frame is **NED**, per the repo-wide convention:

- `world` (W): x north, y east, z **down**. Gravity is `+z`.
- `body` (B): x forward, y right, z down. Origin at the CAD origin.
- `cad` (C): the frame the meshes and `params.yaml` are authored in. Right-Forward-Up.

The MuJoCo world frame *is* the NED world frame. Making MuJoCo z-up instead
would need a conversion on every quantity entering and leaving the
hydrodynamics module, which is exactly where sign errors are most expensive.
The only cost is cosmetic, and the MJCF ships explicit `chase` and `side`
cameras so the viewer starts right-way-up.

## Source of truth

The vehicle's physical description is **not** duplicated in this package. Mass,
displaced volume, inertia tensor, COM, COB and thruster geometry are read from
`tauv_sim/config/params.yaml`, the same file the Stonefish simulator uses, so
the two simulators cannot silently disagree. `sim/tauv_mujoco/params.py`
converts everything from the CAD frame into the body frame on load.

`models/osprey.xml` is generated from that config by `tools/generate_mjcf.py`.
It is committed so it can be read and diffed, and a test fails if it goes stale.

## Flagged assumptions

`tools/generate_mjcf.py` prints every parameter that looks questionable, and
each one has a matching `TODO(mujoco-migration)` comment in
`tauv_sim/config/params.yaml`. As of Step 1 there are four, all inherited from
the Stonefish config and all deliberately left at their configured values:

1. **Inertia tensor is `diag(10, 10, 10)` kg·m².** A uniform-density solid of
   `hull_physical.stl` at 22.3 kg gives roughly `diag(0.51, 0.51, 0.67)`. The
   configured value is ~20x larger and exactly isotropic, which no real
   asymmetric hull is. Needs the real CAD tensor.
2. **COB is 26 mm below COM**, which is passively unstable in roll and pitch.
   Osprey carries flotation high on the hull, which implies the opposite.
3. **Net buoyancy is +26.5 N (+2.7 kgf)** in fresh water, several times the
   usual AUV ballast margin.
4. **Vertical thrusters have a 0.246 m roll arm but a 0.094 m pitch arm**,
   an unusual asymmetry for a nearly square hull.

None of these block Step 1, but 1 and 2 will strongly shape the Step 2
restoring-force and Step 6 oscillation-decay results.

## Step 1 notes

The rigid body model. Key decisions:

- **Inertia is set explicitly**, not inferred from geoms. `<compiler
  inertiafromgeom="false">` plus an explicit `<inertial>` carrying `mass`,
  `pos` (the COM) and `fullinertia`. Every geom is `mass="0"`. MJCF's
  `fullinertia` takes the whole tensor, so no eigendecomposition is needed
  (unlike the Stonefish path in `util.cpp`, which had to diagonalize).
- **Visual and collision geoms are separate.** The visual geom is the
  60k-triangle decimated hull with `contype=0 conaffinity=0`; the collision geom
  is the 3.5k-triangle `hull_physical.stl`.
- **`hull_visual.stl` must be converted.** It is 65 MB *ASCII* STL, and
  MuJoCo's loader only accepts binary STL. `tools/prepare_assets.py` re-encodes
  and decimates it to 3.0 MB.
- **Sites** are placed at the COB, the COM, and all eight thrusters. Each
  thruster site's **+x axis is the direction a positive command pushes**: the
  left-handed propeller sign flip that Stonefish applies at force-application
  time (`Thruster.cpp`) is baked into the site orientation instead, so the
  thruster model in Step 3 can always push along +x.

### Things worth knowing for Step 2

Two MuJoCo behaviours were confirmed empirically here, both of which the
hydrodynamics module depends on:

- **`xfrc_applied` force acts at the body's centre of mass** (`data.xipos`),
  not the body frame origin. Buoyancy acts at the COB, so it must be applied as
  a force at the COM plus a torque `r_com→cob × F_buoyancy`.
- **A free joint's `qvel[0:3]` is the velocity of the body frame ORIGIN**, in
  world coordinates, while `qvel[3:6]` is angular velocity in the **body**
  frame. Since the COM is offset 61 mm from the origin, these differ by
  `omega × r`, and confusing them silently corrupts every drag term.
