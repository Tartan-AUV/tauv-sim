"""
Converts the Osprey visual hull mesh into a form MuJoCo can load.

`assets/osprey/hull_visual.stl` is a 65 MB *ASCII* STL with 315k triangles.
MuJoCo's STL loader only accepts binary STL, so the mesh is re-encoded, and
decimated if it exceeds `meshes.visual_max_faces`, into `models/assets/`.

The collision mesh (`hull_physical.stl`) is already binary and is referenced in
place, so it is not touched.

Run:  python tools/prepare_assets.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

import trimesh  # noqa: E402

from tauv_mujoco.params import load_params  # noqa: E402


def prepare_visual_mesh(source: Path, destination: Path, max_faces: int) -> None:
    mesh = trimesh.load(source, force="mesh")
    print(f"loaded {source.name}: {len(mesh.faces)} faces")

    if max_faces and len(mesh.faces) > max_faces:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
            print(f"decimated to {len(mesh.faces)} faces")
        except Exception as exc:
            # Decimation needs an optional backend. Falling back to the full-res
            # mesh is always correct, just slower to load, so it must not be fatal.
            print(f"decimation unavailable ({exc}); keeping full resolution")

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(trimesh.exchange.stl.export_stl(mesh))
    size_mb = destination.stat().st_size / 1e6
    print(f"wrote {destination} ({len(mesh.faces)} faces, {size_mb:.1f} MB, binary STL)")


def main() -> int:
    meshes = load_params().meshes

    if not meshes.collision_path.exists():
        print(f"ERROR: collision mesh not found: {meshes.collision_path}")
        return 1

    prepare_visual_mesh(
        meshes.visual_source_path, meshes.visual_path, meshes.visual_max_faces
    )
    print(f"collision mesh referenced in place: {meshes.collision_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
