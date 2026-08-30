"""
Regenerates `models/osprey.xml` from the config files.

Run:  python tools/generate_mjcf.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

from tauv_mujoco.checks import net_buoyancy_newtons, open_questions  # noqa: E402
from tauv_mujoco.model import write_mjcf  # noqa: E402
from tauv_mujoco.params import load_params  # noqa: E402


def main() -> int:
    params = load_params()
    path = write_mjcf(params=params)

    inertial = params.inertial
    buoyant_mass_freshwater = inertial.volume * 1000.0
    print(f"wrote {path}")
    print(f"  mass            {inertial.mass:.4f} kg")
    print(f"  displaced vol   {inertial.volume:.5f} m^3 "
          f"({buoyant_mass_freshwater:.3f} kg of fresh water)")
    print(f"  r_com_B         {inertial.r_com_B}")
    print(f"  r_cob_B         {inertial.r_cob_B}")
    print(f"  r_com->cob_B    {inertial.r_com_cob_B}")
    print(f"  inertia_com_B   {inertial.inertia_com_B.tolist()}")
    print(f"  net buoyancy    {net_buoyancy_newtons(params):+.2f} N (fresh water)")

    questions = open_questions(params)
    if questions:
        print(f"\nFLAGGED ASSUMPTIONS ({len(questions)}) - see TODOs in "
              "tauv_sim/config/params.yaml:")
        for i, question in enumerate(questions, 1):
            print(f"  {i}. {question}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
