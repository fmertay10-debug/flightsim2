"""Generate the example cases in ``examples/``.

Each case directory receives:

* ``for005.dat``   the DATCOM input deck
* ``for006.dat``   the DATCOM output it produced
* ``aero.npz``     the parsed AeroData (coefficients + fill masks)
* ``<case>.html``  interactive 3-D viewer — double-click, opens in any
                   browser (self-contained, nothing to install)
* ``<case>.obj``   the same model as Wavefront .obj for CAD/3-D tools

Re-run this script after toolchain changes:  py examples/make_examples.py
For a matplotlib 3-D view:  py tools/view_vehicle.py all
"""
from pathlib import Path

from pydatcom import (Vehicle, define_example_canard_missile,
                      define_example_rocket, run_pipeline, vehicle_to_html,
                      vehicle_to_obj)

HERE = Path(__file__).resolve().parent
DELTA = [-20.0, -10.0, 0.0, 10.0, 20.0]


def _boattail_rocket() -> Vehicle:
    """Example rocket with a 2 ft boattail (total length kept at 27 ft)."""
    return Vehicle(L_af=21.0, L_bt=2.0, D_bt=0.8)


def _three_fin_rocket() -> Vehicle:
    """Example rocket with 3 fins (120 deg, one vertical) instead of 4."""
    return Vehicle(fin_num=3)


CASES = {
    # name: (vehicle, fin-deflection schedule)
    "01_rocket_baseline":    (define_example_rocket(), None),
    "02_rocket_fin_control": (define_example_rocket(), DELTA),
    "03_rocket_boattail":    (_boattail_rocket(), None),
    "04_rocket_three_fin":   (_three_fin_rocket(), None),
    "05_canard_missile":     (define_example_canard_missile(), DELTA),
}


def main() -> None:
    for name, (vehicle, delta) in CASES.items():
        outdir = HERE / name
        print(f"{name} ...", flush=True)
        aero = run_pipeline(vehicle, M=None, alpha=None, delta=delta,
                            outdir=str(outdir))
        aero.save_npz(str(outdir / "aero.npz"))
        vehicle_to_obj(vehicle, str(outdir / f"{name}.obj"))
        vehicle_to_html(vehicle, str(outdir / f"{name}.html"), title=name)
        ctrl = (f"{aero.ndelta} deflections" if aero.ndelta else "no control")
        print(f"    {aero.nalpha} alphas x {aero.nmach} Mach, {ctrl}; "
              f"{name}.obj + {name}.html written")
    print("\nAll example cases regenerated.")


if __name__ == "__main__":
    main()
