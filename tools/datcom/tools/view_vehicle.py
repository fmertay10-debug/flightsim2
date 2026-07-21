"""Interactive 3-D view of the example vehicles (matplotlib).

Usage:
    py tools/view_vehicle.py [case ...]

Cases: rocket (default), boattail, three_fin, canard — or 'all'.
Rotate with the mouse; close the window to exit.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pydatcom import (Vehicle, define_example_canard_missile,
                      define_example_rocket)
from pydatcom.geometry import plot_vehicle

CASES = {
    "rocket": define_example_rocket,
    "boattail": lambda: Vehicle(L_af=21.0, L_bt=2.0, D_bt=0.8),
    "three_fin": lambda: Vehicle(fin_num=3),
    "canard": define_example_canard_missile,
}


def main(args) -> int:
    names = args or ["rocket"]
    if names == ["all"]:
        names = list(CASES)
    unknown = [n for n in names if n not in CASES]
    if unknown:
        print(f"unknown case(s): {', '.join(unknown)}; "
              f"choose from: {', '.join(CASES)} or 'all'")
        return 2
    import matplotlib.pyplot as plt
    for n in names:
        plot_vehicle(CASES[n](), title=n)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
