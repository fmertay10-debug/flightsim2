"""Vehicle geometry definition for the DATCOM toolchain.

Python port of the MATLAB ``define_*`` vehicle structs. A :class:`Vehicle`
holds the same fields as the MATLAB ``vehicle`` struct; ``define_*`` helper
functions return ready-made configurations the way the MATLAB
``define_example_rocket.m`` files do.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Vehicle:
    """Launch-vehicle geometry (lengths in ``unit``, "FT" or "M")."""

    unit: str = "FT"          # length units of the fields below ("FT" or "M")

    # --- Body geometry ---
    L: float = 27.0           # total length
    D: float = 16 / 12        # body diameter
    L_nc: float = 4.0         # nose cone length
    L_af: float = 23.0        # afterbody length
    L_bt: float = 0.0         # boattail length (0 = none)
    D_bt: float = 0.0         # boattail base diameter
    nc: str = "ogive"         # nose cone type ("ogive" or "conical")
    r_n: float = 0.12         # nose tip bluntness radius

    xcg: float = 15.0         # axial cg from nose tip
    zcg: float = 0.0          # radial cg offset

    # Optional explicit outer-mould-line (filled automatically if empty)
    oml_x: List[float] = field(default_factory=list)
    oml_r: List[float] = field(default_factory=list)

    # --- Fin geometry ---
    fin_root: float = 40 / 12     # fin root chord at body
    fin_tip: float = 22 / 12      # fin tip chord
    fin_sweep: float = 40.0       # fin leading-edge sweep [deg]
    fin_height: float = 20 / 12   # fin semi-span (exposed height)
    fin_disp: float = 0.0         # fin displacement from rocket base
    fin_num: int = 4              # number of fins (3 or 4)
    fin_shape: str = "S-3-10.0-2.5-80.0"  # NACA/airfoil section string

    # --- Misc ---
    R_a: float = 0.00025          # surface roughness

    # --- Fin control-surface (flap) geometry; defaults applied if None ---
    fin_flap_c: Optional[float] = None   # flap chord / fin chord (default 0.3)
    phete: Optional[float] = None        # tan TE half angle 90-99% (default .013)
    phetep: Optional[float] = None       # tan TE half angle 95-99% (default .010)

    # --- Optional canard (forward surface, ROADMAP I6) ---
    # All four of canard_x/root/tip/height must be set together. The canard
    # is modeled as a fixed cruciform surface on the cylindrical section;
    # pitch control (delta) stays on the tail fins. Roll cases are skipped
    # for canard vehicles (DATCOM would put STYPE=4 ailerons on the canard).
    canard_x: Optional[float] = None      # root-chord LE at body, from nose tip
    canard_root: Optional[float] = None   # canard root chord at body
    canard_tip: Optional[float] = None    # canard tip chord
    canard_height: Optional[float] = None # canard semi-span (exposed height)
    canard_sweep: float = 0.0             # canard LE sweep [deg]
    canard_shape: Optional[str] = None    # airfoil string (default: fin_shape)

    @property
    def has_canard(self) -> bool:
        return self.canard_x is not None


def define_example_rocket() -> Vehicle:
    """The example finned rocket (matches MATLAB ``define_example_rocket``)."""
    return Vehicle()


def define_example_canard_missile() -> Vehicle:
    """The example rocket with a small fixed canard on the forebody."""
    return Vehicle(canard_x=5.0, canard_root=1.2, canard_tip=0.6,
                   canard_height=0.8, canard_sweep=30.0)
