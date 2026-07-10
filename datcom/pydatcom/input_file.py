"""Write a USAF Digital DATCOM input deck (``for005.dat``).

Rebuilt on :mod:`pydatcom.deck` (ROADMAP I3 / D4): geometry mapping and
case-chaining logic live here; card formatting rules live in ``deck.py``.
Validated against the legacy (MATLAB-port) writer by running both decks
through DATCOM.exe and comparing the parsed aero tables numerically.
"""
from __future__ import annotations

import math
import warnings
from typing import List, Optional, Sequence

from .deck import Deck, Namelist
from .vehicle import Vehicle

# Default Mach schedule (17 = DATCOM cap). The deck sets STMACH=0.99 /
# TSMACH=1.01, so 0.8/0.95 run on extended subsonic methods and 1.05/1.2 on
# supersonic ones — the 0.6-1.4 band is covered but carries no transonic
# physics (no drag rise); treat those columns as low-confidence (D10).
DEFAULT_MACH = [.01, .2, .4, .6, .8, .95, 1.05, 1.2, 1.4, 1.8, 2.2, 2.6, 3.0, 3.5, 4.0, 4.5, 5.0]
DEFAULT_ALPHA = ([round(0.5 * i, 1) for i in range(0, 17)]   # 0:.5:8
                 + list(range(9, 20))                         # 9:19
                 + list(range(20, 181, 5)))                   # 20:5:180

# DATCOM limits (Users Manual, Figure 3 / Section 3.4)
MAX_MACH = 17        # NMACH per case
MAX_DELTA = 9        # NDELTA per case
ALPHA_BATCH = 17     # alphas per chained case (program cap is 20; 17 kept
                     # from the validated toolchain)

# Nose-shape parameter (Users Manual, Figure 6): 1.0 = cone, 2.0 = ogive.
# The legacy toolchain used 1.0 for every nose type; fixed per ROADMAP D6.
_BNOSE = {"conical": 1.0, "ogive": 2.0}


def _tand(deg: float) -> float:
    return math.tan(math.radians(deg))


def _panel_geometry(root: float, tip: float, sweep: float, height: float,
                    r: float):
    """Extend an exposed fin panel to the body centerline.

    Returns ``(l_d, l_s, l_e, l_b)``: LE setback at the body and at the
    tip, TE extension, and the centerline root chord DATCOM needs for
    CHRDR (validated toolchain formulas).
    """
    l_d = r * _tand(sweep)
    l_s = (r + height) * _tand(sweep)
    l_e = r * (tip + l_s - l_d - root) / height
    l_b = root + l_d - l_e
    return l_d, l_s, l_e, l_b


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def _validate(vehicle: Vehicle, M: List[float], alpha: List[float],
              delta: List[float], fin_flap_c: float,
              warn_transonic: bool = True) -> None:
    """Input checks; messages kept from the validated toolchain."""
    if len(M) > MAX_MACH:
        raise ValueError("Cannot generate and run an input file with more "
                         "than 17 Mach number entries.")
    if min(M) < 0:
        raise ValueError("Mach number cannot be negative.")
    if min(alpha) < 0:
        raise ValueError("Program will automatically flip alpha, keep all "
                         "alpha greater than zero.")
    if vehicle.zcg > vehicle.D / 2:
        raise ValueError("Center of gravity offset is greater than the "
                         "radius of the rocket.")
    if vehicle.xcg > vehicle.L:
        raise ValueError("Center of gravity is longer than the length of "
                         "the rocket.")
    if len(delta) > MAX_DELTA:
        raise ValueError("DATCOM cannot accept more than 9 control deflection "
                         "entries (NDELTA <= 9).")
    if delta and fin_flap_c >= 1:
        raise ValueError("Fin flap chord fraction (fin_flap_c) must be < 1.")
    if vehicle.nc not in _BNOSE:
        raise ValueError(f"Unknown nose cone type {vehicle.nc!r}; "
                         f"expected one of {sorted(_BNOSE)}.")
    canard_fields = (vehicle.canard_x, vehicle.canard_root,
                     vehicle.canard_tip, vehicle.canard_height)
    if any(f is not None for f in canard_fields):
        if any(f is None for f in canard_fields):
            raise ValueError("Canard requires canard_x, canard_root, "
                             "canard_tip and canard_height together.")
        if vehicle.canard_x < vehicle.L_nc:
            raise ValueError("Canard must sit on the cylindrical section "
                             "(canard_x >= L_nc).")
        tail_le = vehicle.L - vehicle.fin_disp - vehicle.fin_root
        if vehicle.canard_x + vehicle.canard_root >= tail_le:
            raise ValueError("Canard overlaps the tail fins (canard TE must "
                             "be forward of the tail root LE).")
    if delta and vehicle.has_canard:
        warnings.warn("Canard vehicle: differential-roll (ASYFLP) cases are "
                      "skipped - DATCOM's STYPE=4 ailerons attach to the "
                      "forward surface (the canard), not the tail. clroll "
                      "and cn_asy will be absent from the result.")
    if warn_transonic and any(0.6 < m < 1.4 for m in M):
        warnings.warn("Mach numbers between .6 and 1.4 (transonic) run on "
                      "STMACH/TSMACH-extended sub/supersonic methods: no "
                      "transonic physics (drag rise missed). Treat those "
                      "results as low-confidence.")


# ---------------------------------------------------------------------------
# Deck construction
# ---------------------------------------------------------------------------
def build_deck(vehicle: Vehicle,
               M: Optional[Sequence[float]] = None,
               alpha: Optional[Sequence[float]] = None,
               delta: Optional[Sequence[float]] = None,
               alt: float = 1000.0) -> str:
    """Build the DATCOM input deck and return it as a string.

    ``delta`` (symmetric fin deflection schedule, degrees, max 9 entries)
    enables the fin-control cases ($SYMFLP pitch + $ASYFLP roll). ``alt``
    is the flight altitude in the deck's length unit; DATCOM uses it for
    the Reynolds number.
    """
    user_mach = M is not None
    M = list(DEFAULT_MACH) if M is None else [0.01 if m == 0 else m for m in M]
    alpha = list(DEFAULT_ALPHA if alpha is None else alpha)
    delta = list(delta) if delta is not None else []

    v = vehicle
    fin_flap_c = v.fin_flap_c if v.fin_flap_c is not None else 0.3
    phete = v.phete if v.phete is not None else 0.013
    phetep = v.phetep if v.phetep is not None else 0.010

    # the curated default schedule includes transonic-band points on
    # purpose (D10) — only warn when the user supplied their own list
    _validate(v, M, alpha, delta, fin_flap_c, warn_transonic=user_mach)

    # --- Body outer mould line (default: nose + cylinder [+ boattail]) ---
    oml_x = list(v.oml_x)
    oml_r = list(v.oml_r)
    if not oml_x:
        oml_x = [0.0, v.L_nc, v.L_nc + v.L_af]
        oml_r = [0.0, v.D / 2, v.D / 2]
        if v.L_bt:
            oml_x.append(v.L_nc + v.L_af + v.L_bt)
            oml_r.append(v.D_bt / 2)

    # --- Derived fin geometry ---
    r = v.D / 2
    l_d, l_s, l_e, l_b = _panel_geometry(v.fin_root, v.fin_tip, v.fin_sweep,
                                         v.fin_height, r)
    fin_le_x = (v.L - v.fin_root - l_d) - v.fin_disp            # fin apex station
    if v.has_canard:
        c_shape = v.canard_shape or v.fin_shape
        c_l_d, _, _, c_l_b = _panel_geometry(v.canard_root, v.canard_tip,
                                             v.canard_sweep, v.canard_height, r)
        canard_le_x = v.canard_x - c_l_d                        # canard apex station

    deck = Deck()
    deck.control("CASEID PUB_ROCKET")

    # --- FLTCON: flight conditions ---
    n_alpha1 = min(len(alpha), ALPHA_BATCH)
    deck.card(Namelist("FLTCON")
              .num("NMACH", len(M), "%.1f")
              .arr("MACH", M))
    deck.card(Namelist("FLTCON")
              .num("NALPHA", n_alpha1, "%.1f")
              .arr("ALSCHD", alpha[:ALPHA_BATCH]))
    deck.card(Namelist("FLTCON")
              .num("NALT", 1, "%.1f")
              .arr("ALT", [alt]))
    deck.card(Namelist("FLTCON")
              .num("STMACH", 0.99)
              .num("TSMACH", 1.01)
              .num("LOOP", 2.0, "%.1f"))

    # --- SYNTHS: component placement ---
    # Forward lifting surface is always the "wing", aft surface the
    # "horizontal tail" (manual 2.4.5): canard -> XW + tail fins -> XH,
    # or tail fins alone -> XW. Vertical panels stay on the tail fins.
    synths = (Namelist("SYNTHS")
              .num("XCG", v.xcg, "%.1f")
              .num("ZCG", v.zcg, "%.1f"))
    if v.has_canard:
        synths.num("XW", canard_le_x, nl=True).num("ZW", 0.0).num("ALIW", 0.0)
        synths.num("XH", fin_le_x, nl=True).num("ZH", 0.0).num("ALIH", 0.0)
    else:
        synths.num("XW", fin_le_x, nl=True).num("ZW", 0.0).num("ALIW", 0.0)
    synths.num("XV", fin_le_x, nl=True).num("ZV", 0.0)
    if v.fin_num == 3:
        dhdadi = -30.0
    else:  # 4 fins: ventral fin mirrors the bottom panel
        dhdadi = 0.0
        synths.num("XVF", (v.L - (l_b - l_s + l_e)) - v.fin_disp, nl=True)
        synths.num("ZVF", -(v.fin_height + v.D / 2))
    synths.lit("VERTUP", ".TRUE.", nl=True)
    deck.card(synths)

    # --- OPTINS: reference quantities ---
    deck.card(Namelist("OPTINS")
              .num("SREF", math.pi * r ** 2, "%.3f")
              .num("CBARR", v.L, "%.1f")
              .num("BLREF", v.fin_height, "%.1f")
              .num("ROUGFC", v.R_a, "%.6f"))

    # --- BODY geometry ---
    body = (Namelist("BODY")
            .num("NX", len(oml_x), "%.1f")
            .num("BNOSE", _BNOSE[v.nc], "%.1f")
            .num("BLN", v.L_nc)
            .num("BLA", v.L_af)
            .num("DS", v.r_n * 2))
    if v.L_bt:
        body.num("BTAIL", 1.0, "%.1f")
    body.arr("X", oml_x, nl=True)
    body.arr("R", oml_r, nl=True)
    body.num("METHOD", 2.0, "%.1f", nl=True)
    deck.card(body)

    # --- Lifting-surface panels ---
    # No canard: tail fins are the "wing" (horizontal pair) + VT/VF panels.
    # Canard: canard is the "wing", tail fins the "horizontal tail".
    if v.has_canard:
        deck.control(f"NACA-W-{c_shape}")
        deck.card(Namelist("WGPLNF")
                  .num("CHRDTP", v.canard_tip)
                  .num("SSPNE", v.canard_height)
                  .num("SSPN", v.canard_height + r)
                  .num("DHDADI", 0.0, "%.1f", nl=True)
                  .num("CHRDR", c_l_b)
                  .num("SAVSI", v.canard_sweep)
                  .num("TYPE", 1.0, "%.1f"))
        deck.control(f"NACA-H-{v.fin_shape}")
        deck.card(Namelist("HTPLNF")
                  .num("CHRDTP", v.fin_tip)
                  .num("SSPNE", v.fin_height)
                  .num("SSPN", v.fin_height + r)
                  .num("DHDADI", dhdadi, "%.1f", nl=True)
                  .num("CHRDR", l_b)
                  .num("SAVSI", v.fin_sweep)
                  .num("TYPE", 1.0, "%.1f"))
    else:
        deck.control(f"NACA-W-{v.fin_shape}")
        deck.card(Namelist("WGPLNF")
                  .num("CHRDTP", v.fin_tip)
                  .num("SSPNE", v.fin_height)
                  .num("SSPN", v.fin_height + r)
                  .num("DHDADI", dhdadi, "%.1f", nl=True)
                  .num("CHRDR", l_b)
                  .num("SAVSI", v.fin_sweep)
                  .num("TYPE", 1.0, "%.1f"))
    deck.control(f"NACA-V-{v.fin_shape}")
    deck.card(Namelist("VTPLNF")
              .num("CHRDTP", v.fin_tip).num("SSPNE", v.fin_height)
              .num("SSPN", v.fin_height + r).num("CHRDR", l_b)
              .num("SAVSI", v.fin_sweep).num("TYPE", 1.0, "%.1f"))
    deck.card(Namelist("VFPLNF")
              .num("CHRDTP", l_b).num("SSPNE", v.fin_height)
              .num("SSPN", v.fin_height + r).num("CHRDR", v.fin_tip)
              .num("SAVSI", -v.fin_sweep).num("TYPE", 1.0, "%.1f"))

    # --- SYMFLP: symmetric (pitch) fin deflection ---
    chrdfi = chrdfo = spanfi = spanfo = 0.0
    if delta:
        chrdfi = fin_flap_c * v.fin_root
        chrdfo = fin_flap_c * v.fin_tip
        spanfi = r
        spanfo = r + v.fin_height
        deck.card(Namelist("SYMFLP")
                  .num("FTYPE", 1.0, "%.1f")
                  .num("NTYPE", 1.0, "%.1f")
                  .num("NDELTA", len(delta), "%.1f")
                  .arr("DELTA", delta, nl=True)
                  .num("PHETE", phete, "%.4f", nl=True)
                  .num("PHETEP", phetep, "%.4f")
                  .num("CHRDFI", chrdfi, "%.3f", nl=True)
                  .num("CHRDFO", chrdfo, "%.3f")
                  .num("SPANFI", spanfi, "%.3f")
                  .num("SPANFO", spanfo, "%.3f"))

    # --- Case 1 control cards ---
    deck.control("DIM " + v.unit)
    deck.control("DAMP")
    deck.control("SAVE")
    deck.control("PLOT")
    deck.control("NEXT CASE")

    # --- Chained cases for the remaining alpha batches (>17 alphas) ---
    n_batch = math.ceil(len(alpha) / ALPHA_BATCH)
    for i in range(1, n_batch):
        batch = alpha[i * ALPHA_BATCH:(i + 1) * ALPHA_BATCH]
        deck.card(Namelist("FLTCON")
                  .num("NALPHA", len(batch), "%.1f")
                  .arr("ALSCHD", batch))
        deck.control("DAMP")
        deck.control("SAVE")
        deck.control("PLOT")
        if len(batch) == ALPHA_BATCH:
            deck.control("NEXT CASE")

    # --- ASYFLP: differential (roll) deflection cases ---
    # Skipped for canard vehicles: DATCOM defines STYPE=4 ailerons on the
    # forward surface ("wing" = canard), which is not the control surface.
    if delta and not v.has_canard:
        if n_batch > 1 and len(alpha) % ALPHA_BATCH != 0:
            deck.control("NEXT CASE")   # close the last (partial) alpha case
        for i in range(n_batch):
            if i == 0:
                deck.card(Namelist("ASYFLP")
                          .num("STYPE", 4.0, "%.1f")
                          .num("NDELTA", len(delta), "%.1f")
                          .arr("DELTAL", delta, nl=True)
                          .arr("DELTAR", [-d for d in delta], nl=True)
                          .num("PHETE", phete, "%.4f", nl=True)
                          .num("CHRDFI", chrdfi, "%.3f", nl=True)
                          .num("CHRDFO", chrdfo, "%.3f")
                          .num("SPANFI", spanfi, "%.3f")
                          .num("SPANFO", spanfo, "%.3f"))
            batch = alpha[i * ALPHA_BATCH:(i + 1) * ALPHA_BATCH]
            deck.card(Namelist("FLTCON")
                      .num("NALPHA", len(batch), "%.1f")
                      .arr("ALSCHD", batch))
            deck.control("DAMP")
            deck.control("SAVE")
            deck.control("PLOT")
            if i < n_batch - 1:
                deck.control("NEXT CASE")

    return deck.text()


def write_input_file(vehicle: Vehicle,
                     M: Optional[Sequence[float]] = None,
                     alpha: Optional[Sequence[float]] = None,
                     delta: Optional[Sequence[float]] = None,
                     out_path: str = "for005.dat",
                     alt: float = 1000.0) -> str:
    """Write the DATCOM input deck to ``out_path`` and return the path."""
    text = build_deck(vehicle, M, alpha, delta, alt=alt)
    with open(out_path, "w") as f:
        f.write(text)
    return out_path
