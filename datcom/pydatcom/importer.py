"""Parse a USAF Digital DATCOM ``for006.dat`` output file.

Python port of MATLAB ``datcom_import.m``. Reads the static and dynamic
stability coefficients plus the fin-control increments ($SYMFLP pitch and
$ASYFLP roll) into an :class:`AeroData` object whose fields mirror the
MATLAB ``aero`` struct (NumPy arrays, shape ``[nalpha x nmach]`` and, for
the control increments, ``[nalpha x ndelta x nmach]``).
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

# Line-matching strings (identical to the MATLAB source)
_M_CASE = " CASEID "
_M_FLTCON = "  $FLTCON"
_M_SYNTHS = "  $SYNTHS"
_M_OPTINS = "  $OPTINS"
_M_BODY = "  $BODY"
_M_STRT = "1          THE FOLLOWING IS A LIST OF ALL"
_M_STATIC = "ALPHA     CD       CL"
_M_STATIC_END = "1"
_M_DYNAMIC = "ALPHA       CLQ          CMQ           CLAD"
_M_DYNAMIC_END = "0*** NDM PRINTED WHEN NO DATCOM METHODS EXIST"
_M_HILIFT = "CHARACTERISTICS OF HIGH LIFT AND CONTROL DEVICES"
_M_SYMFLP = "DELTA     D(CL)"
_M_DCDI = "INDUCED DRAG COEFFICIENT INCREMENT"
_M_YAW_ASY = "YAWING MOMENT COEFFICIENT,CN,DUE TO CONTROL DEFLECTION"
_M_ROLL = "DELTAL          DELTAR          (CL)ROLL"

_SPLIT_SPACE = re.compile(r" +")

# Coefficient tables stored as [nalpha x nmach]
_STAT_COEFFS = ["cd", "cl", "cm", "cn", "ca", "xcp", "cma", "cyb", "cnb",
                "clb", "cla", "clq", "cmq", "clad", "cmad", "clp", "cyp",
                "cnp", "cnr", "clr"]

# Fin-control tables (present only when a deflection schedule was run)
_CTRL_COEFFS = ["dcl_sym", "dcm_sym", "dclmax_sym", "dcdmin_sym", "dcdi_sym",
                "clroll", "cn_asy"]


def _to_double(s: str) -> float:
    """MATLAB ``str2double`` semantics: non-numeric -> NaN."""
    try:
        return float(s)
    except (ValueError, TypeError):
        return float("nan")


def _split_space(line: str) -> List[str]:
    """Mirror MATLAB ``strsplit(line, ' ')`` (collapse runs of spaces;
    a leading space yields a leading empty token)."""
    return _SPLIT_SPACE.split(line)


@dataclass
class AeroData:
    """DATCOM aerodynamic coefficient set (mirrors the MATLAB struct)."""

    mach: np.ndarray = field(default_factory=lambda: np.array([]))
    alpha: np.ndarray = field(default_factory=lambda: np.array([]))
    nmach: int = 0
    nalpha: int = 0
    sref: float = 0.0
    cbar: float = 0.0
    blref: float = 0.0
    dim: str = "ft"
    deriv: str = "deg"
    xcg: float = 0.0
    zcg: float = 0.0
    rougfc: float = 0.0
    mcrit: float = 0.0
    ndelta: int = 0
    delta: np.ndarray = field(default_factory=lambda: np.array([]))
    deltal: np.ndarray = field(default_factory=lambda: np.array([]))
    deltar: np.ndarray = field(default_factory=lambda: np.array([]))

    # Static / dynamic coefficient tables [nalpha x nmach]
    cd: np.ndarray = None
    cl: np.ndarray = None
    cm: np.ndarray = None
    cn: np.ndarray = None
    ca: np.ndarray = None
    xcp: np.ndarray = None
    cma: np.ndarray = None
    cyb: np.ndarray = None
    cnb: np.ndarray = None
    clb: np.ndarray = None
    cla: np.ndarray = None
    clq: np.ndarray = None
    cmq: np.ndarray = None
    clad: np.ndarray = None
    cmad: np.ndarray = None
    clp: np.ndarray = None
    cyp: np.ndarray = None
    cnp: np.ndarray = None
    cnr: np.ndarray = None
    clr: np.ndarray = None

    # Fin-control tables
    dcl_sym: Optional[np.ndarray] = None      # [ndelta x nmach]
    dcm_sym: Optional[np.ndarray] = None      # [ndelta x nmach]
    dclmax_sym: Optional[np.ndarray] = None   # [ndelta x nmach]
    dcdmin_sym: Optional[np.ndarray] = None   # [ndelta x nmach]
    dcdi_sym: Optional[np.ndarray] = None     # [nalpha x ndelta x nmach]
    clroll: Optional[np.ndarray] = None       # [ndeltal x nmach]
    cn_asy: Optional[np.ndarray] = None       # [nalpha x ndeltal x nmach]

    # Fill mask (ROADMAP D9): fill[name][...] is True where the value was
    # NOT read from DATCOM output but produced by a repair (interpolation,
    # forward-fill, zero-fill or row copy). Deliberate modeling choices
    # (alpha=0 broadcast, empirical sideslip reuse) inherit the mask of
    # their source cells rather than being marked as repairs.
    fill: Optional[Dict[str, np.ndarray]] = None

    def save_npz(self, path: str) -> None:
        """Save all array/scalar fields to a ``.npz`` archive.

        Fill-mask entries are flattened to ``fill_<name>`` boolean arrays.
        """
        data = {}
        for k, v in self.__dict__.items():
            if v is None:
                continue
            if k == "fill":
                for name, mask in v.items():
                    data["fill_" + name] = mask
            else:
                data[k] = v
        np.savez(path, **data)


class _Reader:
    """Sequential line reader returning ``None`` at EOF (like ``fgetl``)."""

    def __init__(self, lines: List[str]):
        self._lines = lines
        self._i = 0

    def getl(self) -> Optional[str]:
        if self._i >= len(self._lines):
            return None
        line = self._lines[self._i]
        self._i += 1
        return line


# ---------------------------------------------------------------------------
# NaN-fill helpers (mirror MATLAB fillmissing)
# ---------------------------------------------------------------------------
def _interp_nan_columns(A: np.ndarray, x: np.ndarray) -> np.ndarray:
    """fillmissing(A,'linear','SamplePoints',x) per column: linearly
    interpolate interior NaNs, leave leading/trailing NaNs untouched."""
    A = A.astype(float).copy()
    for j in range(A.shape[1]):
        col = A[:, j]
        mask = ~np.isnan(col)
        if mask.sum() < 2:
            continue
        idx = np.where(mask)[0]
        first, last = idx[0], idx[-1]
        interior = np.arange(first, last + 1)
        need = interior[np.isnan(col[interior])]
        if need.size:
            col[need] = np.interp(x[need], x[idx], col[idx])
        A[:, j] = col
    return A


def _ffill_axis0(A: np.ndarray) -> np.ndarray:
    """fillmissing(A,'previous') along dim 0 (leading NaNs stay NaN)."""
    A = A.astype(float).copy()
    for j in range(A.shape[1]):
        last = np.nan
        for i in range(A.shape[0]):
            if np.isnan(A[i, j]):
                A[i, j] = last
            else:
                last = A[i, j]
    return A


def _ffill_axis1(A: np.ndarray) -> np.ndarray:
    """fillmissing(A','previous')' -> previous-fill along dim 1."""
    return _ffill_axis0(A.T).T


def import_datcom(filename: str = "for006.dat") -> AeroData:
    """Parse ``filename`` and return the populated :class:`AeroData`."""
    aero = AeroData()

    # Mutable accumulators used during the control-card pass
    mach_list: List[float] = []
    alpha_list: List[float] = []
    delta_list: List[float] = []
    deltal_list: List[float] = []
    deltar_list: List[float] = []

    with open(filename, "r", errors="replace") as fh:
        rdr = _Reader(fh.read().splitlines())

    search_flag = 1
    mach_count = 0          # 0-based mach index (MATLAB mach_count starts at 1)
    alpha_count = 0         # 0-based alpha block start
    ctrl_mach = 0
    subsonic = True
    alpha_temp = 0
    sym_done = roll_done = None

    tline = rdr.getl()
    while tline is not None:
        if search_flag == 1:
            # ----- control cards -----
            if tline.startswith(_M_CASE):
                while tline is not None and not tline.startswith(_M_STRT):
                    if tline.startswith(_M_FLTCON):
                        if "MACH(1)=" in tline:
                            rest = tline.split("MACH(1)=", 1)[1]
                            mach_list += _scan_floats(rest)
                            tline = rdr.getl()
                            while tline is not None and tline.startswith("   "):
                                mach_list += _scan_floats(tline)
                                tline = rdr.getl()
                        elif "ALSCHD(1)=" in tline:
                            rest = tline.split("ALSCHD(1)=", 1)[1]
                            alpha_list += _scan_floats(rest)
                            tline = rdr.getl()
                            while tline is not None and tline.startswith("   "):
                                alpha_list += _scan_floats(tline)
                                tline = rdr.getl()
                        elif "ALT(1)=" in tline:
                            tline = rdr.getl()
                        else:
                            tline = rdr.getl()
                    elif tline.startswith(_M_SYNTHS):
                        aero.xcg = _after_float(tline, "XCG=")
                        aero.zcg = _after_float(tline, "ZCG=")
                        tline = rdr.getl()
                    elif tline.startswith(_M_OPTINS):
                        aero.sref = _after_float(tline, "SREF=")
                        aero.cbar = _after_float(tline, "CBARR=")
                        aero.blref = _after_float(tline, "BLREF=")
                        aero.rougfc = _after_float(tline, "ROUGFC")
                        tline = rdr.getl()
                    elif tline.startswith(_M_BODY):
                        tline = rdr.getl()
                    elif tline.startswith(" DAMP"):
                        tline = rdr.getl()
                    elif tline.startswith(" SAVE"):
                        tline = rdr.getl()
                    elif tline.startswith(" DIM"):
                        dim = tline.split(" DIM ", 1)[1] if " DIM " in tline else ""
                        if dim.strip() == "FT":
                            aero.dim = "ft"
                        elif dim.strip() == "M":
                            aero.dim = "m"
                        tline = rdr.getl()
                    elif ("DELTA(1)=" in tline or "DELTAL(1)=" in tline
                          or "DELTAR(1)=" in tline):
                        if "DELTAL(1)=" in tline:
                            key, target = "DELTAL(1)=", deltal_list
                        elif "DELTAR(1)=" in tline:
                            key, target = "DELTAR(1)=", deltar_list
                        else:
                            key, target = "DELTA(1)=", delta_list
                        target += _scan_floats(tline.split(key, 1)[1])
                        tline = rdr.getl()
                        while (tline is not None and tline.startswith("   ")
                               and _scan_floats(tline)):
                            target += _scan_floats(tline)
                            tline = rdr.getl()
                    else:
                        tline = rdr.getl()
            elif tline.startswith(_M_STRT):
                search_flag = 2
                # Dedup the alpha schedule (control cases echo it again)
                aero.mach = np.array(mach_list, dtype=float)
                aero.alpha = np.array(sorted(set(alpha_list)), dtype=float)
                aero.delta = np.array(delta_list, dtype=float)
                aero.deltal = np.array(deltal_list, dtype=float)
                aero.deltar = np.array(deltar_list, dtype=float)
                aero.ndelta = len(aero.delta)

                na, nm = len(aero.alpha), len(aero.mach)
                for name in _STAT_COEFFS:
                    setattr(aero, name, np.zeros((na, nm)))
                if aero.ndelta > 0:
                    aero.dcl_sym = np.zeros((aero.ndelta, nm))
                    aero.dcm_sym = np.zeros((aero.ndelta, nm))
                    aero.dclmax_sym = np.zeros((aero.ndelta, nm))
                    aero.dcdmin_sym = np.zeros((aero.ndelta, nm))
                    aero.dcdi_sym = np.zeros((na, aero.ndelta, nm))
                if len(aero.deltal) > 0:
                    aero.clroll = np.zeros((len(aero.deltal), nm))
                    aero.cn_asy = np.zeros((na, len(aero.deltal), nm))
                sym_done = [False] * nm
                roll_done = [False] * nm
                tline = rdr.getl()
            else:
                tline = rdr.getl()
                continue

        elif search_flag == 2:
            # ----- flight coefficient data -----
            if tline.startswith(" NUMBER"):
                rdr.getl()
                tline = rdr.getl()
                parts = (tline or "").split()
                mach = _to_double(parts[1]) if len(parts) > 1 else float("nan")
                subsonic = not (mach > 1)
            elif _M_STATIC in tline:
                rdr.getl()
                tline = rdr.getl()
                c = _split_space(tline)
                cols = {k: [_to_double(c[idx])] for k, idx in
                        (("cd", 2), ("cl", 3), ("cm", 4), ("cn", 5), ("ca", 6),
                         ("xcp", 7), ("cla", 8), ("cma", 9), ("cyb", 10),
                         ("cnb", 11))}
                alpha_temp = 0
                tline = rdr.getl()
                while tline is not None and not tline.startswith(_M_STATIC_END):
                    c = _split_space(tline)
                    # Skip non-data lines inside the block (e.g. the
                    # "0NOTE - CANARD CONFIGURATION ..." advisory)
                    if len(c) < 3 or np.isnan(_to_double(c[1])):
                        tline = rdr.getl()
                        continue
                    cols["cd"].append(_to_double(c[2]))
                    cols["cl"].append(_to_double(c[3]))
                    cols["cm"].append(_to_double(c[4]))
                    cols["cn"].append(_to_double(c[5]))
                    cols["ca"].append(_to_double(c[6]))
                    cols["xcp"].append(_to_double(c[7]))
                    cols["cla"].append(_to_double(c[8]))
                    cols["cma"].append(_to_double(c[9]) if subsonic else float("nan"))
                    cols["cyb"].append(float("nan"))
                    cols["cnb"].append(float("nan"))
                    alpha_temp += 1
                    tline = rdr.getl()
                sl = slice(alpha_count, alpha_count + alpha_temp + 1)
                for k in ("cd", "cl", "cm", "cn", "ca", "xcp", "cla", "cma",
                          "cyb", "cnb"):
                    getattr(aero, k)[sl, mach_count] = cols[k]
            elif _M_DYNAMIC in tline:
                rdr.getl()
                tline = rdr.getl()
                c = _split_space(tline)
                cols = {"clq": [_to_double(c[2])], "cmq": [_to_double(c[3])],
                        "cmad": [_to_double(c[5])], "clp": [_to_double(c[6])],
                        "cyp": [_to_double(c[7])], "cnp": [_to_double(c[8])],
                        "cnr": [_to_double(c[9])], "clr": [_to_double(c[10])]}
                alpha_temp = 0
                tline = rdr.getl()
                while tline is not None and not tline.startswith(_M_DYNAMIC_END):
                    c = _split_space(tline)
                    # Skip non-data lines inside the block (config notes)
                    if len(c) < 3 or np.isnan(_to_double(c[1])):
                        tline = rdr.getl()
                        continue
                    cols["clp"].append(_to_double(c[2]))
                    cols["cyp"].append(_to_double(c[3]))
                    cols["cnp"].append(_to_double(c[4]))
                    cols["cnr"].append(_to_double(c[5]))
                    cols["clr"].append(_to_double(c[6]))
                    cols["clq"].append(float("nan"))
                    cols["cmq"].append(float("nan"))
                    cols["cmad"].append(float("nan"))
                    alpha_temp += 1
                    tline = rdr.getl()
                sl = slice(alpha_count, alpha_count + alpha_temp + 1)
                for k in ("clq", "cmq", "cmad", "clp", "cyp", "cnp", "cnr", "clr"):
                    getattr(aero, k)[sl, mach_count] = cols[k]
                mach_count += 1
            elif _M_HILIFT in tline:
                tline = rdr.getl()
                while tline is not None:
                    if tline.startswith("0 "):
                        m = _scan_floats(tline[1:])
                        if m:
                            ctrl_mach = int(np.argmin(np.abs(aero.mach - m[0])))
                            break
                    tline = rdr.getl()
            elif _M_SYMFLP in tline:
                count = 0
                while count < aero.ndelta and tline is not None:
                    tline = rdr.getl()
                    c = (tline or "").strip().split()
                    if len(c) >= 5 and not np.isnan(_to_double(c[0])):
                        if not sym_done[ctrl_mach]:
                            aero.dcl_sym[count, ctrl_mach] = _to_double(c[1])
                            aero.dcm_sym[count, ctrl_mach] = _to_double(c[2])
                            aero.dclmax_sym[count, ctrl_mach] = _to_double(c[3])
                            aero.dcdmin_sym[count, ctrl_mach] = _to_double(c[4])
                        count += 1
                sym_done[ctrl_mach] = True
            elif _M_DCDI in tline:
                rdr.getl(); rdr.getl(); rdr.getl()
                tline = rdr.getl()
                while tline is not None:
                    c = tline.strip().split()
                    a = _to_double(c[0]) if c else float("nan")
                    if np.isnan(a) or len(c) < 2:
                        break
                    a_idx = int(np.argmin(np.abs(aero.alpha - a)))
                    n = min(len(c) - 1, aero.ndelta)
                    aero.dcdi_sym[a_idx, 0:n, ctrl_mach] = \
                        [_to_double(v) for v in c[1:n + 1]]
                    tline = rdr.getl()
            elif _M_YAW_ASY in tline:
                rdr.getl(); rdr.getl(); rdr.getl()
                tline = rdr.getl()
                while tline is not None:
                    c = tline.strip().split()
                    a = _to_double(c[0]) if c else float("nan")
                    if np.isnan(a) or len(c) < 2:
                        break
                    a_idx = int(np.argmin(np.abs(aero.alpha - a)))
                    n = min(len(c) - 1, len(aero.deltal))
                    aero.cn_asy[a_idx, 0:n, ctrl_mach] = \
                        [_to_double(v) for v in c[1:n + 1]]
                    tline = rdr.getl()
            elif _M_ROLL in tline:
                count = 0
                while count < len(aero.deltal) and tline is not None:
                    tline = rdr.getl()
                    c = (tline or "").strip().split()
                    if len(c) >= 3 and not np.isnan(_to_double(c[0])):
                        if not roll_done[ctrl_mach]:
                            aero.clroll[count, ctrl_mach] = _to_double(c[2])
                        count += 1
                roll_done[ctrl_mach] = True
            elif "CREST CRITICAL MACH =" in tline:
                aero.mcrit = _after_float(tline, "=")
                tline = rdr.getl()
            else:
                tline = rdr.getl()
                continue

            # case complete -> advance alpha block / reset mach counter
            if mach_count >= len(aero.mach):
                mach_count = 0
                alpha_count += alpha_temp + 1
                if alpha_count >= len(aero.alpha):
                    alpha_count = 0

    _postprocess(aero)
    return aero


_FLOAT_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


def _scan_floats(s: str) -> List[float]:
    """MATLAB ``sscanf(s,'%f,')``: skip whitespace, read a float, consume an
    optional comma, repeat; stop at the first non-numeric character (e.g. the
    ``$`` namelist terminator that abuts the last value, as in ``5.00$``)."""
    out: List[float] = []
    pos, n = 0, len(s)
    while pos < n:
        while pos < n and s[pos] in " \t":
            pos += 1
        m = _FLOAT_RE.match(s, pos)
        if not m:
            break
        out.append(float(m.group()))
        pos = m.end()
        if pos < n and s[pos] == ",":
            pos += 1
    return out


def _after_float(line: str, key: str) -> float:
    """MATLAB ``sscanf(extractAfter(line,key),'%f')``."""
    if key not in line:
        return 0.0
    rest = line.split(key, 1)[1]
    m = re.match(r"\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)", rest)
    return float(m.group(1)) if m else 0.0


def _postprocess(aero: AeroData) -> None:
    """Repairs, NaN handling and negative-alpha extension (datcom_import tail).

    Every repair is recorded in ``aero.fill`` (ROADMAP D9): a cell's mask is
    True when its value did not come from DATCOM output. The masks undergo
    the same copies/mirroring as the data so they stay cell-aligned.
    """
    # DATCOM prints literal 0.0 where a method drops out at high alpha
    # (seen for CN/CM at |alpha| >= 20 on some geometries, ROADMAP D11).
    # Detect maximal runs of exact zeros in 10 <= |alpha| <= 45 whose left
    # neighbor is large and whose right bound is either a large SAME-SIGN
    # value (a genuine zero crossing has opposite signs) or the table end
    # (short campaign schedules). Cells become NaN before the mask
    # snapshot, then bounded runs are repaired by local interpolation.
    # Legacy behavior beyond 45 deg is deliberately untouched.
    dropout_runs = []                     # (name, col j, start i, end k)
    for name in ("cn", "cm"):
        A = getattr(aero, name)
        for j in range(A.shape[1]):
            col = A[:, j]
            n = len(col)
            i = 1
            while i < n:
                if (col[i] == 0.0 and 10.0 <= abs(aero.alpha[i]) <= 45.0
                        and abs(col[i - 1]) > 0.05):
                    k = i
                    while (k < n and col[k] == 0.0
                           and abs(aero.alpha[k]) <= 45.0):
                        k += 1
                    ok = (k >= n or (abs(col[k]) > 0.05
                                     and col[i - 1] * col[k] > 0))
                    if ok:
                        col[i:k] = np.nan
                        dropout_runs.append((name, j, i, k))
                    i = k
                else:
                    i += 1

    # Snapshot NaN locations before any repair -> the fill mask
    fill: Dict[str, np.ndarray] = {
        name: np.isnan(getattr(aero, name)) for name in _STAT_COEFFS}
    for name in _CTRL_COEFFS:
        A = getattr(aero, name)
        if A is not None:
            fill[name] = np.isnan(A)

    # Repair: first xcp row = second row, fix mach(1)/alpha(end)
    aero.xcp[0, :] = aero.xcp[1, :]
    fill["xcp"][0, :] = True
    aero.nalpha = len(aero.alpha)
    aero.nmach = len(aero.mach)
    # Normalize the default envelope's stand-in values only: M=0.01 is the
    # deck's stand-in for M=0, and the default alpha schedule ends at 180.
    # Arbitrary campaign schedules (e.g. M=[2,3]) must not be rewritten —
    # the legacy unconditional rewrite mislabeled the first Mach / last
    # alpha of any non-default schedule.
    if aero.mach[0] <= 0.011:
        aero.mach[0] = 0.0
    if aero.alpha[-1] >= 179.5:
        aero.alpha[-1] = 180.0

    # Repair bounded dropout runs by local interpolation (unbounded runs
    # at the table edge fall through to the zero-fill below, but masked)
    for name, j, i, k in dropout_runs:
        col = getattr(aero, name)[:, j]
        if k < len(col):
            col[i:k] = np.interp(aero.alpha[i:k],
                                 [aero.alpha[i - 1], aero.alpha[k]],
                                 [col[i - 1], col[k]])

    # Repair NaN data
    aero.cma = _interp_nan_columns(aero.cma, aero.alpha)
    aero.clq = _ffill_axis0(aero.clq)
    aero.cmq = _ffill_axis0(aero.cmq)
    aero.clp = _ffill_axis1(aero.clp)
    aero.cyp = _ffill_axis1(aero.cyp)
    aero.cnp = _ffill_axis1(aero.cnp)
    aero.cnr = _ffill_axis1(aero.cnr)
    aero.clr = _ffill_axis1(aero.clr)

    # Remaining NaN -> 0 (static/dynamic + control tables)
    for name in _STAT_COEFFS + _CTRL_COEFFS:
        A = getattr(aero, name)
        if A is not None:
            A[np.isnan(A)] = 0.0

    # Broadcast the alpha=0 value across all alpha for these derivatives
    # (deliberate: DATCOM reports them once per Mach; mask broadcasts too)
    for name in ("cyb", "cnb", "clq", "cmq", "clad", "cmad"):
        A = getattr(aero, name)
        A[:, :] = A[0:1, :]
        fill[name][:, :] = fill[name][0:1, :]

    # Empirical sideslip data: reuse the alpha derivatives
    aero.cyb = aero.cla.copy()
    aero.cnb = aero.cma.copy()
    fill["cyb"] = fill["cla"].copy()
    fill["cnb"] = fill["cma"].copy()

    # Flip & extend for negative alpha
    a = aero.alpha
    aero.alpha = np.concatenate([-a[1:][::-1], a])

    odd = {"cl", "cm", "cn"}  # antisymmetric in alpha
    for name in _STAT_COEFFS:
        A = getattr(aero, name)
        top = A[1:, :][::-1, :]
        if name in odd:
            top = -top
        setattr(aero, name, np.concatenate([top, A], axis=0))
        M = fill[name]
        fill[name] = np.concatenate([M[1:, :][::-1, :], M], axis=0)

    # Control increments: dcl/dcm/dclmax/dcdmin/clroll depend on (delta,mach)
    # only; only dcdi_sym and cn_asy are functions of alpha.
    if aero.ndelta > 0:
        d = aero.delta
        if np.max(np.abs(d + d[::-1])) < 1e-6:
            # symmetric schedule: dCDi(-alpha,delta) = dCDi(alpha,-delta)
            dcdi_neg = aero.dcdi_sym[1:, ::-1, :][::-1, :, :]
            mask_neg = fill["dcdi_sym"][1:, ::-1, :][::-1, :, :]
        else:
            dcdi_neg = aero.dcdi_sym[1:, :, :][::-1, :, :]
            mask_neg = fill["dcdi_sym"][1:, :, :][::-1, :, :]
        aero.dcdi_sym = np.concatenate([dcdi_neg, aero.dcdi_sym], axis=0)
        fill["dcdi_sym"] = np.concatenate([mask_neg, fill["dcdi_sym"]], axis=0)
    if len(aero.deltal) > 0:
        cn_neg = -aero.cn_asy[1:, :, :][::-1, :, :]
        aero.cn_asy = np.concatenate([cn_neg, aero.cn_asy], axis=0)
        M = fill["cn_asy"]
        fill["cn_asy"] = np.concatenate([M[1:, :, :][::-1, :, :], M], axis=0)

    aero.fill = fill
