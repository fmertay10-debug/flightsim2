"""Native 3-D visualization of a :class:`Vehicle` (no external tools).

Builds a triangle mesh straight from the same geometry the deck writer
uses — body of revolution (true tangent-ogive or conical nose, cylinder,
optional boattail or explicit OML), cruciform fins, optional canard — and
offers two outputs:

* :func:`vehicle_to_obj` — Wavefront ``.obj`` file; double-click opens in
  Windows 3D Viewer (and imports everywhere else).
* :func:`plot_vehicle` — interactive matplotlib 3-D figure (optional dep).

The mesh is for the eyes: panels are thin extruded trapezoids and the
nose bluntness radius is not modeled.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from .vehicle import Vehicle

Part = Tuple[str, np.ndarray, np.ndarray]   # (name, vertices [n,3], faces [m,3])

_N_SEG = 48        # azimuthal segments of the body of revolution
_N_NOSE = 24       # profile points along the nose curve


def _tand(deg: float) -> float:
    return math.tan(math.radians(deg))


# ---------------------------------------------------------------------------
# Body of revolution
# ---------------------------------------------------------------------------
def _body_profile(v: Vehicle) -> Tuple[np.ndarray, np.ndarray]:
    """Meridian profile (x, r) from nose tip to base."""
    if v.oml_x:
        return np.asarray(v.oml_x, float), np.asarray(v.oml_r, float)

    r = v.D / 2.0
    # nose: cosine-spaced stations for tip resolution
    t = 0.5 * (1 - np.cos(np.linspace(0, np.pi, _N_NOSE)))
    x_n = t * v.L_nc
    if v.nc == "ogive" and v.L_nc > r:
        rho = (r ** 2 + v.L_nc ** 2) / (2 * r)          # tangent-ogive radius
        r_n = np.sqrt(rho ** 2 - (v.L_nc - x_n) ** 2) + r - rho
    else:                                               # conical
        r_n = r * x_n / v.L_nc
    xs = list(x_n) + [v.L_nc + v.L_af]
    rs = list(r_n) + [r]
    if v.L_bt:
        xs.append(v.L_nc + v.L_af + v.L_bt)
        rs.append(v.D_bt / 2.0)
    return np.asarray(xs), np.asarray(rs)


def _revolve(xs: np.ndarray, rs: np.ndarray, n_seg: int = _N_SEG) -> Part:
    """Revolve a meridian profile around the x-axis; close the base."""
    theta = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    verts: List[Tuple[float, float, float]] = []
    levels: List[Tuple[str, int]] = []          # ("point"|"ring", start index)
    for x, r in zip(xs, rs):
        if r <= 1e-12:
            levels.append(("point", len(verts)))
            verts.append((x, 0.0, 0.0))
        else:
            levels.append(("ring", len(verts)))
            verts.extend(zip([x] * n_seg, r * cos_t, r * sin_t))

    faces: List[Tuple[int, int, int]] = []
    for (kind_a, a), (kind_b, b) in zip(levels, levels[1:]):
        for j in range(n_seg):
            k = (j + 1) % n_seg
            if kind_a == "point" and kind_b == "ring":
                faces.append((a, b + j, b + k))
            elif kind_a == "ring" and kind_b == "point":
                faces.append((a + j, b, a + k))
            elif kind_a == "ring" and kind_b == "ring":
                faces.append((a + j, b + j, b + k))
                faces.append((a + j, b + k, a + k))
    # base cap
    kind, s = levels[-1]
    if kind == "ring":
        c = len(verts)
        verts.append((xs[-1], 0.0, 0.0))
        for j in range(n_seg):
            faces.append((s + (j + 1) % n_seg, s + j, c))

    return "body", np.asarray(verts), np.asarray(faces, int)


# ---------------------------------------------------------------------------
# Fin panels
# ---------------------------------------------------------------------------
def _fin_azimuths(n: int) -> List[float]:
    """Panel azimuth angles [deg]; 0 = +y, 90 = +z (up)."""
    return [90.0, 210.0, 330.0] if n == 3 else [0.0, 90.0, 180.0, 270.0]


def _fin_prism(x_le: float, c_root: float, c_tip: float, sweep: float,
               height: float, r_mount: float, azim_deg: float,
               thickness: float, name: str) -> Part:
    """Thin extruded trapezoid for one fin panel."""
    th = math.radians(azim_deg)
    u = np.array([0.0, math.cos(th), math.sin(th)])     # spanwise (radial)
    w = np.array([0.0, -math.sin(th), math.cos(th)])    # thickness direction
    ex = np.array([1.0, 0.0, 0.0])

    s0 = 0.92 * r_mount                                  # bury root in body
    s1 = r_mount + height
    x_tip = x_le + height * _tand(sweep)
    corners = [x_le * ex + s0 * u,                       # root LE
               x_tip * ex + s1 * u,                      # tip LE
               (x_tip + c_tip) * ex + s1 * u,            # tip TE
               (x_le + c_root) * ex + s0 * u]            # root TE

    half = 0.5 * thickness * w
    verts = np.array([c + half for c in corners] +
                     [c - half for c in corners])        # 0-3 front, 4-7 back
    quads = [(0, 1, 2, 3), (7, 6, 5, 4),                 # faces
             (0, 4, 5, 1), (1, 5, 6, 2),                 # LE, tip
             (2, 6, 7, 3), (3, 7, 4, 0)]                 # TE, root
    faces = []
    for a, b, c, d in quads:
        faces.append((a, b, c))
        faces.append((a, c, d))
    return name, verts, np.asarray(faces, int)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def vehicle_mesh(v: Vehicle) -> List[Part]:
    """Triangle-mesh parts for the whole vehicle."""
    xs, rs = _body_profile(v)
    parts = [_revolve(xs, rs)]

    fin_le = v.L - v.fin_disp - v.fin_root
    r_fin = float(np.interp(fin_le + v.fin_root / 2, xs, rs))
    t_fin = 0.03 * v.fin_root
    for i, az in enumerate(_fin_azimuths(v.fin_num), 1):
        parts.append(_fin_prism(fin_le, v.fin_root, v.fin_tip, v.fin_sweep,
                                v.fin_height, r_fin, az, t_fin, f"fin_{i}"))

    if v.has_canard:
        r_c = float(np.interp(v.canard_x + v.canard_root / 2, xs, rs))
        t_c = 0.03 * v.canard_root
        for i, az in enumerate(_fin_azimuths(v.fin_num), 1):
            parts.append(_fin_prism(v.canard_x, v.canard_root, v.canard_tip,
                                    v.canard_sweep, v.canard_height, r_c,
                                    az, t_c, f"canard_{i}"))
    return parts


def vehicle_to_obj(v: Vehicle, path: str) -> str:
    """Write the vehicle mesh as a Wavefront ``.obj`` and return ``path``."""
    parts = vehicle_mesh(v)
    with open(path, "w") as f:
        f.write("# generated by pydatcom.geometry.vehicle_to_obj\n")
        offset = 1                                       # .obj is 1-based
        for name, verts, faces in parts:
            f.write(f"o {name}\n")
            for x, y, z in verts:
                f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for a, b, c in faces:
                f.write(f"f {a + offset} {b + offset} {c + offset}\n")
            offset += len(verts)
    return path


def plot_vehicle(v: Vehicle, title: str = ""):
    """Interactive matplotlib 3-D view; returns the figure."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    colors = {"body": (0.78, 0.78, 0.80), "fin": (0.85, 0.25, 0.25),
              "canard": (0.25, 0.45, 0.85)}
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(projection="3d")
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for name, verts, faces in vehicle_mesh(v):
        color = colors["body" if name == "body"
                       else "canard" if name.startswith("canard") else "fin"]
        try:                                    # shade= needs matplotlib 3.7+
            coll = Poly3DCollection(verts[faces], facecolors=color,
                                    shade=True)
        except (TypeError, ValueError):
            coll = Poly3DCollection(verts[faces], facecolors=color,
                                    edgecolors=(0, 0, 0, 0.08),
                                    linewidths=0.2)
        ax.add_collection3d(coll)
        lo = np.minimum(lo, verts.min(axis=0))
        hi = np.maximum(hi, verts.max(axis=0))
    span = hi - lo
    pad = 0.03 * span.max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_zlim(lo[2] - pad, hi[2] + pad)
    ax.set_box_aspect(span + 2 * pad)
    ax.set_xlabel(f"x [{v.unit.lower()}]")
    ax.set_title(title or "pydatcom vehicle")
    return fig
