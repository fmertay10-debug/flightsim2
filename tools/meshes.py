"""Procedural low-poly vehicle meshes in the BODY frame.

Body frame matches the sim: +x forward (nose), +y right, +z down. Meshes are
centered near the CG so attitude rotation looks right. Returned as a dict:
    {"parts": [{"name","color","vertices":[[x,y,z]...],"faces":[[i,j,k]...]}]}
Kept small (a few hundred faces) so the browser animates smoothly.
"""
import math


def _ring(x, radius, n, cx=0.0, cz=0.0):
    return [[x, cx + radius * math.cos(t), cz + radius * math.sin(t)]
            for t in [2 * math.pi * k / n for k in range(n)]]


def _tube(verts, faces, x0, r0, x1, r1, n, color_start):
    """Add a truncated-cone segment between two rings; returns updated index."""
    base = len(verts)
    verts.extend(_ring(x0, r0, n))
    verts.extend(_ring(x1, r1, n))
    quads = []
    for k in range(n):
        a = base + k
        b = base + (k + 1) % n
        c = base + n + (k + 1) % n
        d = base + n + k
        quads.append([a, b, c])
        quads.append([a, c, d])
    faces.extend(quads)
    return color_start


def rocket_mesh(length, diameter, n_fins=4, nose_frac=0.18,
                fin_frac=0.16, body_color="#c9ced6", fin_color="#e08a3c"):
    """Nose (+x forward) + cylinder body + n fins at the tail. Centered at mid."""
    r = diameter / 2.0
    n = 16
    half = length / 2.0
    xnose = half                       # nose tip forward
    xbody = half - nose_frac * length  # nose/body junction
    xtail = -half

    body_v, body_f = [], []
    # Ogive-ish nose: a few rings from tip to junction.
    steps = 5
    prev_x, prev_r = xnose, 0.0
    for s in range(1, steps + 1):
        f = s / steps
        x = xnose - f * (xnose - xbody)
        rr = r * math.sqrt(f)          # ogive curve
        _tube(body_v, body_f, prev_x, prev_r, x, rr, n, 0)
        prev_x, prev_r = x, rr
    # Cylinder body.
    _tube(body_v, body_f, xbody, r, xtail, r, n, 0)
    # Tail cap.
    base = len(body_v)
    body_v.append([xtail, 0, 0])
    ring0 = len(body_v)
    body_v.extend(_ring(xtail, r, n))
    for k in range(n):
        body_f.append([base, ring0 + (k + 1) % n, ring0 + k])

    parts = [{"name": "body", "color": body_color, "vertices": body_v, "faces": body_f}]

    # Fins: flat trapezoids around the tail, in body y-z plane.
    fin_len = fin_frac * length
    fin_h = r * 2.2
    root_x0, root_x1 = xtail + fin_len, xtail
    for i in range(n_fins):
        ang = 2 * math.pi * i / n_fins
        ca, sa = math.cos(ang), math.sin(ang)
        def pt(x, rad):
            return [x, rad * ca, rad * sa]
        fv = [pt(root_x0, r), pt(root_x1, r),
              pt(root_x1, r + fin_h), pt(root_x0 + fin_len * 0.3, r + fin_h)]
        ff = [[0, 1, 2], [0, 2, 3]]
        parts.append({"name": f"fin_{i+1}", "color": fin_color,
                      "vertices": fv, "faces": ff})
    return {"parts": parts}


def aircraft_mesh(span, length, body_color="#8892a0",
                  wing_color="#5a9fd4", tail_color="#d45a5a"):
    """Fuselage tube + swept wings + horizontal & vertical tail. +x forward."""
    r = length * 0.05
    n = 12
    half = length / 2.0
    nose, tail = half, -half

    fv, ff = [], []
    _tube(fv, ff, nose, r * 0.25, nose - length * 0.2, r, n, 0)
    _tube(fv, ff, nose - length * 0.2, r, tail + length * 0.12, r * 0.7, n, 0)
    _tube(fv, ff, tail + length * 0.12, r * 0.7, tail, r * 0.2, n, 0)
    parts = [{"name": "fuselage", "color": body_color, "vertices": fv, "faces": ff}]

    b = span / 2.0
    cr, ct = length * 0.32, length * 0.12   # root/tip chord
    xle = length * 0.05                      # wing root leading edge (fwd of CG)
    sweep = length * 0.14
    def wing(sign):
        y = sign * b
        v = [[xle, 0, 0], [xle - cr, 0, 0],
             [xle - sweep - ct, y, 0], [xle - sweep, y, 0]]
        f = [[0, 1, 2], [0, 2, 3]] if sign > 0 else [[0, 2, 1], [0, 3, 2]]
        return {"name": "wing", "color": wing_color, "vertices": v, "faces": f}
    parts.append(wing(+1))
    parts.append(wing(-1))

    # Horizontal stabilizer.
    hb = span * 0.34
    xh = tail + length * 0.14
    ch = length * 0.14
    for sign in (+1, -1):
        y = sign * hb
        v = [[xh, 0, 0], [xh - ch, 0, 0],
             [xh - ch - length * 0.05, y, 0], [xh - length * 0.04, y, 0]]
        f = [[0, 1, 2], [0, 2, 3]] if sign > 0 else [[0, 2, 1], [0, 3, 2]]
        parts.append({"name": "htail", "color": tail_color, "vertices": v, "faces": f})

    # Vertical tail (in x-z plane, up = -z).
    xv = tail + length * 0.16
    vh = length * 0.16
    v = [[xv, 0, 0], [xv - length * 0.13, 0, 0],
         [xv - length * 0.15, 0, -vh], [xv - length * 0.04, 0, -vh]]
    parts.append({"name": "vtail", "color": tail_color,
                  "vertices": v, "faces": [[0, 1, 2], [0, 2, 3]]})
    return {"parts": parts}


def mesh_for(vehicle_cfg, dynamics):
    """Choose a mesh from a loaded vehicle.json (or None for kinematic movers)."""
    if vehicle_cfg is None or dynamics == "kinematic":
        # Generic small dart for targets/traffic.
        return aircraft_mesh(span=6.0, length=8.0, body_color="#9aa0a6",
                             wing_color="#b0b4b8", tail_color="#b0b4b8")

    vtype = vehicle_cfg.get("type", "rocket")
    if vtype == "aircraft":
        aero = vehicle_cfg.get("aero", {})
        if "dir" in aero:
            # Table-aero aircraft (e.g. the F-16): dimensions aren't in the JSON.
            span, length = 9.14, 15.0
        else:
            span = aero.get("bspan_m", 10.0)
            length = max(span * 0.9, aero.get("cbar_m", 1.5) * 6.0)
        return aircraft_mesh(span=span, length=length)

    # Rocket: prefer explicit geometry, else infer from aero references.
    geom = vehicle_cfg.get("geometry", {})
    aero = vehicle_cfg.get("aero", {})
    length = geom.get("length_m") or aero.get("lref_m") or aero.get("cbar_m", 3.0)
    diameter = geom.get("diameter_m") or aero.get("dref_m") or (length * 0.06)
    return rocket_mesh(length=length, diameter=diameter)
