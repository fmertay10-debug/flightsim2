"""Procedural low-poly vehicle meshes in the BODY frame.

Body frame matches the sim: +x forward (nose), +y right, +z down. Meshes are
centered near the CG so attitude rotation looks right. Returned as a dict:
    {"parts": [{"name","color","vertices":[[x,y,z]...],"faces":[[i,j,k]...],
                "hinges": [{"channel","axis","origin","sign"}, ...]}]}
A part with hinges rotates at runtime by sum(sign * channel_value) about each
axis (unit vector, body frame) through its origin -- this is how fin, control
surface, and TVC-nozzle deflections animate. Hinge signs are chosen for
VISUAL plausibility against the channel sign conventions (core/Channel.h),
not aerodynamic exactness. Kept small (a few hundred faces) so the browser
animates smoothly.
"""
import json
import math
import os


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

    # Fins: flat trapezoids around the tail, each hinged about its outward
    # radial axis so channel deflections animate. Cruciform mapping: the
    # horizontal pair carries elevator, the vertical pair rudder, all four
    # carry differential aileron (same handedness about the outward axis).
    fin_len = fin_frac * length
    fin_h = r * 2.2
    root_x0, root_x1 = xtail + fin_len, xtail
    for i in range(n_fins):
        ang = 2 * math.pi * i / n_fins
        ca, sa = math.cos(ang), math.sin(ang)
        def pt(x, rad):
            return [x, rad * ca, rad * sa]
        # Clipped delta: aft-swept leading edge, tip chord ~45% of root.
        fv = [pt(root_x0, r), pt(root_x1, r),
              pt(root_x1, r + fin_h), pt(root_x0 - fin_len * 0.55, r + fin_h)]
        ff = [[0, 1, 2], [0, 2, 3]]
        axis = [0.0, ca, sa]                       # outward radial
        origin = [(root_x0 + root_x1) / 2.0, r * ca, r * sa]
        hinges = [{"channel": "aileron", "axis": axis, "origin": origin, "sign": 1.0}]
        horizontal = abs(ca) > 0.7071
        if horizontal:
            hinges.append({"channel": "elevator", "axis": axis, "origin": origin,
                           "sign": 1.0 if ca > 0 else -1.0})
        else:
            hinges.append({"channel": "rudder", "axis": axis, "origin": origin,
                           "sign": 1.0 if sa > 0 else -1.0})
        parts.append({"name": f"fin_{i+1}", "color": fin_color,
                      "vertices": fv, "faces": ff, "hinges": hinges})

    # TVC nozzle: a short bell aft of the tail, gimbaled by the tvc channels.
    # The bell tilts opposite the thrust deflection (exhaust points where the
    # thrust does not): +tvc_pitch -> thrust +z -> bell aft end -z.
    noz_v, noz_f = [], []
    noz_len = 0.06 * length
    _tube(noz_v, noz_f, xtail, r * 0.55, xtail - noz_len, r * 0.8, 12, 0)
    origin = [xtail, 0.0, 0.0]
    parts.append({"name": "nozzle", "color": "#6b6f78",
                  "vertices": noz_v, "faces": noz_f,
                  "hinges": [
                      {"channel": "tvc_pitch", "axis": [0, 1, 0],
                       "origin": origin, "sign": -1.0},
                      {"channel": "tvc_yaw", "axis": [0, 0, 1],
                       "origin": origin, "sign": -1.0},
                  ]})
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
        # Whole-wing aileron animation: +aileron = right roll = right TE up.
        return {"name": "wing", "color": wing_color, "vertices": v, "faces": f,
                "hinges": [{"channel": "aileron", "axis": [0, 1, 0],
                            "origin": [xle - cr * 0.3, 0, 0],
                            "sign": -0.5 if sign > 0 else 0.5}]}
    parts.append(wing(+1))
    parts.append(wing(-1))

    # Horizontal stabilizer (all-moving: carries the elevator deflection,
    # +elevator = trailing edge down).
    hb = span * 0.34
    xh = tail + length * 0.14
    ch = length * 0.14
    for sign in (+1, -1):
        y = sign * hb
        v = [[xh, 0, 0], [xh - ch, 0, 0],
             [xh - ch - length * 0.05, y, 0], [xh - length * 0.04, y, 0]]
        f = [[0, 1, 2], [0, 2, 3]] if sign > 0 else [[0, 2, 1], [0, 3, 2]]
        parts.append({"name": "htail", "color": tail_color, "vertices": v,
                      "faces": f,
                      "hinges": [{"channel": "elevator", "axis": [0, 1, 0],
                                  "origin": [xh - ch * 0.3, 0, 0], "sign": 1.0}]})

    # Vertical tail (in x-z plane, up = -z; +rudder = trailing edge left).
    xv = tail + length * 0.16
    vh = length * 0.16
    v = [[xv, 0, 0], [xv - length * 0.13, 0, 0],
         [xv - length * 0.15, 0, -vh], [xv - length * 0.04, 0, -vh]]
    parts.append({"name": "vtail", "color": tail_color,
                  "vertices": v, "faces": [[0, 1, 2], [0, 2, 3]],
                  "hinges": [{"channel": "rudder", "axis": [0, 0, 1],
                              "origin": [xv - length * 0.04, 0, 0], "sign": 1.0}]})
    return {"parts": parts}


def datcom_mesh(mesh_json, vehicle_cfg):
    """Turn a committed DATCOM mesh.json (the real outer mould line + fin/canard
    plates, x from the nose tip, aft-positive) into a renderable, articulated
    mesh in the viewer's body frame (+x FORWARD, +y right, +z down).

    The only frame change is the x-axis: DATCOM measures x aft from the nose,
    the viewer wants x forward from the CG, so x_body = xcg - x_datcom (the CG
    lands at the mesh origin, which is where the vehicle rotates). Fin plates
    get hinges inferred from their own radial position (cruciform mapping:
    horizontal pair = elevator, vertical pair = rudder, all four = aileron);
    a gimballed motor adds a TVC nozzle bell driven by the tvc channels.
    Canards are fixed surfaces (the aero model gives them no channel)."""
    geom = vehicle_cfg.get("geometry", {})
    xcg = geom.get("xcg_m")
    if xcg is None:
        xcg = geom.get("length_m", 3.0) / 2.0

    def to_body(v):
        return [xcg - v[0], v[1], v[2]]

    parts = []
    tail_x = None                       # most-aft body-x, for the nozzle
    for p in mesh_json["parts"]:
        name = p["name"]
        verts = [to_body(v) for v in p["vertices"]]
        xs = [v[0] for v in verts]
        tail_x = min(xs) if tail_x is None else min(tail_x, min(xs))
        out = {"name": name, "vertices": verts, "faces": p["faces"]}

        if name == "body":
            out["color"] = "#c9ced6"
        elif name.startswith("fin_"):
            out["color"] = "#e08a3c"
            # Radial (outward) direction of this fin, from its vertex spread.
            my = sum(v[1] for v in verts) / len(verts)
            mz = sum(v[2] for v in verts) / len(verts)
            mag = math.hypot(my, mz) or 1.0
            uy, uz = my / mag, mz / mag
            axis = [0.0, uy, uz]
            # Root (near the body) x-station of the plate = its most-forward x.
            root_x = max(xs)
            r = math.hypot(my, mz)
            origin = [root_x, uy * r * 0.5, uz * r * 0.5]
            hinges = [{"channel": "aileron", "axis": axis,
                       "origin": origin, "sign": 1.0}]
            if abs(uy) > abs(uz):                    # horizontal pair -> elevator
                hinges.append({"channel": "elevator", "axis": axis,
                               "origin": origin, "sign": 1.0 if uy > 0 else -1.0})
            else:                                    # vertical pair -> rudder
                hinges.append({"channel": "rudder", "axis": axis,
                               "origin": origin, "sign": 1.0 if uz > 0 else -1.0})
            out["hinges"] = hinges
        elif name.startswith("canard_"):
            out["color"] = "#7f8794"                 # fixed forward surface
        else:
            out["color"] = "#9aa0a6"
        parts.append(out)

    # TVC bell aft of the tail for a gimballed motor.
    has_gimbal = any("gimbal" in c for c in vehicle_cfg.get("components", []))
    if has_gimbal and tail_x is not None:
        length = geom.get("length_m", 3.0)
        rad = geom.get("diameter_m", length * 0.06) / 2.0
        noz_v, noz_f = [], []
        _tube(noz_v, noz_f, tail_x, rad * 0.55, tail_x - 0.06 * length,
              rad * 0.8, 12, 0)
        origin = [tail_x, 0.0, 0.0]
        parts.append({"name": "nozzle", "color": "#6b6f78",
                      "vertices": noz_v, "faces": noz_f,
                      "hinges": [
                          {"channel": "tvc_pitch", "axis": [0, 1, 0],
                           "origin": origin, "sign": -1.0},
                          {"channel": "tvc_yaw", "axis": [0, 0, 1],
                           "origin": origin, "sign": -1.0},
                      ]})
    return {"parts": parts}


def mesh_for(vehicle_cfg, dynamics, vehicle_dir=None):
    """Choose a mesh from a loaded vehicle.json (or None for kinematic movers).

    Prefers the vehicle's committed DATCOM mesh.json (its REAL geometry) when
    vehicle_dir is given and the file exists; otherwise falls back to a
    procedural airframe from the reference dimensions."""
    if vehicle_cfg is None or dynamics == "kinematic":
        # Generic small dart for targets/traffic.
        return aircraft_mesh(span=6.0, length=8.0, body_color="#9aa0a6",
                             wing_color="#b0b4b8", tail_color="#b0b4b8")

    mesh_file = (vehicle_cfg.get("geometry", {}) or {}).get("mesh")
    if vehicle_dir and mesh_file:
        path = os.path.join(vehicle_dir, mesh_file)
        if os.path.exists(path):
            with open(path) as f:
                return datcom_mesh(json.load(f), vehicle_cfg)

    # The aero component tells us the airframe family and its references.
    aero = next((c for c in vehicle_cfg.get("components", [])
                 if c.get("type", "").endswith("_aero")), {})
    atype = aero.get("type", "rocket_table_aero")

    if atype in ("aircraft_aero", "f16_aero"):
        if atype == "f16_aero":
            # Table-aero aircraft (e.g. the F-16): dimensions aren't in the JSON.
            span, length = 9.14, 15.0
        else:
            span = aero.get("bspan_m", 10.0)
            length = max(span * 0.9, aero.get("cbar_m", 1.5) * 6.0)
        return aircraft_mesh(span=span, length=length)

    # Rocket: prefer explicit geometry, else infer from aero references.
    geom = vehicle_cfg.get("geometry", {})
    length = geom.get("length_m") or aero.get("lref_m") or aero.get("cbar_m", 3.0)
    diameter = geom.get("diameter_m") or aero.get("dref_m") or (length * 0.06)
    return rocket_mesh(length=length, diameter=diameter)
