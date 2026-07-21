"""Analyze a vehicle's motion from a scenario CSV log: control-tracking
quality (rise time, overshoot, settling, steady-state error), oscillation /
limit-cycle detection (dominant frequency + decay), and flight-envelope
summary. Prints a text report and, with --html, writes time-history plots.

Reads the setpoint columns (pitch_sp, heading_sp, altitude_sp, speed_sp) the
CsvLogger emits, so it can measure the tracked channels directly.

Usage:
    py tools/analyze.py data/output/lqr_rocket_launch/lqr-rocket.csv
    py tools/analyze.py data/output/f16_cruise/viper.csv --html
"""
import argparse
import math
import os
import sys

import numpy as np

R2D = 180.0 / math.pi

# (measured column, setpoint column, label, is_angle)
CHANNELS = [
    ("theta", "pitch_sp", "pitch", True),
    ("psi", "heading_sp", "heading", True),
    ("altitude", "altitude_sp", "altitude", False),
    ("airspeed", "speed_sp", "speed", False),
]


def load(path):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    return rows


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def step_metrics(t, y, target, is_angle):
    """Rise/overshoot/settling of y tracking a (piecewise-constant) target.
    Analyzes the last commanded segment so multi-step plans report the final
    capture. Returns a dict or None if the channel is never commanded."""
    cmd = np.asarray(target, float)
    valid = ~np.isnan(cmd)
    if not valid.any():
        return None

    # Find the last change in the command -> the final step to analyze.
    idx = np.where(valid)[0]
    seg_start = idx[0]
    for k in idx[1:]:
        if abs(cmd[k] - cmd[k - 1]) > (1e-3 if not is_angle else math.radians(0.2)):
            seg_start = k
    end = idx[-1]
    sl = slice(seg_start, end + 1)
    tt, yy = t[sl], y[sl]
    c = cmd[seg_start]
    y0 = yy[0]

    err = wrap_pi(yy - c) if is_angle else (yy - c)
    step = (c - y0)
    if abs(step) < (math.radians(0.5) if is_angle else 1e-6):
        # Regulation (no real step): report steady-state error + wander only.
        ss = err[int(0.8 * len(err)):]
        return {"kind": "hold", "sse": _fmt(np.mean(ss), is_angle),
                "wander": _fmt(np.std(err), is_angle)}

    # Normalized response 0->1.
    resp = (yy - y0) / step
    over = (np.max(resp) - 1.0) * 100.0 if step > 0 else (1.0 - np.min(resp)) * 100.0
    over = max(over, 0.0)

    def cross(level):
        w = np.where(resp >= level)[0]
        return tt[w[0]] - tt[0] if len(w) else float("nan")
    rise = cross(0.9)

    # Settling to +/-2% of the step.
    tol = 0.02
    settled = float("nan")
    for i in range(len(resp)):
        if np.all(np.abs(resp[i:] - 1.0) <= tol):
            settled = tt[i] - tt[0]
            break
    sse = err[-1]
    return {"kind": "step", "step": _fmt(step, is_angle),
            "rise_s": rise, "overshoot_pct": over,
            "settling_s": settled, "sse": _fmt(sse, is_angle)}


def _fmt(v, is_angle):
    return v * R2D if is_angle else v


def oscillation(t, sig):
    """Dominant oscillation frequency and whether it grows/decays, from the
    detrended signal via FFT + envelope slope."""
    if len(t) < 32:
        return None
    dt = np.median(np.diff(t))
    x = sig - np.polyval(np.polyfit(t, sig, 3), t)   # remove slow trend
    if np.std(x) < 1e-9:
        return {"amp": 0.0, "freq_hz": 0.0, "trend": "flat"}
    win = np.hanning(len(x))
    sp = np.abs(np.fft.rfft(x * win))
    fr = np.fft.rfftfreq(len(x), dt)
    k = np.argmax(sp[1:]) + 1
    fpk = fr[k]
    # Envelope trend: compare RMS of first vs last third.
    a = np.std(x[: len(x) // 3])
    b = np.std(x[-len(x) // 3:])
    trend = "growing" if b > 1.3 * a else "decaying" if b < 0.7 * a else "sustained"
    return {"amp": float(np.std(x)), "freq_hz": float(fpk), "trend": trend}


def report(path, html, tmin=None, tmax=None):
    d = load(path)
    t = d["time"]
    if tmin is not None or tmax is not None:
        lo = tmin if tmin is not None else t[0]
        hi = tmax if tmax is not None else t[-1]
        keep = (t >= lo) & (t <= hi)
        d = d[keep]
        t = d["time"]
    name = os.path.splitext(os.path.basename(path))[0]
    print(f"\n=== motion analysis: {name} ===")
    print(f"  {len(t)} samples, {t[0]:.2f}..{t[-1]:.2f} s")
    print(f"  peak speed {np.nanmax(d['airspeed']):.1f} m/s, "
          f"peak Mach {np.nanmax(d['mach']):.2f}, "
          f"apogee {np.nanmax(d['altitude']):.0f} m, "
          f"max |alpha| {np.nanmax(np.abs(d['alpha']))*R2D:.1f} deg")

    for meas, spc, label, is_ang in CHANNELS:
        if meas not in d.dtype.names or spc not in d.dtype.names:
            continue
        m = step_metrics(t, d[meas], d[spc], is_ang)
        if m is None:
            continue
        unit = "deg" if is_ang else ("m" if label in ("altitude",) else
                                     "m/s" if label == "speed" else "")
        if m["kind"] == "step":
            print(f"  [{label}] step {m['step']:+.1f}{unit}: "
                  f"rise(90%) {m['rise_s']:.2f}s, overshoot {m['overshoot_pct']:.0f}%, "
                  f"settle(2%) {m['settling_s']:.2f}s, ss-err {m['sse']:+.2f}{unit}")
        else:
            print(f"  [{label}] hold: ss-err {m['sse']:+.2f}{unit}, "
                  f"wander {m['wander']:.2f}{unit}")

    # Oscillation on the control surface + pitch rate (limit-cycle / wiggle).
    for sig, lab, ang in [("elevator", "elevator", True), ("q", "pitch-rate", True),
                          ("aileron", "aileron", True)]:
        if sig not in d.dtype.names:
            continue
        osc = oscillation(t, d[sig])
        if osc and osc["amp"] > (math.radians(0.3) if ang else 0.1):
            amp = osc["amp"] * (R2D if ang else 1.0)
            print(f"  [{lab}] oscillation ~{osc['freq_hz']:.2f} Hz, "
                  f"amp {amp:.2f}{'deg' if ang else ''} ({osc['trend']})")

    if html:
        write_html(name, d, path)


def write_html(name, d, srcpath):
    """Small self-contained SVG time-history dashboard (no libraries)."""
    t = d["time"]
    panels = [
        ("altitude vs alt_sp", [("altitude", "#58a6ff", 1), ("altitude_sp", "#f0883e", 1)], "m"),
        ("pitch vs pitch_sp", [("theta", "#58a6ff", R2D), ("pitch_sp", "#f0883e", R2D)], "deg"),
        ("heading vs heading_sp", [("psi", "#58a6ff", R2D), ("heading_sp", "#f0883e", R2D)], "deg"),
        ("controls", [("elevator", "#56d364", R2D), ("aileron", "#bc8cff", R2D),
                      ("rudder", "#f97583", R2D)], "deg"),
        ("alpha / beta", [("alpha", "#58a6ff", R2D), ("beta", "#f0883e", R2D)], "deg"),
        ("speed / mach", [("airspeed", "#58a6ff", 1)], "m/s"),
    ]
    svgs = []
    for title, series, unit in panels:
        svgs.append(svg_panel(title, t, d, series, unit))
    html = HTML.replace("__TITLE__", name).replace("__PANELS__", "\n".join(svgs))
    out = os.path.join(os.path.dirname(srcpath), "analysis.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"  wrote {out}")


def svg_panel(title, t, d, series, unit):
    W, H, PAD = 460, 150, 30
    lines, allv = [], []
    for col, color, scale in series:
        if col not in d.dtype.names:
            continue
        y = d[col] * scale
        allv.append(y[~np.isnan(y)])
    if not allv:
        return ""
    vmin = min(v.min() for v in allv if len(v))
    vmax = max(v.max() for v in allv if len(v))
    if vmax - vmin < 1e-6:
        vmax += 1
    tmin, tmax = t[0], t[-1]

    def X(tt):
        return PAD + (tt - tmin) / (tmax - tmin) * (W - 2 * PAD)

    def Y(vv):
        return H - PAD - (vv - vmin) / (vmax - vmin) * (H - 2 * PAD)

    paths = []
    for col, color, scale in series:
        if col not in d.dtype.names:
            continue
        y = d[col] * scale
        pts, cur = [], []
        for i in range(len(t)):
            if np.isnan(y[i]):
                if cur:
                    pts.append(cur); cur = []
                continue
            cur.append(f"{X(t[i]):.1f},{Y(y[i]):.1f}")
        if cur:
            pts.append(cur)
        dash = ' stroke-dasharray="4 3"' if col.endswith("_sp") else ""
        for seg in pts:
            paths.append(f'<polyline points="{" ".join(seg)}" fill="none" '
                         f'stroke="{color}" stroke-width="1.5"{dash}/>')
    legend = " ".join(f'<tspan fill="{c}">{col}</tspan>' for col, c, _ in series
                      if col in d.dtype.names)
    return f'''<div class="panel"><svg viewBox="0 0 {W} {H}">
      <rect x="0" y="0" width="{W}" height="{H}" fill="#0d1117"/>
      <text x="{PAD}" y="16" fill="#c9d1d9" font-size="12">{title} [{unit}]</text>
      <text x="{PAD}" y="{H-6}" font-size="10">{legend}</text>
      <text x="{W-PAD}" y="16" fill="#6a737d" font-size="9" text-anchor="end">{vmax:.0f}</text>
      <text x="{W-PAD}" y="{H-PAD}" fill="#6a737d" font-size="9" text-anchor="end">{vmin:.0f}</text>
      {"".join(paths)}
    </svg></div>'''


HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>analysis: __TITLE__</title>
<style>body{margin:0;background:#010409;color:#c9d1d9;font:13px system-ui}
h1{padding:12px 16px;margin:0;font-size:15px;background:#161b22;border-bottom:1px solid #30363d}
.grid{display:flex;flex-wrap:wrap;gap:8px;padding:12px}
.panel{background:#0d1117;border:1px solid #30363d;border-radius:6px}
svg{display:block;width:460px;height:150px}</style></head>
<body><h1>flightsim2 &middot; motion analysis &middot; __TITLE__</h1>
<div class="grid">__PANELS__</div></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="scenario CSV log (data/output/<scenario>/<vehicle>.csv)")
    ap.add_argument("--html", action="store_true", help="also write analysis.html")
    ap.add_argument("--tmin", type=float, help="restrict analysis to t >= tmin [s]")
    ap.add_argument("--tmax", type=float, help="restrict analysis to t <= tmax [s] "
                    "(e.g. the powered/controlled phase before ballistic descent)")
    args = ap.parse_args()
    if not os.path.exists(args.log):
        sys.exit(f"no such log: {args.log}")
    report(args.log, args.html, args.tmin, args.tmax)


if __name__ == "__main__":
    main()
