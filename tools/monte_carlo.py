"""Monte Carlo dispersion analysis for an intercept scenario.

Runs a guided-intercept scenario many times with randomized launch aim,
target-location uncertainty, wind, and motor thrust, then aggregates the
outcome: hit probability (Pk), miss-distance statistics, CEP/R90, and the
2-D miss dispersion pattern. Writes a self-contained HTML report (scatter +
histogram) and prints a text summary.

Each run perturbs a copy of the scenario and executes `flightsim --json`, so
no sim rebuild is needed. The guided vehicle (the one with a "guidance" block)
and the intercept target are found automatically.

Usage:
    py tools/monte_carlo.py data/scenarios/sam_intercept.json --runs 200
    py tools/monte_carlo.py data/scenarios/aam_intercept.json --runs 300 \
        --sigma-aim-deg 1.5 --sigma-target-m 40 --sigma-thrust 0.04
"""
import argparse
import copy
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
EXE = os.path.join(PROJ, "build", "flightsim.exe")
if not os.path.exists(EXE):
    EXE = os.path.join(PROJ, "build", "flightsim")


def strip_comments(text):
    out = []
    for line in text.splitlines():
        q = False
        for i, ch in enumerate(line):
            if ch == '"':
                q = not q
            elif ch == '/' and i + 1 < len(line) and line[i + 1] == '/' and not q:
                line = line[:i]
                break
        out.append(line)
    return "\n".join(out)


def load_jsonc(path):
    with open(path) as f:
        return json.loads(strip_comments(f.read()))


def inline_vehicle(entry, scen_dir):
    """Replace an entry's "vehicle" file reference with an inline definition
    whose data-file paths are absolute, so a perturbed copy loads correctly
    from a temp dir (and its thrust can be scaled)."""
    if "vehicle" not in entry:
        return entry.get("definition")
    vpath = entry["vehicle"]
    vpath = vpath if os.path.isabs(vpath) else os.path.join(scen_dir, vpath)
    vpath = os.path.abspath(vpath)
    vdir = os.path.dirname(vpath)
    veh = load_jsonc(vpath)
    # Absolutize relative data paths.
    blocks = list(veh.get("components", []))
    blocks.append(veh.get("gnc", {}).get("control_law", {}))
    blocks.append(veh.get("mass", {}))
    for blk in blocks:
        for key in ("tables_csv", "control_csv", "schedule", "table", "dir"):
            if key in blk and isinstance(blk[key], str) and not os.path.isabs(blk[key]):
                blk[key] = os.path.join(vdir, blk[key])
    return veh


def perturb(base, pursuer_name, target_name, rng, s):
    """Return a perturbed deep copy of the scenario for one run."""
    scn = copy.deepcopy(base)
    for e in scn["vehicles"]:
        if e["name"] == pursuer_name:
            init = e["initial"]
            eul = init.get("euler_deg", [0, 0, 0])
            init["euler_deg"] = [eul[0] + rng.normal(0, s["aim"]),
                                 eul[1] + rng.normal(0, s["aim"]),
                                 eul[2] + rng.normal(0, s["aim"])]
            v = init.get("velocity_ned_ms", [0, 0, 0])
            init["velocity_ned_ms"] = [c * (1 + rng.normal(0, s["vel"])) for c in v]
            # Motor thrust dispersion (scale the whole curve).
            if "definition" in e:
                for comp in e["definition"].get("components", []):
                    if "thrust_curve" in comp:
                        k = 1 + rng.normal(0, s["thrust"])
                        comp["thrust_curve"] = [[t, f * k]
                                                for t, f in comp["thrust_curve"]]
        if e["name"] == target_name:
            init = e["initial"]
            p = init.get("position_ned_m", [0, 0, 0])
            init["position_ned_m"] = [p[i] + rng.normal(0, s["target"]) for i in range(3)]
    # Wind dispersion (replace environment wind with a perturbed constant).
    env = scn.setdefault("environment", {})
    env["wind"] = {"type": "constant",
                   "ned_ms": [rng.normal(0, s["wind"]), rng.normal(0, s["wind"]), 0.0]}
    return scn


def run_one(scn, tmpdir, idx):
    path = os.path.join(tmpdir, f"run_{idx}.json")
    # Disable per-run CSV logging to avoid thousands of files.
    for e in scn["vehicles"]:
        e.pop("log", None)
    with open(path, "w") as f:
        json.dump(scn, f)
    r = subprocess.run([EXE, path, "--json"], capture_output=True, text=True)
    try:
        return json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:
        return {"error": r.stdout + r.stderr}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario")
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--sigma-aim-deg", type=float, default=1.5,
                    help="launch attitude 1-sigma per axis [deg]")
    ap.add_argument("--sigma-vel", type=float, default=0.02,
                    help="launch speed 1-sigma [fraction]")
    ap.add_argument("--sigma-target-m", type=float, default=40.0,
                    help="target-location 1-sigma per axis [m]")
    ap.add_argument("--sigma-wind", type=float, default=4.0,
                    help="wind 1-sigma per component [m/s]")
    ap.add_argument("--sigma-thrust", type=float, default=0.04,
                    help="motor total-impulse 1-sigma [fraction]")
    ap.add_argument("--out", help="output HTML (default output/<scenario>/monte_carlo.html)")
    args = ap.parse_args()

    base = load_jsonc(args.scenario)
    scen_dir = os.path.dirname(os.path.abspath(args.scenario))
    name = base.get("name", "scenario")

    ic = base.get("end_conditions", {}).get("intercept")
    if not ic:
        sys.exit("monte_carlo: scenario has no end_conditions.intercept")
    pursuer, target = ic["pursuer"], ic["target"]
    hit_radius = ic.get("hit_radius_m", 8.0)

    # Inline the pursuer's vehicle so thrust can be dispersed and paths absolutized.
    for e in base["vehicles"]:
        if e["name"] == pursuer:
            veh = inline_vehicle(e, scen_dir)
            if veh is not None:
                e["definition"] = veh
                e.pop("vehicle", None)

    sig = {"aim": args.sigma_aim_deg, "vel": args.sigma_vel,
           "target": args.sigma_target_m, "wind": args.sigma_wind,
           "thrust": args.sigma_thrust}
    rng = np.random.default_rng(args.seed)

    print(f"Monte Carlo: {name}  ({args.runs} runs)")
    print(f"  dispersions: aim {sig['aim']}deg, speed {sig['vel']*100:.0f}%, "
          f"target {sig['target']}m, wind {sig['wind']}m/s, thrust {sig['thrust']*100:.0f}%")

    hits, misses, miss_vecs = 0, [], []
    errors = 0
    with tempfile.TemporaryDirectory() as tmp:
        for i in range(args.runs):
            scn = perturb(base, pursuer, target, rng, sig)
            res = run_one(scn, tmp, i)
            if "error" in res or not res.get("intercept"):
                errors += 1
                continue
            misses.append(res["miss_m"])
            miss_vecs.append(res["miss_ned"])
            if res["hit"]:
                hits += 1
            if (i + 1) % max(1, args.runs // 10) == 0:
                print(f"  {i+1}/{args.runs} ...", flush=True)

    n = len(misses)
    if n == 0:
        sys.exit("all runs failed -- check the scenario runs standalone first")
    misses = np.array(misses)
    V = np.array(miss_vecs)                    # miss vectors (target - pursuer), NED
    pk = hits / n

    # 2-D miss in the plane most transverse to the run: use East & Up.
    cross = V[:, 1]                            # East [m]
    vert = -V[:, 2]                            # Up [m]
    radial = np.hypot(cross, vert)
    cep = np.percentile(radial, 50)
    r90 = np.percentile(radial, 90)

    print(f"\n  runs analyzed: {n}  (errors {errors})")
    print(f"  Pk (hit, <= {hit_radius:.0f} m): {pk*100:.1f}%")
    print(f"  miss distance: mean {misses.mean():.1f} m, median {np.median(misses):.1f} m, "
          f"min {misses.min():.1f}, max {misses.max():.1f}")
    print(f"  CEP {cep:.1f} m, R90 {r90:.1f} m")

    out = args.out or os.path.join(PROJ, "output", name, "monte_carlo.html")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(html_report(name, args, sig, n, pk, hit_radius, misses,
                            cross, vert, cep, r90))
    print(f"  wrote {out}")


def html_report(name, args, sig, n, pk, hit_radius, misses, cross, vert, cep, r90):
    scatter = scatter_svg(cross, vert, hit_radius, cep, r90)
    hist = hist_svg(misses, hit_radius)
    return f"""<!doctype html><html><head><meta charset="utf-8">
<title>Monte Carlo: {name}</title>
<style>body{{margin:0;background:#010409;color:#c9d1d9;font:13px system-ui}}
h1{{padding:12px 16px;margin:0;font-size:15px;background:#161b22;border-bottom:1px solid #30363d}}
.wrap{{display:flex;flex-wrap:wrap;gap:16px;padding:16px}}
.card{{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:14px}}
table{{border-collapse:collapse}} td{{padding:3px 12px 3px 0}} .k{{color:#8b949e}}
.big{{font-size:22px;color:#58a6ff;font-variant-numeric:tabular-nums}}</style></head>
<body><h1>flightsim2 &middot; Monte Carlo dispersion &middot; {name}</h1>
<div class="wrap">
  <div class="card"><div class="k">hit probability (Pk)</div>
    <div class="big">{pk*100:.1f}%</div>
    <table>
      <tr><td class="k">runs</td><td>{n}</td></tr>
      <tr><td class="k">hit radius</td><td>{hit_radius:.0f} m</td></tr>
      <tr><td class="k">CEP (50%)</td><td>{cep:.1f} m</td></tr>
      <tr><td class="k">R90</td><td>{r90:.1f} m</td></tr>
      <tr><td class="k">miss mean</td><td>{misses.mean():.1f} m</td></tr>
      <tr><td class="k">miss median</td><td>{np.median(misses):.1f} m</td></tr>
      <tr><td class="k">miss max</td><td>{misses.max():.1f} m</td></tr>
    </table>
    <div class="k" style="margin-top:10px">dispersions (1&sigma;)</div>
    <table>
      <tr><td class="k">aim</td><td>{sig['aim']}&deg;</td></tr>
      <tr><td class="k">launch speed</td><td>{sig['vel']*100:.0f}%</td></tr>
      <tr><td class="k">target loc</td><td>{sig['target']:.0f} m</td></tr>
      <tr><td class="k">wind</td><td>{sig['wind']:.0f} m/s</td></tr>
      <tr><td class="k">thrust</td><td>{sig['thrust']*100:.0f}%</td></tr>
    </table>
  </div>
  <div class="card"><div class="k">miss dispersion (cross-range vs vertical, target at origin)</div>
    {scatter}</div>
  <div class="card"><div class="k">miss-distance histogram</div>
    {hist}</div>
</div></body></html>"""


def scatter_svg(cross, vert, hit_radius, cep, r90):
    W = H = 360
    m = max(np.abs(cross).max(), np.abs(vert).max(), r90) * 1.15 + 1
    def X(v): return W / 2 + v / m * (W / 2 - 20)
    def Y(v): return H / 2 - v / m * (H / 2 - 20)
    pts = "".join(f'<circle cx="{X(cross[i]):.1f}" cy="{Y(vert[i]):.1f}" r="2.2" '
                  f'fill="#58a6ff" opacity="0.55"/>' for i in range(len(cross)))
    rings = ""
    for rad, col, lab in [(hit_radius, "#56d364", "hit"), (cep, "#e3b341", "CEP"),
                          (r90, "#f97583", "R90")]:
        rr = rad / m * (W / 2 - 20)
        rings += (f'<circle cx="{W/2}" cy="{H/2}" r="{rr:.1f}" fill="none" '
                  f'stroke="{col}" stroke-dasharray="3 3" opacity="0.8"/>'
                  f'<text x="{W/2+rr+2:.1f}" y="{H/2:.1f}" fill="{col}" font-size="10">'
                  f'{lab} {rad:.0f}m</text>')
    return f'''<svg viewBox="0 0 {W} {H}" style="width:360px;height:360px">
      <rect width="{W}" height="{H}" fill="#05070c"/>
      <line x1="{W/2}" y1="0" x2="{W/2}" y2="{H}" stroke="#21262d"/>
      <line x1="0" y1="{H/2}" x2="{W}" y2="{H/2}" stroke="#21262d"/>
      {pts}{rings}
      <circle cx="{W/2}" cy="{H/2}" r="3" fill="#fff"/>
    </svg>'''


def hist_svg(misses, hit_radius):
    W, H, PAD = 360, 220, 30
    counts, edges = np.histogram(misses, bins=24)
    bw = (W - 2 * PAD) / len(counts)
    mx = counts.max() or 1
    bars = ""
    for i, c in enumerate(counts):
        h = c / mx * (H - 2 * PAD)
        x = PAD + i * bw
        col = "#56d364" if edges[i] <= hit_radius else "#58a6ff"
        bars += f'<rect x="{x:.1f}" y="{H-PAD-h:.1f}" width="{bw-1:.1f}" height="{h:.1f}" fill="{col}"/>'
    hx = PAD + (hit_radius - edges[0]) / (edges[-1] - edges[0]) * (W - 2 * PAD)
    return f'''<svg viewBox="0 0 {W} {H}" style="width:360px;height:220px">
      <rect width="{W}" height="{H}" fill="#05070c"/>
      {bars}
      <line x1="{hx:.1f}" y1="{PAD}" x2="{hx:.1f}" y2="{H-PAD}" stroke="#56d364" stroke-dasharray="3 3"/>
      <text x="{PAD}" y="{H-8}" fill="#8b949e" font-size="10">{edges[0]:.0f} m</text>
      <text x="{W-PAD}" y="{H-8}" fill="#8b949e" font-size="10" text-anchor="end">{edges[-1]:.0f} m</text>
      <text x="{PAD}" y="16" fill="#8b949e" font-size="10">miss distance (green = hit)</text>
    </svg>'''


if __name__ == "__main__":
    main()
