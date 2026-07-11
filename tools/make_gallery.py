"""Build the committed portfolio gallery: run each curated scenario, render
its interactive viewer into docs/gallery/, and write an index page.

    py tools/make_gallery.py            # all curated scenarios
    py tools/make_gallery.py hybrid_launch intercept   # a subset

Requires a built sim (build/flightsim). The gallery is self-contained: clone
the repo, open docs/gallery/index.html -- or serve the folder with GitHub
Pages for a live portfolio link.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
GALLERY = os.path.join(PROJ, "docs", "gallery")
SIM = os.path.join(PROJ, "build", "flightsim")

# (scenario stem, card title, card description)
CURATED = [
    ("hybrid_launch", "Hybrid TVC+fin launcher",
     "One allocation-based control law flies a gimbaled nozzle AND fins: "
     "TVC steers off the pad, authority blends to the fins as dynamic "
     "pressure builds, and the fins track a post-burnout pitch command "
     "alone. Open the Allocation preset to watch the moment hand-off."),
    ("intercept", "Missile intercept",
     "An agile interceptor under proportional navigation chases a turning "
     "drone to a hit. Chase camera + LOS overlay show the closing geometry; "
     "the Intercept preset plots range and closing speed to the endgame."),
    ("aam_intercept", "Air-to-air missile",
     "Full-pipeline AAM -- DATCOM aerodynamics, auto-designed LQR gain "
     "schedule, ProNav guidance -- launched from a carrier aircraft against "
     "a maneuvering bandit."),
    ("tvc_launch", "TVC rocket launch",
     "A finless rocket steered purely by thrust vectoring through its "
     "gravity-turn pitch program; after burnout the gimbal loses authority "
     "and it coasts ballistically. Watch the nozzle bell work."),
    ("f16_turn", "F-16 turn",
     "The real Stevens & Lewis / NASA wind-tunnel F-16 model (statically "
     "unstable airframe + SAS autopilot) flying a commanded heading "
     "change."),
    ("datcom_rocket_launch", "DATCOM sounding rocket",
     "A sounding rocket whose aerodynamic tables come straight from a "
     "Missile DATCOM run, flown by a hand-tuned PID through a pitch "
     "program to apogee."),
]

INDEX_TEMPLATE = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>flightsim2 gallery</title>
<style>
body{margin:0;background:#0b0e13;color:#cfd6e1;
  font:15px/1.55 system-ui,Segoe UI,sans-serif;padding:40px 20px}
.wrap{max-width:980px;margin:0 auto}
h1{color:#e8edf5;font-size:26px;margin:0 0 4px}
.sub{color:#7d8ca3;margin-bottom:28px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(290px,1fr));gap:16px}
a.card{display:block;background:#11161f;border:1px solid #1e2633;border-radius:10px;
  padding:18px 18px 14px;text-decoration:none;color:inherit;transition:border-color .15s}
a.card:hover{border-color:#3a5f9e}
.card h2{color:#e8edf5;font-size:17px;margin:0 0 8px}
.card p{color:#9fb2cc;font-size:13px;margin:0 0 10px}
.card .open{color:#4da3ff;font-size:13px}
.foot{margin-top:30px;color:#5d6c82;font-size:13px}
code{background:#161d29;border-radius:4px;padding:1px 6px}
</style></head><body><div class="wrap">
<h1>flightsim2 &mdash; interactive flight gallery</h1>
<div class="sub">6-DOF scenario-driven flight simulator (C++17, no external
dependencies) with component-based vehicles, control allocation, and
auto-designed autopilots. Every viewer below is a single offline HTML file:
articulated 3-D animation, toggleable overlays, and synced telemetry plots.</div>
<div class="grid">
CARDS
</div>
<div class="foot">Regenerate with <code>py tools/make_gallery.py</code> after
building the sim. Source: the flightsim2 repository.</div>
</div></body></html>
"""

CARD = """<a class="card" href="{stem}.html"><h2>{title}</h2><p>{desc}</p>
<span class="open">open viewer &rarr;</span></a>"""


def main():
    picks = sys.argv[1:] or [c[0] for c in CURATED]
    os.makedirs(GALLERY, exist_ok=True)

    cards = []
    for stem, title, desc in CURATED:
        if stem not in picks:
            continue
        scenario = os.path.join(PROJ, "scenarios", stem + ".json")
        print(f"== {stem}")
        r = subprocess.run([SIM, scenario], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"flightsim failed for {stem}:\n{r.stdout}{r.stderr}")
        out = os.path.join(GALLERY, stem + ".html")
        r = subprocess.run([sys.executable, os.path.join(HERE, "visualize.py"),
                            scenario, "--out", out], capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"visualize failed for {stem}:\n{r.stdout}{r.stderr}")
        print("   " + r.stdout.strip().splitlines()[0])
    for stem, title, desc in CURATED:
        if stem in picks and os.path.exists(os.path.join(GALLERY, stem + ".html")):
            cards.append(CARD.format(stem=stem, title=title, desc=desc))

    with open(os.path.join(GALLERY, "index.html"), "w", encoding="utf-8") as f:
        f.write(INDEX_TEMPLATE.replace("CARDS", "\n".join(cards)))
    print(f"wrote {os.path.join(GALLERY, 'index.html')}  ({len(cards)} cards)")


if __name__ == "__main__":
    main()
