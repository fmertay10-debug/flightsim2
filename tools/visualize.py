"""Render a flightsim2 scenario's CSV logs to a self-contained interactive
3-D viewer (single HTML file, works offline in any browser).

The viewer (three.js + uPlot, vendored in tools/vendor/, inlined at build
time) shows every logged vehicle flown as its articulated 3-D model -- fins,
control surfaces, and TVC nozzle deflect with the logged channel values --
with playback controls, focus-vehicle cameras, toggleable overlays, and
preset plots of every logged/derived quantity.

Usage:
    py tools/visualize.py scenarios/intercept.json
    py tools/visualize.py scenarios/f16_turn.json --out output/f16.html
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
VENDOR = os.path.join(HERE, "vendor")
sys.path.insert(0, HERE)

from meshes import mesh_for  # noqa: E402

G0 = 9.80665
NFRAMES = 600          # animation timeline cap (plots keep full resolution)
STD_CHANNELS = ["elevator", "aileron", "rudder", "throttle", "tvc_pitch", "tvc_yaw"]


def strip_comments(text):
    """Remove // line comments so the sim's JSONC scenario files parse."""
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


def rnd(v):
    """6 significant digits -- shrinks the embedded payload substantially."""
    if v == 0.0 or v != v or v in (float("inf"), float("-inf")):
        return v
    return float(f"{v:.6g}")


def read_log(path):
    """Every CSV column as a float list, keyed by header name."""
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"empty log: {path}")
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


# ------------------------------------------------------------ derived series

def add_derived(cols):
    """Quantities computable from the log alone."""
    n = len(cols["time"])
    t, vx, vy, vz = cols["time"], cols["vx"], cols["vy"], cols["vz"]
    cols["vground"] = [math.hypot(vx[i], vy[i]) for i in range(n)]
    cols["vspeed"] = [-vz[i] for i in range(n)]
    cols["gamma"] = [math.atan2(-vz[i], max(cols["vground"][i], 1e-9))
                     for i in range(n)]
    # Load factor: |dv/dt - g| / g0 (specific force magnitude, NED gravity).
    gload = [1.0] * n
    for i in range(1, n - 1):
        dt = t[i + 1] - t[i - 1]
        if dt <= 0:
            continue
        ax = (vx[i + 1] - vx[i - 1]) / dt
        ay = (vy[i + 1] - vy[i - 1]) / dt
        az = (vz[i + 1] - vz[i - 1]) / dt - G0
        gload[i] = math.sqrt(ax * ax + ay * ay + az * az) / G0
    if n > 2:
        gload[0], gload[-1] = gload[1], gload[-2]
    cols["gload"] = gload


def sample_at(cols, key, tt):
    """Nearest-sample a column of another vehicle's log at time tt."""
    src = cols["time"]
    lo, hi = 0, len(src) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if src[mid] < tt:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(src[lo - 1] - tt) < abs(src[lo] - tt):
        lo -= 1
    return cols[key][lo]


def add_relative(cols, target_cols):
    """Range / closing speed / LOS angles from this vehicle to its target."""
    n = len(cols["time"])
    rng, closing, losaz, losel = [], [], [], []
    for i in range(n):
        tt = cols["time"][i]
        dx = sample_at(target_cols, "x", tt) - cols["x"][i]
        dy = sample_at(target_cols, "y", tt) - cols["y"][i]
        dz = sample_at(target_cols, "z", tt) - cols["z"][i]
        r = math.sqrt(dx * dx + dy * dy + dz * dz)
        rvx = sample_at(target_cols, "vx", tt) - cols["vx"][i]
        rvy = sample_at(target_cols, "vy", tt) - cols["vy"][i]
        rvz = sample_at(target_cols, "vz", tt) - cols["vz"][i]
        rng.append(r)
        closing.append(-(dx * rvx + dy * rvy + dz * rvz) / max(r, 1e-6))
        losaz.append(math.atan2(dy, dx))
        losel.append(math.atan2(-dz, math.hypot(dx, dy)))
    cols["range"] = rng
    cols["closing"] = closing
    cols["los_az"] = losaz
    cols["los_el"] = losel


# ------------------------------------------------------------ payload build

def resample_frames(cols, times, comp_prefixes):
    """Nearest-sample the animation-rate track from the full log."""
    src = cols["time"]
    idx, j = [], 0
    for tt in times:
        while j + 1 < len(src) and src[j + 1] <= tt:
            j += 1
        idx.append(j)
    pick = lambda key: [rnd(cols[key][k]) for k in idx] if key in cols else None
    aero = next((p for p in comp_prefixes if p.endswith("_aero")), None)

    frames = {
        "pos": [[rnd(cols["x"][k]), rnd(cols["y"][k]), rnd(cols["z"][k])] for k in idx],
        "eul": [[rnd(cols["phi"][k]), rnd(cols["theta"][k]), rnd(cols["psi"][k])] for k in idx],
        "vel": [[rnd(cols["vx"][k]), rnd(cols["vy"][k]), rnd(cols["vz"][k])] for k in idx],
        "alive": [1 if src[0] <= tt <= src[-1] + 1e-9 else 0 for tt in times],
        "chan": {c: pick(c) for c in STD_CHANNELS if c in cols},
        "thrust": pick("thrust"),
        "alpha": pick("alpha"), "beta": pick("beta"), "mach": pick("mach"),
        "spPitch": pick("pitch_sp"), "spHeading": pick("heading_sp"),
    }
    if aero:
        frames["aeroF"] = [[rnd(cols[aero + "_fx"][k]), rnd(cols[aero + "_fy"][k]),
                            rnd(cols[aero + "_fz"][k])] for k in idx]
    return frames


def component_prefixes(cols):
    """Component labels recovered from the *_mx wrench columns."""
    return [c[:-3] for c in cols if c.endswith("_mx")]


def build(scenario_path):
    scn = load_jsonc(scenario_path)
    scn_dir = os.path.dirname(os.path.abspath(scenario_path))

    # Guidance / intercept pairings drive the LOS overlay + intercept preset.
    targets = {e["name"]: e["guidance"]["target"]
               for e in scn["vehicles"] if "guidance" in e}
    ic = scn.get("end_conditions", {}).get("intercept")
    if ic:
        targets.setdefault(ic["pursuer"], ic["target"])

    logs, cfgs = {}, {}
    for entry in scn["vehicles"]:
        if "log" not in entry:
            continue
        log_path = entry["log"] if os.path.isabs(entry["log"]) \
            else os.path.join(PROJ, entry["log"])
        if not os.path.exists(log_path):
            print(f"  (skipping {entry['name']}: no log at {log_path})")
            continue
        cfg = None
        if "definition" in entry:
            cfg = entry["definition"]
        elif "vehicle" in entry:
            vpath = entry["vehicle"] if os.path.isabs(entry["vehicle"]) \
                else os.path.join(scn_dir, entry["vehicle"])
            if os.path.exists(vpath):
                cfg = load_jsonc(vpath)
        logs[entry["name"]] = read_log(log_path)
        cfgs[entry["name"]] = (cfg, entry.get("dynamics", "six_dof"))

    if not logs:
        raise SystemExit("no vehicle logs found -- run the scenario first")

    for name, cols in logs.items():
        add_derived(cols)
        tgt = targets.get(name)
        if tgt in logs:
            add_relative(cols, logs[tgt])

    max_t = max(c["time"][-1] for c in logs.values())
    times = [rnd(max_t * k / (NFRAMES - 1)) for k in range(NFRAMES)]

    vehicles = []
    for name, cols in logs.items():
        cfg, dynamics = cfgs[name]
        comps = component_prefixes(cols)
        vehicles.append({
            "name": name,
            "target": targets.get(name),
            "components": comps,
            "mesh": mesh_for(cfg, dynamics),
            "frames": resample_frames(cols, times, comps),
            "series": {k: [rnd(v) for v in vals] for k, vals in cols.items()},
        })

    return {"name": scn.get("name", "scenario"), "times": times,
            "vehicles": vehicles}


def render_html(data):
    vendor = {}
    for key, fname in (("THREE_JS", "three.min.js"),
                       ("ORBIT_JS", "OrbitControls.js"),
                       ("UPLOT_JS", "uPlot.iife.min.js"),
                       ("UPLOT_CSS", "uPlot.min.css")):
        with open(os.path.join(VENDOR, fname), encoding="utf-8") as f:
            vendor[key] = f.read()
    html = HTML_TEMPLATE
    html = html.replace("/*UPLOT_CSS*/", vendor["UPLOT_CSS"])
    html = html.replace("/*THREE_JS*/", vendor["THREE_JS"])
    html = html.replace("/*ORBIT_JS*/", vendor["ORBIT_JS"])
    html = html.replace("/*UPLOT_JS*/", vendor["UPLOT_JS"])
    payload = json.dumps(data, separators=(",", ":"))
    return html.replace("/*DATA*/", payload)


# ---------------------------------------------------------------- HTML/JS

HTML_TEMPLATE = r"""<!doctype html>
<html><head><meta charset="utf-8">
<title>flightsim2 viewer</title>
<style>
/*UPLOT_CSS*/
html,body{margin:0;height:100%;overflow:hidden;background:#0b0e13;color:#cfd6e1;
  font:13px/1.4 system-ui,Segoe UI,sans-serif}
#scene{position:absolute;inset:0}
#topbar{position:absolute;top:0;left:0;right:0;height:44px;display:flex;
  align-items:center;gap:10px;padding:0 12px;background:rgba(13,17,24,.85);
  border-bottom:1px solid #1e2633;z-index:10}
#topbar b{color:#e8edf5;font-size:14px}
button,select{background:#1a212d;color:#cfd6e1;border:1px solid #2a3547;
  border-radius:4px;padding:4px 10px;cursor:pointer;font:inherit}
button:hover{background:#232d3d}
#slider{flex:1;accent-color:#4da3ff}
#timelab{min-width:88px;text-align:right;font-variant-numeric:tabular-nums}
</style></head>
<body>
<div id="scene"></div>
<div id="topbar">
  <b id="title"></b>
  <button id="play">&#9654;</button>
  <select id="speed">
    <option value="0.25">0.25x</option><option value="0.5">0.5x</option>
    <option value="1" selected>1x</option><option value="2">2x</option>
    <option value="4">4x</option><option value="8">8x</option>
  </select>
  <input id="slider" type="range" min="0" max="1" step="1" value="0">
  <span id="timelab"></span>
  <select id="focus"></select>
  <select id="cammode">
    <option value="follow" selected>follow</option>
    <option value="orbit">orbit</option>
    <option value="chase">chase</option>
  </select>
</div>
<script>/*THREE_JS*/</script>
<script>/*ORBIT_JS*/</script>
<script>/*UPLOT_JS*/</script>
<script>
"use strict";
const DATA = /*DATA*/;

/*PURE-BEGIN*/
// NED (x north, y east, z down) -> scene (x east, y up, z north): right-handed.
function nedToScene(p){ return [p[1], -p[2], p[0]]; }
// Body->NED rotation from ZYX euler (columns = body axes in NED).
function bodyToNed(phi,theta,psi){
  const cf=Math.cos(phi),sf=Math.sin(phi),ct=Math.cos(theta),st=Math.sin(theta),
        cp=Math.cos(psi),sp=Math.sin(psi);
  return [
    [ct*cp, sf*st*cp-cf*sp, cf*st*cp+sf*sp],
    [ct*sp, sf*st*sp+cf*cp, cf*st*sp-sf*cp],
    [-st,   sf*ct,          cf*ct         ]];
}
// Body->scene 3x3 (row-major): S * R_bn with S = rows [ [0,1,0],[0,0,-1],[1,0,0] ].
function bodyToSceneMatrix(phi,theta,psi){
  const R = bodyToNed(phi,theta,psi);
  return [ R[1][0],R[1][1],R[1][2],
          -R[2][0],-R[2][1],-R[2][2],
           R[0][0],R[0][1],R[0][2] ];
}
/*PURE-END*/

// ---------------------------------------------------------------- scene
const container = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e13);
scene.fog = new THREE.Fog(0x0b0e13, 1, 1);   // ranged after bbox known

const camera = new THREE.PerspectiveCamera(55, 1, 0.5, 5e6);
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x20242c, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 0.8);
sun.position.set(0.4, 1, 0.6);
scene.add(sun);

// World extent from every trajectory point.
let ext = 100;
for (const v of DATA.vehicles)
  for (const p of v.frames.pos){
    const s = nedToScene(p);
    ext = Math.max(ext, Math.abs(s[0]), Math.abs(s[1]), Math.abs(s[2]));
  }
ext *= 1.2;
scene.fog.far = ext*8;

const grid = new THREE.GridHelper(ext*2, 40, 0x2b3a52, 0x1a2433);
scene.add(grid);
const groundMat = new THREE.MeshLambertMaterial({color:0x121a24});
const ground = new THREE.Mesh(new THREE.PlaneGeometry(ext*4, ext*4), groundMat);
ground.rotation.x = -Math.PI/2; ground.position.y = -0.5;
scene.add(ground);

// -------------------------------------------------------------- vehicles
function partMesh(part){
  const g = new THREE.BufferGeometry();
  const verts = new Float32Array(part.vertices.length*3);
  part.vertices.forEach((v,i)=>{ verts[3*i]=v[0]; verts[3*i+1]=v[1]; verts[3*i+2]=v[2]; });
  g.setAttribute('position', new THREE.BufferAttribute(verts,3));
  g.setIndex(part.faces.flat());
  g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshLambertMaterial(
    {color:new THREE.Color(part.color||'#c9ced6'), side:THREE.DoubleSide}));
  m.matrixAutoUpdate = false;
  return m;
}

const vObjs = DATA.vehicles.map(v=>{
  const grp = new THREE.Group();
  grp.matrixAutoUpdate = false;
  const hinged = [];
  for (const part of v.mesh.parts){
    const m = partMesh(part);
    if (part.hinges && part.hinges.length) hinged.push({mesh:m, hinges:part.hinges});
    grp.add(m);
  }
  scene.add(grp);

  // Trail: preallocated line, draw range grows with the frame index.
  const n = DATA.times.length;
  const tp = new Float32Array(n*3);
  v.frames.pos.forEach((p,i)=>{ const s = nedToScene(p);
    tp[3*i]=s[0]; tp[3*i+1]=s[1]; tp[3*i+2]=s[2]; });
  const tg = new THREE.BufferGeometry();
  tg.setAttribute('position', new THREE.BufferAttribute(tp,3));
  const trail = new THREE.Line(tg, new THREE.LineBasicMaterial(
    {color:0x4da3ff, transparent:true, opacity:0.7}));
  trail.frustumCulled = false;
  scene.add(trail);
  return {v, grp, hinged, trail};
});

const M4 = new THREE.Matrix4(), Ma = new THREE.Matrix4(), Mb = new THREE.Matrix4();
const AXIS = new THREE.Vector3();

function chanAt(v, name, k){
  const c = v.frames.chan[name];
  return c ? c[k] : 0;
}

function applyFrame(k){
  for (const o of vObjs){
    const f = o.v.frames;
    o.grp.visible = !!f.alive[k];
    o.trail.geometry.setDrawRange(0, k+1);
    if (!f.alive[k]) continue;
    const R = bodyToSceneMatrix(f.eul[k][0], f.eul[k][1], f.eul[k][2]);
    const s = nedToScene(f.pos[k]);
    o.grp.matrix.set(R[0],R[1],R[2],s[0], R[3],R[4],R[5],s[1],
                     R[6],R[7],R[8],s[2], 0,0,0,1);
    for (const h of o.hinged){
      M4.identity();
      for (const hg of h.hinges){
        const ang = hg.sign * chanAt(o.v, hg.channel, k);
        if (!ang) continue;
        AXIS.set(hg.axis[0],hg.axis[1],hg.axis[2]);
        Ma.makeTranslation(hg.origin[0],hg.origin[1],hg.origin[2]);
        Mb.makeRotationAxis(AXIS, ang);
        Ma.multiply(Mb);
        Mb.makeTranslation(-hg.origin[0],-hg.origin[1],-hg.origin[2]);
        Ma.multiply(Mb);
        M4.multiply(Ma);
      }
      h.mesh.matrix.copy(M4);
    }
  }
}

// -------------------------------------------------------------- playback
const slider = document.getElementById('slider');
const playBtn = document.getElementById('play');
const timelab = document.getElementById('timelab');
const focusSel = document.getElementById('focus');
const camSel = document.getElementById('cammode');
document.getElementById('title').textContent = DATA.name;
slider.max = DATA.times.length-1;
DATA.vehicles.forEach((v,i)=>{
  const opt = document.createElement('option');
  opt.value = i; opt.textContent = v.name;
  focusSel.appendChild(opt);
});

let frame = 0, playing = true, lastNow = null;
const frameDt = DATA.times[1] - DATA.times[0];
playBtn.onclick = ()=>{ playing = !playing; playBtn.innerHTML = playing?'&#10074;&#10074;':'&#9654;'; };
playBtn.innerHTML = '&#10074;&#10074;';
slider.oninput = ()=>{ frame = +slider.value; playing = false; playBtn.innerHTML='&#9654;'; };

function focusPos(k){
  const f = DATA.vehicles[+focusSel.value].frames;
  const s = nedToScene(f.pos[Math.min(k, f.pos.length-1)]);
  return new THREE.Vector3(s[0],s[1],s[2]);
}
function focusSize(){
  const mesh = DATA.vehicles[+focusSel.value].mesh;
  let m = 1;
  for (const p of mesh.parts) for (const v of p.vertices)
    m = Math.max(m, Math.abs(v[0]),Math.abs(v[1]),Math.abs(v[2]));
  return m;
}
function frameCamera(){
  const p = focusPos(Math.round(frame)), d = focusSize()*6;
  camera.position.set(p.x-d, p.y+d*0.5, p.z-d);
  controls.target.copy(p);
}
focusSel.onchange = frameCamera;
camSel.onchange = ()=>{ if (camSel.value!=='orbit') frameCamera();
                        else { controls.target.copy(focusPos(Math.round(frame))); } };
frameCamera();

function tick(now){
  requestAnimationFrame(tick);
  if (lastNow===null) lastNow = now;
  const dt = (now-lastNow)/1000; lastNow = now;
  if (playing){
    frame += dt * (+document.getElementById('speed').value) / frameDt;
    if (frame > DATA.times.length-1) frame = 0;
    slider.value = Math.round(frame);
  }
  const k = Math.min(Math.round(frame), DATA.times.length-1);
  timelab.textContent = 't = ' + DATA.times[k].toFixed(2) + ' s';

  const prevT = controls.target.clone();
  applyFrame(k);
  const mode = camSel.value;
  if (mode==='follow' || mode==='chase'){
    const p = focusPos(k);
    const delta = p.clone().sub(prevT);
    camera.position.add(delta);
    controls.target.copy(p);
    if (mode==='chase'){
      const v = DATA.vehicles[+focusSel.value];
      const R = bodyToSceneMatrix(v.frames.eul[k][0],v.frames.eul[k][1],v.frames.eul[k][2]);
      const back = new THREE.Vector3(-R[0],-R[3],-R[6]).multiplyScalar(focusSize()*8);
      back.y += focusSize()*3;
      camera.position.copy(p.clone().add(back));
    }
  }
  controls.update();
  renderer.render(scene, camera);
}

function resize(){
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w,h);
  camera.aspect = w/h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();
requestAnimationFrame(tick);
</script>
</body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", help="path to the scenario .json that was run")
    ap.add_argument("--out", help="output HTML path "
                    "(default: output/<scenario>/view.html)")
    args = ap.parse_args()

    data = build(args.scenario)
    out = args.out or os.path.join(PROJ, "output", data["name"], "view.html")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(render_html(data))

    size_kb = os.path.getsize(out) / 1024
    print(f"wrote {out}  ({size_kb:.0f} KB, {len(data['vehicles'])} vehicles, "
          f"{len(data['times'])} frames)")
    print("open it in any browser -- no server or internet needed")


if __name__ == "__main__":
    main()
