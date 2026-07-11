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
#panel{position:absolute;top:54px;left:10px;width:190px;z-index:10;
  background:rgba(13,17,24,.88);border:1px solid #1e2633;border-radius:6px;
  padding:8px 10px;max-height:calc(100% - 120px);overflow-y:auto}
#panel details{margin-bottom:6px}
#panel summary{cursor:pointer;color:#e8edf5;font-weight:600;margin-bottom:4px}
#panel label{display:flex;gap:6px;align-items:center;padding:2px 0;cursor:pointer}
#panel input{accent-color:#4da3ff}
#panel .sep{border-top:1px solid #1e2633;margin:6px 0}
#panel .tree details{margin:2px 0 4px}
#panel .tree summary{display:flex;align-items:center;gap:6px;color:#cfd6e1;
  font-weight:500;margin:0}
#panel .tree .panes{margin-left:22px}
#panel .clear{margin:2px 0 6px;width:100%}
#plotdock{position:absolute;top:44px;right:0;bottom:0;width:400px;z-index:9;
  display:none;flex-direction:column;gap:8px;padding:10px;overflow-y:auto;
  overflow-x:hidden;background:rgba(11,14,19,.94);border-left:1px solid #1e2633}
body.withplots #plotdock{display:flex}
body.withplots #scene{right:400px}
.pane{flex:0 0 auto;background:#0e1218;border:1px solid #1a2230;
  border-radius:6px;padding:4px 6px 2px}
.pane h4{margin:2px 0 0 6px;font-size:12px;color:#9fb2cc;font-weight:600}
.u-legend{font-size:11px;color:#cfd6e1}
.u-legend .u-marker{width:0.8em;height:0.8em}
#hint{position:absolute;right:12px;bottom:8px;z-index:8;color:#5d6c82;
  font-size:11px;pointer-events:none}
body.withplots #hint{right:412px}
</style></head>
<body>
<div id="scene"></div>
<div id="panel">
  <details open><summary>Overlays</summary><div id="ovlist"></div>
    <div class="sep"></div>
    <label><input type="checkbox" id="showall"> triad/labels on all</label>
  </details>
  <details open><summary>Plots</summary>
    <div class="tree" id="presetlist"></div>
    <button class="clear" id="clearplots">clear all</button>
  </details>
</div>
<div id="plotdock"></div>
<div id="hint">drag orbit &middot; scroll zoom &middot; space play/pause &middot;
&larr;/&rarr; step &middot; click a plot to seek</div>
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

// ---------------------------------------------------------------- overlays
const DEG = 180/Math.PI;
function vehSize(v){
  let m = 1;
  for (const p of v.mesh.parts) for (const vv of p.vertices)
    m = Math.max(m, Math.abs(vv[0]),Math.abs(vv[1]),Math.abs(vv[2]));
  return m;
}
function makeArrow(colorHex, headFrac=0.22){
  const a = new THREE.ArrowHelper(new THREE.Vector3(1,0,0),
                                  new THREE.Vector3(), 1, colorHex, headFrac, headFrac*0.6);
  a.visible = false;
  return a;
}
// Length with the head a fixed FRACTION of it -- heads stay proportionate to
// the arrow (and so to the vehicle) instead of ballooning in perspective.
function setArrowLen(a, len){
  a.setLength(len, len*0.16, len*0.07);
}
function makeTextSprite(){
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 96;
  const ctx = canvas.getContext('2d');
  const tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial(
    {map:tex, transparent:true, depthTest:false}));
  sp.userData = {canvas, ctx, tex, last:""};
  sp.visible = false;
  return sp;
}
function setSpriteText(sp, text){
  if (sp.userData.last === text) return;
  sp.userData.last = text;
  const {canvas, ctx, tex} = sp.userData;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.font = '600 34px system-ui,sans-serif';
  ctx.fillStyle = 'rgba(10,14,20,0.55)';
  const w = ctx.measureText(text).width + 24;
  ctx.fillRect((canvas.width-w)/2, 14, w, 58);
  ctx.fillStyle = '#e8edf5';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, canvas.width/2, 44);
  tex.needsUpdate = true;
}
// Arc fan in a body plane: origin -> arc(0..angle) -> origin, radius r.
const ARC_N = 20;
function makeArc(colorHex){
  const g = new THREE.BufferGeometry();
  g.setAttribute('position',
    new THREE.BufferAttribute(new Float32Array((ARC_N+2)*3), 3));
  const line = new THREE.Line(g, new THREE.LineBasicMaterial(
    {color:colorHex, transparent:true, opacity:0.9}));
  line.matrixAutoUpdate = false;   // body frame child
  line.visible = false;
  return line;
}
function setArc(line, angle, r, plane){   // plane 'xz' (alpha) | 'xy' (beta)
  const a = line.geometry.attributes.position.array;
  a[0]=0; a[1]=0; a[2]=0;
  for (let i=0;i<=ARC_N;i++){
    const t = angle*i/ARC_N, c = r*Math.cos(t), s = r*Math.sin(t);
    const j = 3*(i+1);
    if (plane==='xz'){ a[j]=c; a[j+1]=0; a[j+2]=s; }
    else             { a[j]=c; a[j+1]=s; a[j+2]=0; }
  }
  line.geometry.attributes.position.needsUpdate = true;
  line.geometry.computeBoundingSphere();
}

const OVERLAYS = [
  {id:'triad',   label:'body axes',      def:true},
  {id:'vel',     label:'velocity vector',def:true},
  {id:'arcs',    label:'alpha/beta arcs',def:true},
  {id:'los',     label:'LOS + closing',  def:true},
  {id:'thrust',  label:'thrust vector',  def:false},
  {id:'plume',   label:'exhaust plume',  def:true},
  {id:'aero',    label:'aero force',     def:false},
  {id:'ghost',   label:'setpoint ghost', def:true},
  {id:'trail',   label:'trails',         def:true},
  {id:'label',   label:'info labels',    def:true},
];
const store = (typeof localStorage !== 'undefined') ? localStorage
  : {getItem:()=>null, setItem:()=>{}};
const ovState = JSON.parse(store.getItem('viz_overlays')||'{}');
for (const o of OVERLAYS) if (!(o.id in ovState)) ovState[o.id] = o.def;
let showAll = !!ovState._showAll;

for (const o of vObjs){
  const L = vehSize(o.v);
  const f = o.v.frames;
  o.size = L;
  o.maxThrust = Math.max(1, ...(f.thrust||[0]));
  o.maxAero = 1;
  if (f.aeroF) for (const F of f.aeroF)
    o.maxAero = Math.max(o.maxAero, Math.hypot(F[0],F[1],F[2]));

  // Body-frame children (transform with the vehicle automatically).
  o.triad = new THREE.AxesHelper(L*1.8);      // x red, y green, z blue
  o.triad.visible = false;
  o.grp.add(o.triad);
  o.arcA = makeArc(0xffa14e); o.arcB = makeArc(0xda6ee8);
  o.grp.add(o.arcA); o.grp.add(o.arcB);
  o.thrustArrow = makeArrow(0xffc94d); o.thrustArrow.matrixAutoUpdate = true;
  o.grp.add(o.thrustArrow);
  o.aeroArrow = makeArrow(0x6fdc8c);
  o.grp.add(o.aeroArrow);
  const plumeGeo = new THREE.ConeGeometry(L*0.07, 1, 10);
  plumeGeo.translate(0, -0.5, 0);             // apex at origin, extends -y
  o.plume = new THREE.Mesh(plumeGeo, new THREE.MeshBasicMaterial(
    {color:0xff9a3d, transparent:true, opacity:0.75,
     blending:THREE.AdditiveBlending, depthWrite:false}));
  o.plume.visible = false;
  o.grp.add(o.plume);
  o.labelSp = makeTextSprite();
  o.labelSp.scale.set(L*5, L*0.94, 1);
  scene.add(o.labelSp);                       // world space (billboard)
  o.arcASp = makeTextSprite(); o.arcASp.scale.set(L*3.2, L*0.6, 1);
  scene.add(o.arcASp);
  o.aeroSp = makeTextSprite(); o.aeroSp.scale.set(L*2.2, L*0.42, 1);
  scene.add(o.aeroSp);

  // World-space glyphs (direction lives in NED, not body).
  o.velArrow = makeArrow(0x4dd7ff); scene.add(o.velArrow);
  o.ghostArrow = makeArrow(0xffffff, 0.18);
  o.ghostArrow.line.material.transparent = true;
  o.ghostArrow.line.material.opacity = 0.55;
  o.ghostArrow.cone.material.transparent = true;
  o.ghostArrow.cone.material.opacity = 0.55;
  scene.add(o.ghostArrow);
}
// LOS: one line + label for the focus vehicle.
const losGeo = new THREE.BufferGeometry();
losGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6),3));
const losLine = new THREE.Line(losGeo, new THREE.LineDashedMaterial(
  {color:0xff5f6b, dashSize:1, gapSize:1}));
losLine.frustumCulled = false;
losLine.visible = false;
scene.add(losLine);
const losSp = makeTextSprite();
scene.add(losSp);

const V3a = new THREE.Vector3(), V3b = new THREE.Vector3();

function updateOverlays(k){
  const focusI = +focusSel.value;
  const focus = vObjs[focusI];
  for (const o of vObjs){
    const isF = o === focus;
    const f = o.v.frames, L = o.size;
    const on = id => ovState[id] && f.alive[k] &&
                     (isF || (showAll && (id==='triad'||id==='label')));
    o.triad.visible = on('triad');
    o.trail.visible = ovState.trail;

    const s = nedToScene(f.pos[k]);
    // Velocity vector (NED direction, world space).
    const vv = f.vel[k], vmag = Math.hypot(vv[0],vv[1],vv[2]);
    o.velArrow.visible = on('vel') && vmag > 1;
    if (o.velArrow.visible){
      const d = nedToScene(vv);
      o.velArrow.position.set(s[0],s[1],s[2]);
      o.velArrow.setDirection(V3a.set(d[0],d[1],d[2]).normalize());
      setArrowLen(o.velArrow, L*2.6);
    }
    // Alpha / beta arcs + readout (body-frame children).
    const showArcs = on('arcs') && f.alpha;
    o.arcA.visible = o.arcB.visible = !!showArcs;
    o.arcASp.visible = !!showArcs;
    if (showArcs){
      setArc(o.arcA, f.alpha[k], L*1.5, 'xz');
      setArc(o.arcB, f.beta[k],  L*1.5, 'xy');
      o.arcASp.position.set(s[0], s[1]+L*2.4, s[2]);
      setSpriteText(o.arcASp,
        'a ' + (f.alpha[k]*DEG).toFixed(1) + '°   b ' +
        (f.beta[k]*DEG).toFixed(1) + '°');
    }
    // Thrust vector: body direction incl. gimbal (matches Propulsor).
    const T = f.thrust ? f.thrust[k] : 0;
    o.thrustArrow.visible = on('thrust') && T > 0;
    if (o.thrustArrow.visible){
      const dp = chanAt(o.v,'tvc_pitch',k), dy = chanAt(o.v,'tvc_yaw',k);
      V3a.set(Math.cos(dp)*Math.cos(dy), -Math.cos(dp)*Math.sin(dy), Math.sin(dp));
      o.thrustArrow.setDirection(V3a);
      setArrowLen(o.thrustArrow, L*(0.8 + 1.8*T/o.maxThrust));
    }
    // Exhaust plume: cone off the tail along -thrust direction, scaled by T.
    o.plume.visible = ovState.plume && f.alive[k] && T > 0;
    if (o.plume.visible){
      const dp = chanAt(o.v,'tvc_pitch',k), dy = chanAt(o.v,'tvc_yaw',k);
      V3a.set(-Math.cos(dp)*Math.cos(dy), Math.cos(dp)*Math.sin(dy), -Math.sin(dp));
      o.plume.position.set(-L*0.98, 0, 0);
      o.plume.quaternion.setFromUnitVectors(V3b.set(0,-1,0).normalize(), V3a);
      const flick = 0.92 + 0.16*Math.random();
      o.plume.scale.set(1, L*(0.6 + 2.8*T/o.maxThrust)*flick, 1);
    }
    // Aero force (body-frame child arrow). sqrt scaling keeps mid-range
    // forces distinguishable; the label states the actual magnitude.
    o.aeroArrow.visible = on('aero') && !!f.aeroF;
    o.aeroSp.visible = false;
    if (o.aeroArrow.visible){
      const F = f.aeroF[k], mag = Math.hypot(F[0],F[1],F[2]);
      o.aeroArrow.visible = mag > 1e-3;
      if (o.aeroArrow.visible){
        o.aeroArrow.setDirection(V3a.set(F[0],F[1],F[2]).normalize());
        const len = L*(0.4 + 1.6*Math.sqrt(mag/o.maxAero));
        setArrowLen(o.aeroArrow, len);
        // Magnitude readout near the arrow tip (world space).
        o.aeroSp.visible = true;
        o.grp.updateMatrixWorld();
        V3b.set(F[0],F[1],F[2]).normalize().multiplyScalar(len*1.15)
           .applyMatrix4(o.grp.matrixWorld);
        o.aeroSp.position.copy(V3b);
        setSpriteText(o.aeroSp, mag >= 1000 ? (mag/1000).toFixed(1)+' kN'
                                            : Math.round(mag)+' N');
      }
    }
    // Setpoint ghost: commanded attitude direction (NED), translucent.
    const spP = f.spPitch ? f.spPitch[k] : NaN;
    const spH = f.spHeading ? f.spHeading[k] : NaN;
    const hasSp = Number.isFinite(spP) || Number.isFinite(spH);
    o.ghostArrow.visible = on('ghost') && hasSp;
    if (o.ghostArrow.visible){
      const e = f.eul[k];
      const p = Number.isFinite(spP) ? spP : e[1];
      const h = Number.isFinite(spH) ? spH : e[2];
      const d = nedToScene([Math.cos(p)*Math.cos(h), Math.cos(p)*Math.sin(h), -Math.sin(p)]);
      o.ghostArrow.position.set(s[0],s[1],s[2]);
      o.ghostArrow.setDirection(V3a.set(d[0],d[1],d[2]).normalize());
      setArrowLen(o.ghostArrow, L*3.2);
    }
    // Info label above the vehicle.
    o.labelSp.visible = on('label');
    if (o.labelSp.visible){
      o.labelSp.position.set(s[0], s[1]+L*1.6, s[2]);
      const alt = -f.pos[k][2];
      setSpriteText(o.labelSp, o.v.name + '  ' + Math.round(alt) + ' m  M' +
                    (f.mach ? f.mach[k].toFixed(2) : '?'));
    }
  }
  // LOS from the focus vehicle to its target.
  const tgtI = DATA.vehicles.findIndex(v => v.name === focus.v.target);
  const showLos = ovState.los && tgtI >= 0 &&
                  focus.v.frames.alive[k] && DATA.vehicles[tgtI].frames.alive[k];
  losLine.visible = losSp.visible = showLos;
  if (showLos){
    const a = focus.v.frames, b = DATA.vehicles[tgtI].frames;
    const pa = nedToScene(a.pos[k]), pb = nedToScene(b.pos[k]);
    const arr = losGeo.attributes.position.array;
    arr[0]=pa[0];arr[1]=pa[1];arr[2]=pa[2];arr[3]=pb[0];arr[4]=pb[1];arr[5]=pb[2];
    losGeo.attributes.position.needsUpdate = true;
    losGeo.computeBoundingSphere();
    losLine.computeLineDistances();
    const dx=b.pos[k][0]-a.pos[k][0], dy=b.pos[k][1]-a.pos[k][1], dz=b.pos[k][2]-a.pos[k][2];
    const rng = Math.hypot(dx,dy,dz);
    const vc = -((b.vel[k][0]-a.vel[k][0])*dx + (b.vel[k][1]-a.vel[k][1])*dy +
                 (b.vel[k][2]-a.vel[k][2])*dz) / Math.max(rng,1e-6);
    losSp.position.set((pa[0]+pb[0])/2, (pa[1]+pb[1])/2 + focus.size*2, (pa[2]+pb[2])/2);
    const sc = Math.max(focus.size*6, rng*0.06);
    losSp.scale.set(sc, sc*0.19, 1);
    setSpriteText(losSp, 'R ' + Math.round(rng) + ' m   Vc ' + Math.round(vc) + ' m/s');
    const dash = Math.max(1, rng/60);
    losLine.material.dashSize = dash; losLine.material.gapSize = dash*0.6;
  }
}

// Panel wiring.
const ovlist = document.getElementById('ovlist');
for (const o of OVERLAYS){
  const lab = document.createElement('label');
  const cb = document.createElement('input');
  cb.type = 'checkbox'; cb.checked = !!ovState[o.id];
  cb.onchange = ()=>{ ovState[o.id] = cb.checked; saveOv(); };
  lab.appendChild(cb); lab.appendChild(document.createTextNode(o.label));
  ovlist.appendChild(lab);
}
const showallCb = document.getElementById('showall');
showallCb.checked = showAll;
showallCb.onchange = ()=>{ showAll = showallCb.checked;
                           ovState._showAll = showAll; saveOv(); };
function saveOv(){ store.setItem('viz_overlays', JSON.stringify(ovState)); }

// ------------------------------------------------------------ preset plots
const PALETTE = ['#4da3ff','#ffa14e','#6fdc8c','#da6ee8','#ff5f6b','#ffd24d','#7ee0e8'];
const SETPOINT_DASH = [7,5];

// Preset library: built per focus vehicle from the columns its log has.
// s(col,label,{deg,sp}) -> series spec; a pane is dropped when no series
// exists; a preset is dropped when no pane survives.
function presetsFor(v){
  const has = c => c in v.series;
  const s = (col,label,o={}) => has(col) ? {col,label,deg:!!o.deg,sp:!!o.sp} : null;
  const P = [];
  const add = (id,label,panes)=>{
    panes = panes.map(p=>({title:p.title, series:p.series.filter(Boolean)}))
                 .filter(p=>p.series.length);
    if (panes.length) P.push({id,label,panes});
  };
  add('tracking','Tracking',[
    {title:'pitch [deg]',   series:[s('theta','actual',{deg:1}), s('pitch_sp','setpoint',{deg:1,sp:1})]},
    {title:'heading [deg]', series:[s('psi','actual',{deg:1}), s('heading_sp','setpoint',{deg:1,sp:1})]},
    {title:'roll [deg]',    series:[s('phi','actual',{deg:1}), s('roll_sp','setpoint',{deg:1,sp:1})]},
  ]);
  add('rates','Rates & load',[
    {title:'body rates [deg/s]', series:[s('p','p',{deg:1}), s('q','q',{deg:1}), s('r','r',{deg:1})]},
    {title:'incidence [deg]',    series:[s('alpha','alpha',{deg:1}), s('beta','beta',{deg:1})]},
    {title:'load factor [g]',    series:[s('gload','n')]},
  ]);
  add('airdata','Air data',[
    {title:'Mach',        series:[s('mach','Mach')]},
    {title:'qbar [kPa]',  series:[has('qbar')?{col:'qbar',label:'qbar',mult:1e-3}:null]},
    {title:'speeds [m/s]',series:[s('airspeed','airspeed'), s('vground','ground'), s('vspeed','vertical')]},
  ]);
  add('traj','Trajectory',[
    {title:'altitude [m]',    series:[s('altitude','altitude')]},
    {title:'flight path [deg]',series:[s('gamma','gamma',{deg:1})]},
    {title:'speed [m/s]',     series:[s('airspeed','airspeed')]},
  ]);
  add('control','Control activity',[
    {title:'elevator [deg]', series:[s('elevator','actual',{deg:1}), s('elevator_cmd','commanded',{deg:1,sp:1})]},
    {title:'aileron [deg]',  series:[s('aileron','actual',{deg:1}), s('aileron_cmd','commanded',{deg:1,sp:1})]},
    {title:'rudder [deg]',   series:[s('rudder','actual',{deg:1}), s('rudder_cmd','commanded',{deg:1,sp:1})]},
    {title:'gimbal [deg]',   series:[s('tvc_pitch','tvc pitch',{deg:1}), s('tvc_yaw','tvc yaw',{deg:1})]},
  ]);
  add('prop','Propulsion & mass',[
    {title:'thrust [kN]', series:[has('thrust')?{col:'thrust',label:'thrust',mult:1e-3}:null]},
    {title:'mass [kg]',   series:[s('mass','mass')]},
    {title:'throttle',    series:[s('throttle','actual'), s('throttle_cmd','commanded',{sp:1})]},
  ]);
  add('intercept','Intercept',[
    {title:'range [m]',        series:[s('range','range')]},
    {title:'closing [m/s]',    series:[s('closing','closing speed')]},
    {title:'LOS angles [deg]', series:[s('los_az','azimuth',{deg:1}), s('los_el','elevation',{deg:1})]},
  ]);
  const comps = v.components || [];
  add('alloc','Allocation',[
    {title:'pitch moment [kNm]', series:comps.map((c,i)=>
      has(c+'_my')?{col:c+'_my',label:c,mult:1e-3}:null)},
    {title:'yaw moment [kNm]',   series:comps.map(c=>
      has(c+'_mz')?{col:c+'_mz',label:c,mult:1e-3}:null)},
    {title:'roll moment [kNm]',  series:comps.map(c=>
      has(c+'_mx')?{col:c+'_mx',label:c,mult:1e-3}:null)},
  ]);
  return P;
}

let plots = [];
// Selected panes: {presetId: [paneIndex, ...]} -- a checkbox TREE in the
// panel; every checked pane stacks in the right-side dock, top to bottom.
let paneSel = JSON.parse(store.getItem('viz_panes') || '{}');
let seeking = false;

function destroyPlots(){
  for (const u of plots) u.destroy();
  plots = [];
  document.getElementById('plotdock').innerHTML = '';
}

function seriesData(v, spec){
  const mult = spec.deg ? DEG : (spec.mult || 1);
  return v.series[spec.col].map(x => Number.isFinite(x) ? x*mult : null);
}

function selectedPanes(){
  const v = DATA.vehicles[+focusSel.value];
  const out = [];
  for (const p of presetsFor(v))
    for (const i of (paneSel[p.id] || []))
      if (p.panes[i]) out.push(p.panes[i]);
  return out;
}

function buildPlots(){
  destroyPlots();
  const v = DATA.vehicles[+focusSel.value];
  const panes = selectedPanes();
  document.body.classList.toggle('withplots', panes.length > 0);
  resize();
  if (!panes.length) return;
  const dock = document.getElementById('plotdock');
  const t = v.series.time;
  for (const pane of panes){
    const el = document.createElement('div');
    el.className = 'pane';
    const h = document.createElement('h4');
    h.textContent = pane.title;
    el.appendChild(h);
    dock.appendChild(el);
    const w = dock.clientWidth - 42;
    const axis = {stroke:'#8ea0b8', grid:{stroke:'#1a2230'}, ticks:{stroke:'#1a2230'}};
    const opts = {
      width: w, height: 190,
      cursor: {y:false, drag:{setScale:true, x:true, y:false}},
      scales: {x:{time:false}},
      axes: [axis, axis],
      series: [{label:'t'}].concat(pane.series.map((sp,i)=>({
        label: sp.label, stroke: PALETTE[i%PALETTE.length], width: sp.sp?1.4:1.8,
        dash: sp.sp ? SETPOINT_DASH : undefined, points:{show:false},
      }))),
      hooks: {
        draw: [u => {                     // animation time line
          const x = u.valToPos(curTime, 'x', true);
          if (x < u.bbox.left || x > u.bbox.left+u.bbox.width) return;
          const ctx = u.ctx;
          ctx.save();
          ctx.strokeStyle = 'rgba(232,237,245,0.65)';
          ctx.setLineDash([4,4]); ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(x, u.bbox.top); ctx.lineTo(x, u.bbox.top+u.bbox.height);
          ctx.stroke();
          ctx.restore();
        }],
        init: [u => {                     // click a plot -> seek the animation
          u.over.addEventListener('click', e => {
            if (seeking) return;
            const rect = u.over.getBoundingClientRect();
            const tt = u.posToVal(e.clientX - rect.left, 'x');
            if (Number.isFinite(tt)) seek(tt);
          });
        }],
      },
    };
    plots.push(new uPlot(opts, [t, ...pane.series.map(sp => seriesData(v, sp))], el));
  }
}

let lastPlotK = -1;
function updatePlotCursor(k){
  if (k === lastPlotK || !plots.length) return;
  lastPlotK = k;
  for (const u of plots) u.redraw(false, false);
}

function seek(tt){
  const T = DATA.times;
  frame = Math.max(0, Math.min(T.length-1, tt/(T[1]-T[0])));
  slider.value = Math.round(frame);
}

// Plot tree: preset groups expand into per-pane checkboxes; a group
// checkbox toggles the whole story (indeterminate when partial).
{
  const list = document.getElementById('presetlist');
  const savePanes = ()=> store.setItem('viz_panes', JSON.stringify(paneSel));
  const rebuild = ()=>{
    const openState = {};
    for (const d of list.children) openState[d.dataset.pid] = d.open;
    list.innerHTML = '';
    for (const p of presetsFor(DATA.vehicles[+focusSel.value])){
      const sel = new Set(paneSel[p.id] || []);
      const det = document.createElement('details');
      det.dataset.pid = p.id;
      det.open = (p.id in openState) ? openState[p.id] : sel.size > 0;
      const sum = document.createElement('summary');
      const gcb = document.createElement('input');
      gcb.type = 'checkbox';
      gcb.checked = sel.size === p.panes.length;
      gcb.indeterminate = sel.size > 0 && sel.size < p.panes.length;
      gcb.addEventListener('click', e => e.stopPropagation());
      gcb.onchange = ()=>{
        paneSel[p.id] = gcb.checked ? p.panes.map((_,i)=>i) : [];
        savePanes(); rebuild(); buildPlots();
      };
      sum.appendChild(gcb);
      sum.appendChild(document.createTextNode(p.label));
      det.appendChild(sum);
      const box = document.createElement('div');
      box.className = 'panes';
      p.panes.forEach((pane, i)=>{
        const lab = document.createElement('label');
        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.checked = sel.has(i);
        cb.onchange = ()=>{
          if (cb.checked) sel.add(i); else sel.delete(i);
          paneSel[p.id] = [...sel].sort((a,b)=>a-b);
          savePanes(); rebuild(); buildPlots();
        };
        lab.appendChild(cb);
        lab.appendChild(document.createTextNode(pane.title));
        box.appendChild(lab);
      });
      det.appendChild(box);
      list.appendChild(det);
    }
  };
  document.getElementById('clearplots').onclick = ()=>{
    paneSel = {}; savePanes(); rebuild(); buildPlots();
  };
  window.__rebuildPresets = rebuild;
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
  updateOverlays(k);
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

let frame = 0, playing = true, lastNow = null, curTime = 0;
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
focusSel.onchange = ()=>{ frameCamera(); window.__rebuildPresets(); buildPlots(); };
camSel.onchange = ()=>{ if (camSel.value!=='orbit') frameCamera();
                        else { controls.target.copy(focusPos(Math.round(frame))); } };
frameCamera();
window.__rebuildPresets();
buildPlots();
let resizeTimer = null;
window.addEventListener('resize', ()=>{
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(buildPlots, 200);
});
window.addEventListener('keydown', e=>{
  if (e.target.tagName === 'SELECT' || e.target.tagName === 'INPUT') return;
  if (e.code === 'Space'){ playBtn.onclick(); e.preventDefault(); }
  else if (e.code === 'ArrowRight' || e.code === 'ArrowLeft'){
    playing = false; playBtn.innerHTML = '&#9654;';
    const step = (e.code === 'ArrowRight' ? 1 : -1) * (e.shiftKey ? 10 : 1);
    frame = Math.max(0, Math.min(DATA.times.length-1, Math.round(frame)+step));
    slider.value = Math.round(frame);
    e.preventDefault();
  }
});

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
  curTime = DATA.times[k];
  timelab.textContent = 't = ' + curTime.toFixed(2) + ' s';
  updatePlotCursor(k);

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
