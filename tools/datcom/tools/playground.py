"""Aero playground — interactive vehicle + live DATCOM coefficients.

Standalone amusement tool (NOT part of the pipeline: nothing in pydatcom,
make_examples or the dataset generator invokes it). Builds a single
self-contained HTML page:

* the 3-D vehicle, pitching with alpha and deflecting its fins with delta,
* sliders for Mach / alpha / fin deflection,
* live coefficient readouts bilinearly interpolated from the real parsed
  DATCOM tables (aero.npz),
* a live CN / CM-vs-alpha plot with a cursor at the current state.

Usage:
    py tools/playground.py [case ...]        # rocket boattail three_fin canard
    py tools/playground.py all

Output: playground/<case>.html (gitignored). From Python:

    from tools.playground import build_playground
    build_playground(vehicle, np.load("aero.npz"), "play.html", "my rocket")
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from pydatcom import (Vehicle, define_example_canard_missile,
                      define_example_rocket)
from pydatcom.geometry import _fin_azimuths, vehicle_mesh

CASES = {
    "rocket": (define_example_rocket, "01_rocket_baseline"),
    "control": (define_example_rocket, "02_rocket_fin_control"),
    "boattail": (lambda: Vehicle(L_af=21.0, L_bt=2.0, D_bt=0.8),
                 "03_rocket_boattail"),
    "three_fin": (lambda: Vehicle(fin_num=3), "04_rocket_three_fin"),
    "canard": (define_example_canard_missile, "05_canard_missile"),
}

_COLORS = {"body": [200, 200, 205], "fin": [217, 64, 64],
           "canard": [64, 115, 217]}

# static/dynamic tables shown live (label, key, decimals)
_READOUTS = [("CN  normal force", "cn", 3), ("CM  pitch moment", "cm", 3),
             ("CD  drag", "cd", 3), ("CL  lift", "cl", 3),
             ("CA  axial force", "ca", 3), ("Xcp centre of pressure", "xcp", 3),
             ("CNa normal-force slope", "cla", 4),
             ("CMa pitch stability", "cma", 4),
             ("CMQ pitch damping", "cmq", 1), ("CLP roll damping", "clp", 3)]


def _get(src, name):
    """Field from an AeroData object or an npz mapping (None if absent)."""
    try:
        v = src[name] if hasattr(src, "files") else getattr(src, name)
    except (KeyError, AttributeError):
        return None
    return None if v is None else np.asarray(v)


def build_playground(vehicle: Vehicle, aero, out_path: str,
                     title: str = "aero playground") -> str:
    """Write the playground HTML for one vehicle + its parsed aero data."""
    # ---- mesh with per-part hinge metadata --------------------------------
    hinge_x = vehicle.L - vehicle.fin_disp - vehicle.fin_root / 2
    azims = _fin_azimuths(vehicle.fin_num)
    parts = []
    for name, verts, faces in vehicle_mesh(vehicle):
        kind = ("body" if name == "body"
                else "canard" if name.startswith("canard") else "fin")
        part = {"name": name, "color": _COLORS[kind],
                "v": [[round(float(c), 5) for c in p] for p in verts],
                "f": [[int(a), int(b), int(c)] for a, b, c in faces]}
        if kind == "fin":
            az = math.radians(azims[int(name.split("_")[1]) - 1])
            u = [0.0, math.cos(az), math.sin(az)]
            if abs(u[1]) > 0.5:                 # horizontal-ish panel: pitch
                if u[1] < 0:                    # keep TE-down for +delta
                    u = [-c for c in u]
                part["hinge"] = {"x": hinge_x, "axis": u}
        parts.append(part)

    # ---- aero tables ------------------------------------------------------
    mach = _get(aero, "mach").tolist()
    alpha = _get(aero, "alpha").tolist()
    tables = {}
    for _, key, _ in _READOUTS:
        t = _get(aero, key)
        if t is not None:
            tables[key] = np.round(t, 6).tolist()
    delta = _get(aero, "delta")
    ctrl = {}
    if delta is not None and delta.size:
        ctrl = {"delta": delta.tolist(),
                "dcl": np.round(_get(aero, "dcl_sym"), 6).tolist(),
                "dcm": np.round(_get(aero, "dcm_sym"), 6).tolist()}
        clroll = _get(aero, "clroll")
        if clroll is not None:
            ctrl["clroll"] = np.round(clroll, 6).tolist()

    data = {"title": title, "xcg": vehicle.xcg,
            "mesh": {"parts": parts},
            "aero": {"mach": mach, "alpha": alpha, "tables": tables,
                     "ctrl": ctrl},
            "readouts": [[lbl, key, dec] for lbl, key, dec in _READOUTS
                         if key in tables]}

    html = (_TEMPLATE
            .replace("__TITLE__", title)
            .replace("__DATA__", json.dumps(data, separators=(",", ":"))))
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path


_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  html,body{margin:0;height:100%;overflow:hidden;background:#1c1e22;
            font-family:system-ui,sans-serif;color:#cdd3df}
  #c{display:block;width:100vw;height:100vh}
  #hud{position:fixed;left:12px;top:10px;font-size:13px;color:#aab;
       user-select:none;pointer-events:none}
  #hud b{color:#dde;font-size:15px}
  #panel{position:fixed;right:0;top:0;bottom:0;width:330px;overflow-y:auto;
         background:rgba(24,26,31,.93);border-left:1px solid #333a46;
         padding:14px 16px;box-sizing:border-box}
  h3{margin:14px 0 6px;font-size:13px;color:#8899bb;text-transform:uppercase;
     letter-spacing:.08em}
  .sl{margin:8px 0}
  .sl label{display:flex;justify-content:space-between;font-size:13px}
  .sl output{color:#ffd479;font-variant-numeric:tabular-nums}
  input[type=range]{width:100%;accent-color:#5588dd}
  table{width:100%;border-collapse:collapse;font-size:13px}
  td{padding:2.5px 0;border-bottom:1px solid #2a2e37}
  td.v{text-align:right;color:#8fd48f;font-variant-numeric:tabular-nums;
       font-family:Consolas,monospace}
  td.v.neg{color:#e08f8f}
  #plot{background:#14161a;border:1px solid #333a46;border-radius:4px;
        margin-top:8px}
  .note{font-size:11px;color:#667;margin-top:10px;line-height:1.5}
</style>
</head>
<body>
<div id="hud"><b>__TITLE__</b><br>
drag: rotate &nbsp;|&nbsp; wheel: zoom &nbsp;|&nbsp; double-click: reset view</div>
<canvas id="c"></canvas>
<div id="panel">
  <h3>Flight condition</h3>
  <div class="sl"><label>Mach <output id="machv"></output></label>
    <input type="range" id="mach"></div>
  <div class="sl"><label>alpha [deg] <output id="alphav"></output></label>
    <input type="range" id="alpha"></div>
  <div class="sl" id="deltarow" style="display:none">
    <label>fin deflection [deg] <output id="deltav"></output></label>
    <input type="range" id="delta"></div>
  <h3>Coefficients</h3>
  <table id="coeffs"></table>
  <h3>CN &amp; CM vs alpha</h3>
  <canvas id="plot" width="296" height="190"></canvas>
  <div class="note">Values bilinearly interpolated from the DATCOM tables in
  this case's aero.npz. Deflection increments are DATCOM's linear-range
  SYMFLP values; totals shown as base + increment. Positive deflection =
  trailing edge down.</div>
</div>
<script>
const D = __DATA__;

// ================= 3-D renderer =================
const cv = document.getElementById("c"), ctx = cv.getContext("2d");
let W, H;
function resize(){
  const dpr = window.devicePixelRatio || 1;
  W = cv.clientWidth; H = cv.clientHeight;
  cv.width = W * dpr; cv.height = H * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}
window.addEventListener("resize", resize);

let lo=[1e30,1e30,1e30], hi=[-1e30,-1e30,-1e30];
for (const p of D.mesh.parts) for (const v of p.v)
  for (let k=0;k<3;k++){ lo[k]=Math.min(lo[k],v[k]); hi[k]=Math.max(hi[k],v[k]); }
const ctr=[0,1,2].map(k=>(lo[k]+hi[k])/2), size=Math.max(...[0,1,2].map(k=>hi[k]-lo[k]));
const tris=[]; D.mesh.parts.forEach((p,pi)=>{ for(const f of p.f) tris.push([pi,f[0],f[1],f[2]]); });

function rotX(a){const c=Math.cos(a),s=Math.sin(a);return[1,0,0,0,c,-s,0,s,c];}
function rotY(a){const c=Math.cos(a),s=Math.sin(a);return[c,0,s,0,1,0,-s,0,c];}
function mul(A,B){const M=new Array(9);
  for(let r=0;r<3;r++)for(let c=0;c<3;c++)
    M[3*r+c]=A[3*r]*B[c]+A[3*r+1]*B[3+c]+A[3*r+2]*B[6+c];
  return M;}
function apply(M,v){return[M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
  M[3]*v[0]+M[4]*v[1]+M[5]*v[2], M[6]*v[0]+M[7]*v[1]+M[8]*v[2]];}
function axisRot(u,a){ // Rodrigues
  const c=Math.cos(a),s=Math.sin(a),t=1-c,[x,y,z]=u;
  return[t*x*x+c,t*x*y-s*z,t*x*z+s*y, t*x*y+s*z,t*y*y+c,t*y*z-s*x,
         t*x*z-s*y,t*y*z+s*x,t*z*z+c];}

let R, zoom;
function resetView(){ R = mul(rotY(-0.5), rotX(0.35)); zoom = 1; draw(); }

function draw(){
  if (!R || !W) return;
  ctx.clearRect(0,0,W,H);
  const a = state.alpha*Math.PI/180, d = state.delta*Math.PI/180;
  const pitch = rotY(a), cg=[D.xcg,0,0];
  const scale = 0.8*Math.min(W-330,H)/size*zoom, camd = 3*size, f = camd*scale;
  const cx=(W-330)/2, cy=H/2;

  const P=[], Zs=[];
  for (const p of D.mesh.parts){
    const hin = p.hinge ? axisRot(p.hinge.axis, -d) : null;
    const pts=new Array(p.v.length), zs=new Array(p.v.length);
    for (let i=0;i<p.v.length;i++){
      let v=p.v[i];
      if (hin){ const r=[v[0]-p.hinge.x,v[1],v[2]]; const q=apply(hin,r);
                v=[q[0]+p.hinge.x,q[1],q[2]]; }
      let w=[v[0]-cg[0],v[1]-cg[1],v[2]-cg[2]];
      w=apply(pitch,w);
      w=[w[0]+cg[0]-ctr[0], w[1]+cg[1]-ctr[1], w[2]+cg[2]-ctr[2]];
      w=apply(R,w);
      const z=w[2]+camd;
      pts[i]=[cx+f*w[0]/z, cy-f*w[1]/z]; zs[i]=w;
    }
    P.push(pts); Zs.push(zs);
  }
  const order=tris.map((t,i)=>{const z=Zs[t[0]];
    return[(z[t[1]][2]+z[t[2]][2]+z[t[3]][2])/3,i];}).sort((x,y)=>x[0]-y[0]);
  const L=[0.45,0.5,0.74];
  for (const [,ti] of order){
    const [pi,a1,b1,c1]=tris[ti], pts=P[pi], z=Zs[pi];
    const u=[z[b1][0]-z[a1][0],z[b1][1]-z[a1][1],z[b1][2]-z[a1][2]];
    const v=[z[c1][0]-z[a1][0],z[c1][1]-z[a1][1],z[c1][2]-z[a1][2]];
    const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    const nl=Math.hypot(n[0],n[1],n[2])||1;
    let s=Math.abs((n[0]*L[0]+n[1]*L[1]+n[2]*L[2])/nl); s=0.35+0.62*s;
    const col=D.mesh.parts[pi].color;
    ctx.fillStyle=`rgb(${col[0]*s|0},${col[1]*s|0},${col[2]*s|0})`;
    ctx.beginPath();
    ctx.moveTo(pts[a1][0],pts[a1][1]); ctx.lineTo(pts[b1][0],pts[b1][1]);
    ctx.lineTo(pts[c1][0],pts[c1][1]); ctx.closePath(); ctx.fill();
  }
  // wind arrow ahead of the nose, along +x (relative wind)
  const A=[], arrow=[[-0.28*size,0,0],[-0.06*size,0,0]];
  for (const v of arrow){
    const w=apply(R,[v[0]+lo[0]-ctr[0],-ctr[1],-ctr[2]]); const z=w[2]+camd;
    A.push([cx+f*w[0]/z, cy-f*w[1]/z]);
  }
  ctx.strokeStyle="#7fa3e8"; ctx.fillStyle="#7fa3e8"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(A[0][0],A[0][1]); ctx.lineTo(A[1][0],A[1][1]); ctx.stroke();
  const ang=Math.atan2(A[1][1]-A[0][1],A[1][0]-A[0][0]);
  ctx.beginPath(); ctx.moveTo(A[1][0],A[1][1]);
  ctx.lineTo(A[1][0]-10*Math.cos(ang-0.4),A[1][1]-10*Math.sin(ang-0.4));
  ctx.lineTo(A[1][0]-10*Math.cos(ang+0.4),A[1][1]-10*Math.sin(ang+0.4));
  ctx.closePath(); ctx.fill();
  ctx.font="12px system-ui"; ctx.fillText("V∞", A[0][0]-4, A[0][1]-8);
}

let dragv=null;
cv.addEventListener("mousedown",e=>{dragv={x:e.clientX,y:e.clientY};});
window.addEventListener("mousemove",e=>{
  if(!dragv)return;
  R=mul(mul(rotY((e.clientX-dragv.x)*0.008),rotX((e.clientY-dragv.y)*0.008)),R);
  dragv={x:e.clientX,y:e.clientY}; draw();
});
window.addEventListener("mouseup",()=>dragv=null);
cv.addEventListener("wheel",e=>{e.preventDefault();
  zoom*=Math.exp(-e.deltaY*0.001); draw();},{passive:false});
cv.addEventListener("dblclick",resetView);

// ================= interpolation =================
const AE=D.aero;
function fidx(g,x){
  if(x<=g[0])return 0;
  if(x>=g[g.length-1])return g.length-1;
  let i=0; while(g[i+1]<x)i++;
  return i+(x-g[i])/(g[i+1]-g[i]);
}
function bilin(T,ai,mi){
  const i=Math.min(Math.floor(ai),T.length-1), j=Math.min(Math.floor(mi),T[0].length-1);
  const i1=Math.min(i+1,T.length-1), j1=Math.min(j+1,T[0].length-1);
  const fa=ai-i, fm=mi-j;
  return T[i][j]*(1-fa)*(1-fm)+T[i1][j]*fa*(1-fm)+T[i][j1]*(1-fa)*fm+T[i1][j1]*fa*fm;
}
function evalTab(key,al,m){
  return bilin(AE.tables[key], fidx(AE.alpha,al), fidx(AE.mach,m));
}
function evalCtrl(key,de,m){
  return bilin(AE.ctrl[key], fidx(AE.ctrl.delta,de), fidx(AE.mach,m));
}

// ================= UI =================
const state={mach:0.3, alpha:0, delta:0};
const $=id=>document.getElementById(id);
function setupSlider(id,min,max,step,val,cb){
  const el=$(id); el.min=min; el.max=max; el.step=step; el.value=val;
  el.addEventListener("input",()=>{cb(parseFloat(el.value)); update();});
}
const mmin=AE.mach[0], mmax=AE.mach[AE.mach.length-1];
setupSlider("mach",mmin,mmax,0.01,0.3,v=>state.mach=v);
setupSlider("alpha",-30,30,0.5,0,v=>state.alpha=v);
const hasCtrl=AE.ctrl && AE.ctrl.delta;
if(hasCtrl){
  $("deltarow").style.display="";
  const ds=AE.ctrl.delta;
  setupSlider("delta",ds[0],ds[ds.length-1],0.5,0,v=>state.delta=v);
}

// coefficient table rows
const tbl=$("coeffs"), cells={};
function addRow(label,id){
  const tr=document.createElement("tr");
  const t1=document.createElement("td"), t2=document.createElement("td");
  t1.textContent=label; t2.className="v"; tr.append(t1,t2); tbl.append(tr);
  cells[id]=t2;
}
for(const [lbl,key] of D.readouts) addRow(lbl,key);
if(hasCtrl){
  addRow("ΔCL  lift incr (δ)","dcl");
  addRow("ΔCM  moment incr (δ)","dcm");
  addRow("CM total (base+Δ)","cmtot");
  if(AE.ctrl.clroll) addRow("Cl roll (differential δ)","clroll");
}
function put(id,v,dec){
  const c=cells[id]; if(!c)return;
  c.textContent=v.toFixed(dec);
  c.classList.toggle("neg",v<0);
}

// live plot
const pl=$("plot"), pctx=pl.getContext("2d");
function drawPlot(){
  const w=pl.width,h=pl.height; pctx.clearRect(0,0,w,h);
  const mi=fidx(AE.mach,state.mach);
  const as=[],cns=[],cms=[];
  for(let i=0;i<AE.alpha.length;i++){
    const al=AE.alpha[i];
    if(al<-32||al>32)continue;
    as.push(al);
    cns.push(bilin(AE.tables.cn,i,mi));
    cms.push(bilin(AE.tables.cm,i,mi));
  }
  const ymin=Math.min(...cns,...cms), ymax=Math.max(...cns,...cms);
  const X=al=>8+(al-as[0])/(as[as.length-1]-as[0])*(w-16);
  const Y=v=>h-16-(v-ymin)/((ymax-ymin)||1)*(h-30);
  pctx.strokeStyle="#39404d"; pctx.lineWidth=1;
  pctx.beginPath(); pctx.moveTo(X(as[0]),Y(0)); pctx.lineTo(X(as[as.length-1]),Y(0));
  pctx.moveTo(X(0),8); pctx.lineTo(X(0),h-8); pctx.stroke();
  const line=(vals,col)=>{
    pctx.strokeStyle=col; pctx.lineWidth=1.6; pctx.beginPath();
    vals.forEach((v,i)=>i?pctx.lineTo(X(as[i]),Y(v)):pctx.moveTo(X(as[i]),Y(v)));
    pctx.stroke(); };
  line(cns,"#8fd48f"); line(cms,"#e0b25f");
  pctx.strokeStyle="#5588dd"; pctx.beginPath();
  pctx.moveTo(X(state.alpha),8); pctx.lineTo(X(state.alpha),h-8); pctx.stroke();
  pctx.font="11px system-ui";
  pctx.fillStyle="#8fd48f"; pctx.fillText("CN",w-58,16);
  pctx.fillStyle="#e0b25f"; pctx.fillText("CM",w-30,16);
}

function update(){
  $("machv").value=state.mach.toFixed(2);
  $("alphav").value=state.alpha.toFixed(1);
  if(hasCtrl) $("deltav").value=state.delta.toFixed(1);
  for(const [,key,dec] of D.readouts) put(key, evalTab(key,state.alpha,state.mach), dec);
  if(hasCtrl){
    const dcl=evalCtrl("dcl",state.delta,state.mach);
    const dcm=evalCtrl("dcm",state.delta,state.mach);
    put("dcl",dcl,3); put("dcm",dcm,3);
    put("cmtot",evalTab("cm",state.alpha,state.mach)+dcm,3);
    if(AE.ctrl.clroll) put("clroll",evalCtrl("clroll",state.delta,state.mach),4);
  }
  drawPlot(); draw();
}

resize(); resetView(); update();
</script>
</body>
</html>
"""


def main(args) -> int:
    names = args or ["control"]
    if names == ["all"]:
        names = list(CASES)
    unknown = [n for n in names if n not in CASES]
    if unknown:
        print(f"unknown case(s): {', '.join(unknown)}; "
              f"choose from: {', '.join(CASES)} or 'all'")
        return 2
    outdir = ROOT / "playground"
    outdir.mkdir(exist_ok=True)
    for n in names:
        factory, case = CASES[n]
        npz_path = ROOT / "examples" / case / "aero.npz"
        if not npz_path.is_file():
            print(f"{n}: missing {npz_path} - run examples/make_examples.py")
            return 1
        with np.load(npz_path) as aero:
            out = build_playground(factory(), aero,
                                   str(outdir / f"{n}.html"),
                                   title=f"aero playground - {n}")
        print(f"{n}: {out}")
    print("\nDouble-click a file to play.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
