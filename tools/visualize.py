"""Render a flightsim2 scenario's CSV logs to a self-contained 3-D HTML.

Produces ONE HTML file (no external libraries, no internet -- opens in any
browser) with two synced, orbit-able views:
  * TRAJECTORY  -- the world, flight paths, and every vehicle flown as its
                   real 3-D model along its path.
  * ATTITUDE    -- a chosen vehicle's actual 3-D model spun by its true
                   attitude, with a fixed reference triad and velocity vector.
A shared timeline scrubs/plays both.

Usage:
    py tools/visualize.py scenarios/intercept.json
    py tools/visualize.py scenarios/f16_maneuver.json --out output/f16.html
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from meshes import mesh_for  # noqa: E402


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


def read_log(path):
    rows = list(csv.DictReader(open(path)))
    t = [float(r["time"]) for r in rows]
    # NED position and euler; velocity for the attitude view's flow vector.
    pos = [[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows]
    eul = [[float(r["phi"]), float(r["theta"]), float(r["psi"])] for r in rows]
    vel = [[float(r["vx"]), float(r["vy"]), float(r["vz"])] for r in rows]
    return {"t": t, "pos": pos, "eul": eul, "vel": vel}


def resample(log, times):
    """Nearest-sample the log onto a shared uniform timeline."""
    src = log["t"]
    out = {"pos": [], "eul": [], "vel": [], "alive": []}
    j = 0
    for tt in times:
        while j + 1 < len(src) and src[j + 1] <= tt:
            j += 1
        alive = src[0] <= tt <= src[-1] + 1e-9
        k = min(j, len(src) - 1)
        out["pos"].append(log["pos"][k])
        out["eul"].append(log["eul"][k])
        out["vel"].append(log["vel"][k])
        out["alive"].append(alive)
    return out


def build(scenario_path):
    scn = load_jsonc(scenario_path)
    scn_dir = os.path.dirname(os.path.abspath(scenario_path))

    vehicles = []
    max_t = 0.0
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

        log = read_log(log_path)
        max_t = max(max_t, log["t"][-1] if log["t"] else 0.0)
        vehicles.append({
            "name": entry["name"],
            "dynamics": entry.get("dynamics", "six_dof"),
            "mesh": mesh_for(cfg, entry.get("dynamics", "six_dof")),
            "log": log,
        })

    if not vehicles:
        raise SystemExit("no vehicle logs found -- run the scenario first")

    # Shared uniform timeline (cap frames so the HTML stays light).
    nframes = 600
    times = [max_t * k / (nframes - 1) for k in range(nframes)]
    for v in vehicles:
        v["frames"] = resample(v["log"], times)
        del v["log"]

    return {"name": scn.get("name", "scenario"),
            "times": times, "vehicles": vehicles}


def render_html(data):
    payload = json.dumps(data, separators=(",", ":"))
    return HTML_TEMPLATE.replace("/*DATA*/", payload)


# ---------------------------------------------------------------- HTML/JS

HTML_TEMPLATE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>flightsim2 viewer</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--fg:#c9d1d9;--accent:#58a6ff;--muted:#8b949e;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif}
  header{padding:10px 16px;background:var(--panel);border-bottom:1px solid #30363d;
         display:flex;align-items:center;gap:16px;flex-wrap:wrap}
  header h1{font-size:15px;margin:0;font-weight:600}
  header .name{color:var(--accent)}
  .views{display:flex;gap:2px;height:62vh;min-height:380px}
  .view{position:relative;flex:1;background:#05070c}
  .view canvas{width:100%;height:100%;display:block;cursor:grab}
  .view canvas:active{cursor:grabbing}
  .label{position:absolute;top:8px;left:12px;font-size:12px;color:var(--muted);
         text-transform:uppercase;letter-spacing:.08em}
  .hint{position:absolute;bottom:8px;left:12px;font-size:11px;color:#5a6470}
  select{position:absolute;top:6px;right:10px;background:var(--panel);color:var(--fg);
         border:1px solid #30363d;border-radius:6px;padding:3px 6px}
  .bar{display:flex;align-items:center;gap:14px;padding:12px 16px;background:var(--panel);
       border-top:1px solid #30363d;flex-wrap:wrap}
  button{background:#21262d;color:var(--fg);border:1px solid #30363d;border-radius:6px;
         padding:6px 14px;cursor:pointer;font-size:13px}
  button:hover{border-color:var(--accent)}
  input[type=range]{flex:1;min-width:180px;accent-color:var(--accent)}
  .t{font-variant-numeric:tabular-nums;color:var(--muted);min-width:118px}
  .legend{display:flex;gap:14px;flex-wrap:wrap;padding:8px 16px;font-size:12px;color:var(--muted)}
  .legend span{display:inline-flex;align-items:center;gap:6px}
  .sw{width:11px;height:11px;border-radius:2px;display:inline-block}
</style></head>
<body>
<header>
  <h1>flightsim2 &middot; <span class="name" id="scnName"></span></h1>
  <span id="speedCtl"></span>
</header>
<div class="views">
  <div class="view"><div class="label">Trajectory</div>
    <canvas id="cTraj"></canvas>
    <div class="hint">drag to orbit &middot; scroll to zoom</div></div>
  <div class="view"><div class="label">Attitude</div>
    <canvas id="cAtt"></canvas>
    <select id="selVeh"></select>
    <div class="hint">real vehicle model at true attitude</div></div>
</div>
<div class="bar">
  <button id="play">Play</button>
  <input type="range" id="scrub" min="0" value="0" step="1">
  <span class="t" id="tlabel">t = 0.00 s</span>
</div>
<div class="legend" id="legend"></div>
<script>
const DATA = /*DATA*/;

// ---- small vec/matrix helpers ----
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scale=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const n=Math.hypot(a[0],a[1],a[2])||1;return[a[0]/n,a[1]/n,a[2]/n];};

// Body->inertial (NED) rotation R^T for 3-2-1 euler (phi,theta,psi).
function bodyToNed(e){
  const[ph,th,ps]=e;
  const cp=Math.cos(ph),sp=Math.sin(ph),ct=Math.cos(th),st=Math.sin(th),
        cs=Math.cos(ps),ss=Math.sin(ps);
  // R (inertial->body) rows; body->inertial is its transpose (applied below).
  const R=[[ct*cs,ct*ss,-st],
           [sp*st*cs-cp*ss,sp*st*ss+cp*cs,sp*ct],
           [cp*st*cs+sp*ss,cp*st*ss-sp*cs,cp*ct]];
  return v=>[R[0][0]*v[0]+R[1][0]*v[1]+R[2][0]*v[2],
             R[0][1]*v[0]+R[1][1]*v[1]+R[2][1]*v[2],
             R[0][2]*v[0]+R[1][2]*v[1]+R[2][2]*v[2]];
}
// NED (north,east,down) -> display (X=east,Y=up,Z=north), right-handed, Y up.
const nedToView=p=>[p[1],-p[2],p[0]];

const VCOLORS=["#58a6ff","#f97583","#56d364","#e3b341","#bc8cff","#39c5cf"];

// ---- camera / renderer ----
function makeRenderer(canvas){
  const ctx=canvas.getContext("2d");
  let az=-0.7,el=0.5,dist=1,tgt=[0,0,0],dpr=1;
  function resize(){
    dpr=window.devicePixelRatio||1;
    canvas.width=canvas.clientWidth*dpr;canvas.height=canvas.clientHeight*dpr;
  }
  function project(p){
    // orbit camera: rotate world by -az about Y then -el about X, then perspective.
    const c1=Math.cos(az),s1=Math.sin(az),c2=Math.cos(el),s2=Math.sin(el);
    let x=p[0]-tgt[0],y=p[1]-tgt[1],z=p[2]-tgt[2];
    let x1=c1*x+s1*z, z1=-s1*x+c1*z;
    let y2=c2*y-s2*z1, z2=s2*y+c2*z1;
    const cz=z2+dist;                        // camera looks down +Z, offset dist
    const f=1.6;
    const w=canvas.width,h=canvas.height;
    const s=(0.5*h)*f/(cz>0.01?cz:0.01);
    return {x:w/2+x1*s, y:h/2-y2*s, depth:cz, s:s};
  }
  return {
    ctx,resize,
    setView:(t,d)=>{tgt=t;dist=d;},
    orbit:(dax,del)=>{az+=dax;el=Math.max(-1.45,Math.min(1.45,el+del));},
    zoom:f=>{dist*=f;},
    project,
    get az(){return az;}, get dist(){return dist;}
  };
}

// Flat-shaded painter's-algorithm mesh draw.
function drawMesh(R,parts,placeFn,light){
  const tris=[];
  for(const part of parts){
    const wv=part.vertices.map(placeFn);
    for(const f of part.faces){
      const a=wv[f[0]],b=wv[f[1]],c=wv[f[2]];
      const pa=R.project(a),pb=R.project(b),pc=R.project(c);
      if(pa.depth<0.02&&pb.depth<0.02&&pc.depth<0.02)continue;
      const n=norm(cross(sub(b,a),sub(c,a)));
      const sh=0.35+0.65*Math.max(0,dot(n,light));
      tris.push({pa,pb,pc,z:(pa.depth+pb.depth+pc.depth)/3,col:part.color,sh});
    }
  }
  tris.sort((u,v)=>v.z-u.z);
  const ctx=R.ctx;
  for(const t of tris){
    ctx.beginPath();
    ctx.moveTo(t.pa.x,t.pa.y);ctx.lineTo(t.pb.x,t.pb.y);ctx.lineTo(t.pc.x,t.pc.y);
    ctx.closePath();
    ctx.fillStyle=shade(t.col,t.sh);ctx.fill();
    ctx.strokeStyle="rgba(0,0,0,.25)";ctx.lineWidth=0.6;ctx.stroke();
  }
}
function shade(hex,f){
  const n=parseInt(hex.slice(1),16);
  const r=Math.min(255,(n>>16&255)*f),g=Math.min(255,(n>>8&255)*f),b=Math.min(255,(n&255)*f);
  return `rgb(${r|0},${g|0},${b|0})`;
}

const LIGHT=norm([-0.4,0.8,0.5]);

// ---- scene setup ----
const V=DATA.vehicles, T=DATA.times, NF=T.length;
document.getElementById("scnName").textContent=DATA.name;

// World bounds (display space) for framing + a body-scale exaggeration.
let lo=[1e9,1e9,1e9],hi=[-1e9,-1e9,-1e9];
V.forEach(v=>v.frames.pos.forEach(p=>{const q=nedToView(p);
  for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],q[i]);hi[i]=Math.max(hi[i],q[i]);}}));
const span=Math.max(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2],10);
const center=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
// Scale each vehicle's model to a fixed fraction of the world span, so every
// vehicle is visible at trajectory scale regardless of its true size (an
// F-16 and a 2.6 m missile both read clearly) without dominating the view.
V.forEach(v=>{
  let mr=0.5;
  v.mesh.parts.forEach(p=>p.vertices.forEach(q=>mr=Math.max(mr,Math.hypot(q[0],q[1],q[2]))));
  v.modelScale=(span*0.025)/mr;
});

const trajR=makeRenderer(document.getElementById("cTraj"));
const attR =makeRenderer(document.getElementById("cAtt"));

// Attitude vehicle selector.
const sel=document.getElementById("selVeh");
V.forEach((v,i)=>{const o=document.createElement("option");o.value=i;o.textContent=v.name;sel.appendChild(o);});
let attIdx=0; sel.onchange=()=>{attIdx=+sel.value;draw();};

// Legend.
const leg=document.getElementById("legend");
V.forEach((v,i)=>{const s=document.createElement("span");
  s.innerHTML=`<span class="sw" style="background:${VCOLORS[i%VCOLORS.length]}"></span>${v.name}`;
  leg.appendChild(s);});

function drawGround(R){
  const ctx=R.ctx;
  const y=-center[1]+ (( -lo[1] ) - center[1])*0; // ground at min altitude
  const g0=lo[1];
  const step=span/10, ext=span*0.75;
  ctx.lineWidth=1;
  for(let gx=-ext;gx<=ext;gx+=step){
    const a=R.project([center[0]+gx,g0,center[2]-ext]);
    const b=R.project([center[0]+gx,g0,center[2]+ext]);
    ctx.strokeStyle="rgba(88,166,255,.10)";
    ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
  }
  for(let gz=-ext;gz<=ext;gz+=step){
    const a=R.project([center[0]-ext,g0,center[2]+gz]);
    const b=R.project([center[0]+ext,g0,center[2]+gz]);
    ctx.strokeStyle="rgba(88,166,255,.10)";
    ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
  }
}

function drawTrajectory(frame){
  const R=trajR,ctx=R.ctx;
  R.resize();
  R.setView(center, span*1.7);
  ctx.clearRect(0,0,R.ctx.canvas.width,R.ctx.canvas.height);
  drawGround(R);
  // Flight-path trails up to the current frame.
  V.forEach((v,i)=>{
    ctx.strokeStyle=VCOLORS[i%VCOLORS.length];ctx.lineWidth=1.6*R.dpr||1.6;
    ctx.globalAlpha=0.85;ctx.beginPath();
    let started=false;
    for(let k=0;k<=frame;k++){
      if(!v.frames.alive[k])continue;
      const p=R.project(nedToView(v.frames.pos[k]));
      if(!started){ctx.moveTo(p.x,p.y);started=true;}else ctx.lineTo(p.x,p.y);
    }
    ctx.stroke();ctx.globalAlpha=1;
  });
  // Vehicles as their real models, oriented + placed, scaled up.
  V.forEach((v,i)=>{
    if(!v.frames.alive[frame])return;
    const rot=bodyToNed(v.frames.eul[frame]);
    const pos=nedToView(v.frames.pos[frame]);
    const ms=v.modelScale;
    const place=vb=>{
      const ned=rot([vb[0]*ms,vb[1]*ms,vb[2]*ms]);
      return add(pos,nedToView(ned));
    };
    drawMesh(R,v.mesh.parts,place,LIGHT);
  });
}

function drawAttitude(frame){
  const R=attR,ctx=R.ctx;
  R.resize();
  const v=V[attIdx];
  // Frame the model: size from its vertex extent.
  let mr=1;
  v.mesh.parts.forEach(p=>p.vertices.forEach(q=>mr=Math.max(mr,Math.hypot(q[0],q[1],q[2]))));
  R.setView([0,0,0], mr*3.2);
  ctx.clearRect(0,0,R.ctx.canvas.width,R.ctx.canvas.height);

  const rot = v.frames.alive[frame] ? bodyToNed(v.frames.eul[frame]) : (x=>x);
  // Fixed reference triad (NED axes shown in display space).
  const axes=[["N",[1,0,0],"#56d364"],["E",[0,1,0],"#58a6ff"],["Down",[0,0,1],"#f97583"]];
  ctx.lineWidth=1.5;ctx.font=`${12}px system-ui`;
  axes.forEach(([lab,d,col])=>{
    const a=R.project([0,0,0]);
    const b=R.project(nedToView(scale(d,mr*1.8)));
    ctx.strokeStyle=col;ctx.globalAlpha=.5;
    ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
    ctx.globalAlpha=1;ctx.fillStyle=col;ctx.fillText(lab,b.x+3,b.y);
  });
  // Velocity vector (body flow) in the same frame.
  if(v.frames.alive[frame]){
    const vv=v.frames.vel[frame];
    const sp=Math.hypot(vv[0],vv[1],vv[2]);
    if(sp>1){
      const dvec=nedToView(scale([vv[0]/sp,vv[1]/sp,vv[2]/sp],mr*2.2));
      const a=R.project([0,0,0]),b=R.project(dvec);
      ctx.strokeStyle="#e3b341";ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
      ctx.fillStyle="#e3b341";ctx.fillText("V",b.x+3,b.y);
    }
  }
  // The vehicle at its true attitude.
  const place=vb=>nedToView(rot(vb));
  drawMesh(R,v.mesh.parts,place,LIGHT);
}

// ---- timeline + interaction ----
let frame=0,playing=false,speed=1,acc=0,last=0;
const scrub=document.getElementById("scrub");scrub.max=NF-1;
const tlabel=document.getElementById("tlabel");
const playBtn=document.getElementById("play");

function draw(){
  drawTrajectory(frame);drawAttitude(frame);
  tlabel.textContent=`t = ${T[frame].toFixed(2)} s`;
  scrub.value=frame;
}
scrub.oninput=()=>{frame=+scrub.value;playing=false;playBtn.textContent="Play";draw();};
playBtn.onclick=()=>{playing=!playing;playBtn.textContent=playing?"Pause":"Play";
  last=performance.now();if(playing)requestAnimationFrame(loop);};

// Playback speed control.
const sc=document.getElementById("speedCtl");
sc.innerHTML='speed <select id="spd"><option>0.25</option><option>0.5</option>'+
  '<option selected>1</option><option>2</option><option>4</option></select>x';
document.getElementById("spd").onchange=e=>{speed=+e.target.value;};

const dur=(T[NF-1]-T[0])||1;
function loop(now){
  if(!playing)return;
  const dt=(now-last)/1000;last=now;
  acc+=dt*speed*(NF-1)/dur;
  while(acc>=1){frame=(frame+1)%NF;acc-=1;}
  draw();requestAnimationFrame(loop);
}

// Mouse orbit on each canvas.
function bindOrbit(R,redraw){
  const cv=R.ctx.canvas;let drag=false,px=0,py=0;
  cv.addEventListener("mousedown",e=>{drag=true;px=e.clientX;py=e.clientY;});
  window.addEventListener("mouseup",()=>drag=false);
  window.addEventListener("mousemove",e=>{if(!drag)return;
    R.orbit((e.clientX-px)*0.01,(e.clientY-py)*0.01);px=e.clientX;py=e.clientY;redraw();});
  cv.addEventListener("wheel",e=>{e.preventDefault();
    R.zoom(e.deltaY>0?1.1:0.9);redraw();},{passive:false});
}
bindOrbit(trajR,()=>drawTrajectory(frame));
bindOrbit(attR,()=>drawAttitude(frame));
window.addEventListener("resize",draw);
draw();
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
