"""Self-contained interactive HTML viewer for a :class:`Vehicle` mesh.

:func:`vehicle_to_html` writes a single ``.html`` file with the triangle
mesh embedded as JSON and a small canvas renderer (flat shading, painter's
algorithm) inlined — no external libraries, no internet, no installed
viewer needed. Double-click the file: it opens in any browser. Drag to
rotate, wheel to zoom, right-drag (or Shift+drag) to pan, double-click to
reset.
"""
from __future__ import annotations

import json

from .geometry import vehicle_mesh
from .vehicle import Vehicle

_COLORS = {"body": [200, 200, 205], "fin": [217, 64, 64],
           "canard": [64, 115, 217]}

_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  html,body{margin:0;height:100%;overflow:hidden;background:#1c1e22;
            font-family:system-ui,sans-serif}
  canvas{display:block;width:100vw;height:100vh}
  #hud{position:fixed;left:12px;top:10px;color:#aab;font-size:13px;
       user-select:none;pointer-events:none}
  #hud b{color:#dde;font-size:15px}
</style>
</head>
<body>
<div id="hud"><b>__TITLE__</b><br>
drag: rotate &nbsp;|&nbsp; wheel: zoom &nbsp;|&nbsp; right-drag: pan
&nbsp;|&nbsp; double-click: reset</div>
<canvas id="c"></canvas>
<script>
const MESH = __MESH__;

const cv = document.getElementById("c"), ctx = cv.getContext("2d");
let W, H, DPR;
function resize(){
  DPR = window.devicePixelRatio || 1;
  W = cv.clientWidth; H = cv.clientHeight;
  cv.width = W * DPR; cv.height = H * DPR;
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  draw();
}
window.addEventListener("resize", resize);

// ---- geometry preprocessing -------------------------------------------
let lo = [ 1e30, 1e30, 1e30], hi = [-1e30,-1e30,-1e30];
for (const p of MESH.parts)
  for (const v of p.v)
    for (let k = 0; k < 3; k++){
      lo[k] = Math.min(lo[k], v[k]); hi[k] = Math.max(hi[k], v[k]);
    }
const ctr = [0,1,2].map(k => (lo[k] + hi[k]) / 2);
const size = Math.max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]);

// flatten triangles: [part, i0, i1, i2]
const tris = [];
MESH.parts.forEach((p, pi) => { for (const f of p.f) tris.push([pi, f[0], f[1], f[2]]); });

// ---- view state --------------------------------------------------------
let R, zoom, panX, panY;
function reset(){
  // start with the nose left, slight tumble
  R = mul(rotY(-0.5), rotX(0.35));
  zoom = 1.0; panX = 0; panY = 0;
  draw();
}
function rotX(a){ const c=Math.cos(a), s=Math.sin(a);
  return [1,0,0, 0,c,-s, 0,s,c]; }
function rotY(a){ const c=Math.cos(a), s=Math.sin(a);
  return [c,0,s, 0,1,0, -s,0,c]; }
function mul(A,B){ const M = new Array(9);
  for (let r=0;r<3;r++) for (let c=0;c<3;c++)
    M[3*r+c] = A[3*r]*B[c] + A[3*r+1]*B[3+c] + A[3*r+2]*B[6+c];
  return M; }
function apply(M,v){ return [
  M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
  M[3]*v[0]+M[4]*v[1]+M[5]*v[2],
  M[6]*v[0]+M[7]*v[1]+M[8]*v[2]]; }

// ---- rendering ---------------------------------------------------------
function draw(){
  if (!R || !W) return;          // not initialized yet (resize before reset)
  ctx.clearRect(0, 0, W, H);
  const scale = 0.85 * Math.min(W, H) / size * zoom;
  const camd = 3.0 * size;                      // camera distance
  const f = camd * scale;                       // projection factor

  // transform all vertices once per part
  const P = [], Z = [];
  for (const p of MESH.parts){
    const pts = new Array(p.v.length), zs = new Array(p.v.length);
    for (let i = 0; i < p.v.length; i++){
      const v = p.v[i];
      const w = apply(R, [v[0]-ctr[0], v[1]-ctr[1], v[2]-ctr[2]]);
      const z = w[2] + camd;
      pts[i] = [W/2 + panX + f * w[0] / z, H/2 + panY - f * w[1] / z];
      zs[i] = w;
    }
    P.push(pts); Z.push(zs);
  }

  // depth sort (painter)
  const order = tris.map((t, i) => {
    const z = Z[t[0]];
    return [ (z[t[1]][2] + z[t[2]][2] + z[t[3]][2]) / 3, i ];
  }).sort((a, b) => a[0] - b[0]);

  const light = [0.45, 0.5, 0.74];
  for (const [, ti] of order){
    const [pi, a, b, c] = tris[ti];
    const pts = P[pi], z = Z[pi];
    // flat shading from the rotated face normal
    const u = [z[b][0]-z[a][0], z[b][1]-z[a][1], z[b][2]-z[a][2]];
    const v = [z[c][0]-z[a][0], z[c][1]-z[a][1], z[c][2]-z[a][2]];
    let n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]];
    const nl = Math.hypot(n[0], n[1], n[2]) || 1;
    let d = (n[0]*light[0] + n[1]*light[1] + n[2]*light[2]) / nl;
    d = 0.35 + 0.62 * Math.abs(d);
    const col = MESH.parts[pi].color;
    ctx.fillStyle = `rgb(${col[0]*d|0},${col[1]*d|0},${col[2]*d|0})`;
    ctx.beginPath();
    ctx.moveTo(pts[a][0], pts[a][1]);
    ctx.lineTo(pts[b][0], pts[b][1]);
    ctx.lineTo(pts[c][0], pts[c][1]);
    ctx.closePath();
    ctx.fill();
  }
}

// ---- interaction -------------------------------------------------------
let drag = null;
cv.addEventListener("mousedown", e => {
  drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
});
window.addEventListener("mousemove", e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  drag.x = e.clientX; drag.y = e.clientY;
  if (drag.pan){ panX += dx; panY += dy; }
  else R = mul(mul(rotY(dx * 0.008), rotX(dy * 0.008)), R);
  draw();
});
window.addEventListener("mouseup", () => drag = null);
cv.addEventListener("contextmenu", e => e.preventDefault());
cv.addEventListener("wheel", e => {
  e.preventDefault();
  zoom *= Math.exp(-e.deltaY * 0.001);
  draw();
}, { passive: false });
cv.addEventListener("dblclick", reset);

resize(); reset();
</script>
</body>
</html>
"""


def vehicle_to_html(v: Vehicle, path: str, title: str = "") -> str:
    """Write a standalone interactive 3-D viewer page and return ``path``."""
    parts = []
    for name, verts, faces in vehicle_mesh(v):
        kind = ("body" if name == "body"
                else "canard" if name.startswith("canard") else "fin")
        parts.append({
            "name": name,
            "color": _COLORS[kind],
            "v": [[round(float(c), 5) for c in p] for p in verts],
            "f": [[int(a), int(b), int(c)] for a, b, c in faces],
        })
    html = (_TEMPLATE
            .replace("__TITLE__", title or "pydatcom vehicle")
            .replace("__MESH__", json.dumps({"parts": parts},
                                            separators=(",", ":"))))
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    return path
