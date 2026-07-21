// Headless attitude check for the generated viewer: extracts the pure frame
// math between the PURE markers from a view.html and asserts the NED/body ->
// scene mapping on known attitudes. Run: node tools/check_viz.mjs <view.html>
import { readFileSync } from "fs";

const path = process.argv[2] ?? "output/aam_intercept/view.html";
const html = readFileSync(path, "utf-8");
const m = html.match(/\/\*PURE-BEGIN\*\/([\s\S]*?)\/\*PURE-END\*\//);
if (!m) { console.error("PURE block not found in " + path); process.exit(1); }
const fns = new Function(m[1] + "; return {nedToScene, bodyToNed, bodyToSceneMatrix};")();

let failures = 0;
function near(a, b, msg) {
  if (Math.abs(a - b) > 1e-9) { console.error(`FAIL ${msg}: ${a} vs ${b}`); failures++; }
}
function mulv(R, v) {   // row-major 3x3 * vec3
  return [R[0]*v[0]+R[1]*v[1]+R[2]*v[2],
          R[3]*v[0]+R[4]*v[1]+R[5]*v[2],
          R[6]*v[0]+R[7]*v[1]+R[8]*v[2]];
}
function checkVec(got, want, msg) {
  for (let i = 0; i < 3; i++) near(got[i], want[i], `${msg}[${i}]`);
}

// Scene frame: x = east, y = up, z = north.
checkVec(fns.nedToScene([1,0,0]), [0,0,1], "north->scene");
checkVec(fns.nedToScene([0,1,0]), [1,0,0], "east->scene");
checkVec(fns.nedToScene([0,0,1]), [0,-1,0], "down->scene");

// Level flight, nose north: body x -> scene z (north), body y (right) ->
// scene x (east), body z (down) -> scene -y.
let R = fns.bodyToSceneMatrix(0,0,0);
checkVec(mulv(R,[1,0,0]), [0,0,1], "level bodyX");
checkVec(mulv(R,[0,1,0]), [1,0,0], "level bodyY");
checkVec(mulv(R,[0,0,1]), [0,-1,0], "level bodyZ");

// Pitch 90 up: nose points UP (+scene y).
R = fns.bodyToSceneMatrix(0, Math.PI/2, 0);
checkVec(mulv(R,[1,0,0]), [0,1,0], "pitch90 bodyX");

// Yaw 90 (nose east): body x -> scene x.
R = fns.bodyToSceneMatrix(0, 0, Math.PI/2);
checkVec(mulv(R,[1,0,0]), [1,0,0], "yaw90 bodyX");

// Roll 90 right: body y (right wing) points DOWN (-scene y).
R = fns.bodyToSceneMatrix(Math.PI/2, 0, 0);
checkVec(mulv(R,[0,1,0]), [0,-1,0], "roll90 bodyY");
checkVec(mulv(R,[1,0,0]), [0,0,1], "roll90 bodyX unchanged");

// Composite: yaw 90 then pitch 45 -- nose east and 45 deg up.
R = fns.bodyToSceneMatrix(0, Math.PI/4, Math.PI/2);
const s = Math.SQRT1_2;
checkVec(mulv(R,[1,0,0]), [s, s, 0], "yaw90pitch45 bodyX");

if (failures) { console.error(failures + " failures"); process.exit(1); }
console.log("check_viz: attitude-mapping checks passed");

// ---------------------------------------------------------------- app smoke
// Run the WHOLE inline app in node: real three.js math/scene classes, stubbed
// renderer/DOM/controls. Steps the animation loop and asserts the vehicle and
// hinged-part matrices respond to the logged data.
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(x => x[1]);
if (scripts.length < 4) { console.error("expected 4 inline scripts"); process.exit(1); }
const [threeSrc, /*orbit*/, /*uplot*/, appSrc] = scripts;

const fakeStyle = () => new Proxy({}, { get: () => () => {}, set: () => true });
const fakeEl = () => new Proxy(function(){}, {
  get(t, k) {
    if (k === "style") return fakeStyle();
    if (k === "value") return "0";
    if (k === "max") return 0;
    if (k === "children") return [];
    if (k === "clientWidth") return 800;
    if (k === "clientHeight") return 600;
    if (k === Symbol.toPrimitive) return () => 0;
    return typeof t[k] !== "undefined" ? t[k] : fakeEl();
  },
  set() { return true; },
  apply() { return fakeEl(); },
});
const rafQueue = [];
const sandbox = {
  console, Math, JSON, Proxy, Float32Array, Uint16Array, Uint32Array,
  requestAnimationFrame: cb => rafQueue.push(cb),
  document: new Proxy({}, { get: (t, k) =>
    k === "createElementNS" || k === "createElement"
      ? () => fakeEl()
      : (k === "getElementById" ? () => fakeEl() : fakeEl()) }),
  window: { addEventListener() {}, devicePixelRatio: 1 },
  self: {},
};
sandbox.globalThis = sandbox;

import vm from "vm";
vm.createContext(sandbox);
vm.runInContext(threeSrc, sandbox);                     // real THREE (UMD -> self)
const THREE = sandbox.self.THREE ?? sandbox.THREE;
sandbox.THREE = THREE;
THREE.WebGLRenderer = class {
  constructor() { this.domElement = fakeEl(); }
  setPixelRatio() {} setSize() {} render() {}
};
THREE.OrbitControls = class {
  constructor() { this.target = new THREE.Vector3(); this.enableDamping = true; }
  update() {}
};
vm.runInContext(appSrc, sandbox);                        // the viewer app itself

// Drive a few animation frames.
for (let i = 0; i < 5 && rafQueue.length; i++) rafQueue.shift()(i * 100);

// Reach into the app state via the context.
const vObjs = vm.runInContext("vObjs", sandbox);
const DATA = vm.runInContext("DATA", sandbox);
if (!vObjs || !vObjs.length) { console.error("FAIL: no vehicle objects built"); process.exit(1); }

// Apply two distant frames and require the group matrix to move + stay finite.
vm.runInContext("applyFrame(0)", sandbox);
const m0 = vObjs[0].grp.matrix.elements.slice();
vm.runInContext(`applyFrame(${DATA.times.length - 100})`, sandbox);
const m1 = vObjs[0].grp.matrix.elements.slice();
if (m0.every((v, i) => v === m1[i])) { console.error("FAIL: vehicle matrix static"); failures++; }
if (![...m0, ...m1].every(Number.isFinite)) { console.error("FAIL: non-finite matrix"); failures++; }

// Articulation: find a frame where a hinge channel is non-zero and require
// the hinged part's local matrix to differ from identity.
const hingedVeh = vObjs.find(o => o.hinged.length);
if (hingedVeh) {
  const v = hingedVeh.v;
  const hingeChans = new Set(
    hingedVeh.hinged.flatMap(h => h.hinges.map(hg => hg.channel)));
  const chanArrs = Object.entries(v.frames.chan ?? {})
    .filter(([name]) => hingeChans.has(name));
  let kActive = -1;
  outer:
  for (let k = 0; k < DATA.times.length; k++)
    for (const [, arr] of chanArrs)
      if (arr && Math.abs(arr[k]) > 1e-3) { kActive = k; break outer; }
  if (kActive >= 0) {
    vm.runInContext(`applyFrame(${kActive})`, sandbox);
    const I = new THREE.Matrix4().elements;
    const moved = hingedVeh.hinged.some(h =>
      h.mesh.matrix.elements.some((e, i) => Math.abs(e - I[i]) > 1e-6));
    if (!moved) { console.error("FAIL: hinged parts never deflect"); failures++; }
    else console.log(`check_viz: articulation responds (frame ${kActive})`);
  } else {
    console.log("check_viz: no active channels in this log (articulation untested)");
  }
}

// Overlays: at a thrust-active frame the plume shows; toggling kills it.
const vf0 = vObjs[0].v.frames;
const kT = vf0.thrust ? vf0.thrust.findIndex(t => t > 0) : -1;
if (kT >= 0) {
  vm.runInContext(`applyFrame(${kT})`, sandbox);
  if (!vObjs[0].plume.visible) { console.error("FAIL: plume not visible under thrust"); failures++; }
  vm.runInContext(`ovState.plume = false; applyFrame(${kT})`, sandbox);
  if (vObjs[0].plume.visible) { console.error("FAIL: plume toggle ignored"); failures++; }
  vm.runInContext(`ovState.plume = true`, sandbox);
}
// Triad/velocity glyphs on the focus vehicle at a live frame.
vm.runInContext("applyFrame(5)", sandbox);
if (!vObjs[0].triad.visible) { console.error("FAIL: triad not visible on focus"); failures++; }
if (!vObjs[0].velArrow.visible) { console.error("FAIL: velocity arrow not visible"); failures++; }
// LOS shows when the focus vehicle has a target.
const losLine = vm.runInContext("losLine", sandbox);
const hasTarget = DATA.vehicles.some(v => v.target &&
  DATA.vehicles.some(w => w.name === v.target));
if (hasTarget) {
  const pi = DATA.vehicles.findIndex(v => v.target);
  vm.runInContext(`focusSel.value=${pi}`, sandbox);   // fake select: set ignored...
  // fake elements drop assignments; drive via ovState only if select stub works
  if (pi === 0) {
    vm.runInContext("applyFrame(5)", sandbox);
    if (!losLine.visible) { console.error("FAIL: LOS not visible for paired focus"); failures++; }
  } else {
    console.log("check_viz: LOS untested (pursuer is not vehicle 0; select stub is inert)");
  }
}

// Preset library: every vehicle with a full log must offer the core stories;
// paired vehicles must offer Intercept; component wrenches must give Allocation.
const presets = vm.runInContext("presetsFor(DATA.vehicles[0])", sandbox);
const ids = presets.map(p => p.id);
for (const want of ["tracking", "rates", "airdata", "traj", "prop"])
  if (!ids.includes(want)) { console.error("FAIL: preset missing: " + want); failures++; }
if (DATA.vehicles[0].components?.length && !ids.includes("alloc"))
  { console.error("FAIL: alloc preset missing despite component data"); failures++; }
if (DATA.vehicles[0].target && "range" in DATA.vehicles[0].series && !ids.includes("intercept"))
  { console.error("FAIL: intercept preset missing for paired vehicle"); failures++; }
for (const p of presets)
  for (const pane of p.panes)
    for (const sp of pane.series)
      if (!(sp.col in DATA.vehicles[0].series))
        { console.error(`FAIL: preset ${p.id} references missing column ${sp.col}`); failures++; }
console.log("check_viz: presets ok (" + ids.join(", ") + ")");

if (failures) { console.error(failures + " failures"); process.exit(1); }
console.log("check_viz: app smoke passed (" + path + ")");
