/* =====================================================================
 * 三维仿真回放 —— 用 10 月 7 日动画里的 CAD 模型与三维引擎，
 * 按仿真器（单片机比赛程序在电脑上运行）记录的每一帧数据驱动小车、机械臂、夹爪和物料。
 * 这段代码被拼进原动画的脚本包里运行，所以能直接用包里的 three.js（名字被压缩过，下面先起别名）。
 * ===================================================================== */
(() => {
"use strict";
const T = {
  Scene: cr, Color: Yt, WebGLRenderer: Yo, PCFShadow: $a, SRGB: Te, ToneMap: so,
  PerspectiveCamera: Re, OrbitControls: $o, HemisphereLight: Er, DirectionalLight: ws,
  Std: yr, DoubleSide: un, Mesh: ke, Box: Ln, Cyl: Un, CanvasTexture: Fi, Sprite: hr, SpriteMaterial: ys,
  Plane: bn, Basic: Mn, BufferGeometry: _e, V3: F, Line: Ss, LineBasic: Ni, Ring: En, Group: Ue,
  Attr: Ce, Sphere: ki, Lathe: zi, V2: at, M4: ie, LineDashed: Mr,
};
const KIN = Gr(Gc());          // 原动画的机械臂/夹爪运动学（solvePose / solveJaws / padAnchors）
const QRLIB = Gr(fd());        // 二维码生成
const MODEL = window.CAD_MODEL;
const $ = (id) => document.getElementById(id);
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const FIELD = 2400;
const COLORS = { 1: ["红", "#e53935"], 2: ["黄", "#f2c200"], 3: ["蓝", "#1e5fd6"], 4: ["绿", "#2e9d4f"], 5: ["黑", "#25292c"], 6: ["浅蓝", "#66c2ee"] };
const LOCS = { 0: "待补料", 1: "转盘", 2: "场地", 3: "车载槽", 4: "夹爪上", 5: "掉落" };
const sceneXZ = (x, y) => [x, FIELD - y];          // 场地坐标（x 右, y 上）→ 三维（X, Z）

/* ---------------- 渲染器、相机、灯光 ---------------- */
const viewport = $("viewport"), canvas = $("scene");
const scene = new T.Scene();
scene.background = new T.Color(css("--scene-bg") || "#e6ecec");
const renderer = new T.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.6));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = T.PCFShadow;
renderer.outputColorSpace = T.SRGB;
renderer.toneMapping = T.ToneMap;
renderer.toneMappingExposure = 1.12;
const cam = new T.PerspectiveCamera(37, 1, 2, 16000);
const insetCam = new T.PerspectiveCamera(35, 1.36, 1, 5000);
cam.position.set(3500, 3500, 3900);
const controls = new T.OrbitControls(cam, canvas);
controls.target.set(1200, 0, 1130);
controls.enableDamping = true; controls.dampingFactor = 0.12;
controls.maxPolarAngle = Math.PI * 0.48; controls.minDistance = 300; controls.maxDistance = 8000;
scene.add(new T.HemisphereLight("#ffffff", "#9aabb0", 2.2));
const sun = new T.DirectionalLight("#fff8e8", 3.1);
sun.position.set(500, 2800, 400); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -1800, right: 1800, top: 1800, bottom: -1800, near: 10, far: 6000 });
sun.target.position.set(1200, 0, 1200); sun.shadow.bias = -3e-4; sun.shadow.normalBias = 0.3;
scene.add(sun, sun.target);
const fill = new T.DirectionalLight("#b8deee", 1.1); fill.position.set(2600, 1200, -300); scene.add(fill);

const mat = (c, m = 0.1, r = 0.6) => new T.Std({ color: c, metalness: m, roughness: r, side: T.DoubleSide });
function add(geo, material, parent = scene) {
  const m = new T.Mesh(geo, material); m.castShadow = true; m.receiveShadow = true; parent.add(m); return m;
}
function box(w, h, d, x, y, z, material, parent = scene) { const m = add(new T.Box(w, h, d), material, parent); m.position.set(x, y, z); return m; }
function cyl(rt, rb, h, x, y, z, material, parent = scene, seg = 40) { const m = add(new T.Cyl(rt, rb, h, seg), material, parent); m.position.set(x, y, z); return m; }
function textTexture(text, color = "#283e49", bg = null, px = 42) {
  const c = document.createElement("canvas"), g = c.getContext("2d");
  g.font = `600 ${px}px Microsoft YaHei, PingFang SC, sans-serif`;
  const w = Math.ceil(g.measureText(text).width + 32);
  c.width = w; c.height = Math.ceil(px * 1.65);
  g.font = `600 ${px}px Microsoft YaHei, PingFang SC, sans-serif`; g.textBaseline = "middle";
  if (bg) { g.fillStyle = bg; const r = c.height * 0.3; g.beginPath(); g.moveTo(r, 0); g.arcTo(c.width, 0, c.width, c.height, r); g.arcTo(c.width, c.height, 0, c.height, r); g.arcTo(0, c.height, 0, 0, r); g.arcTo(0, 0, c.width, 0, r); g.fill(); }
  g.fillStyle = color; g.fillText(text, 16, c.height / 2);
  const tex = new T.CanvasTexture(c); tex.colorSpace = T.SRGB;
  return { tex, w, h: c.height };
}
const floatLabels = [];
function sprite(text, x, y, z, width = 200, color = "#324a55", bg = "rgba(255,255,255,0.86)", parent = scene) {
  const { tex, w, h } = textTexture(text, color, bg);
  const s = new T.Sprite(new T.SpriteMaterial({ map: tex, depthWrite: false, toneMapped: false }));
  s.position.set(x, y, z); s.scale.set(width, (width * h) / w, 1); parent.add(s); floatLabels.push(s); return s;
}
function floorText(text, x, z, width = 170, color = "#42515b", rot = 0, parent = scene) {
  const { tex, w, h } = textTexture(text, color);
  const m = new T.Mesh(new T.Plane(width, (width * h) / w), new T.Basic({ map: tex, transparent: true, depthWrite: false }));
  m.rotation.x = -Math.PI / 2; m.rotation.z = rot; m.position.set(x, 1.1, z); parent.add(m); return m;
}
function polyline(points, color, opacity = 0.6, parent = scene, dashed = false) {
  const g = new T.BufferGeometry().setFromPoints(points.map((p) => new T.V3(...p)));
  const m = dashed ? new T.LineDashed({ color, dashSize: 22, gapSize: 16, transparent: true, opacity }) : new T.LineBasic({ color, transparent: true, opacity });
  const l = new T.Line(g, m); if (dashed) l.computeLineDistances(); parent.add(l); return l;
}

/* ---------------- 场地（不随数据变化的部分） ---------------- */
box(2400, 10, 2400, 1200, -6, 1200, mat("#aeb9be", 0.02, 0.98));
box(2900, 14, 3000, 1200, -18, 1150, mat("#d2dddd", 0.05, 0.85));
const yellow = mat("#f4edbf", 0.01, 0.98), white = mat("#f8faf8", 0.01, 0.95), blue = mat("#355edb", 0.1, 0.55);
for (const cx of [775, 1625]) for (const cy of [775, 1625]) {
  const [X, Z] = sceneXZ(cx, cy); box(450, 1, 450, X, 0.1, Z, yellow); floorText("禁行区", X, Z, 200, "#b3a56e");
}
for (const [zone, cy] of [[1, 2250], [2, 150]]) {
  const [X, Z] = sceneXZ(2250, cy); box(300, 1.2, 300, X, 0.5, Z, blue); floorText("启停区 " + zone, X, Z, 230, "#ffffff");
}
{ const [X, Z] = sceneXZ(1200, 75); box(580, 0.9, 150, X, 0.5, Z, white); }
{ const [X, Z] = sceneXZ(75, 1200); box(150, 0.9, 580, X, 0.5, Z, white); }
polyline([[0, 0.8, 0], [2400, 0.8, 0], [2400, 0.8, 2400], [0, 0.8, 2400], [0, 0.8, 0]], "#748c95");
sprite("原料转盘", 1200, 230, -230, 250);
sprite("暂存区 · 同色码垛", -150, 95, 1200, 330);
sprite("粗加工区", 1200, 75, 2560, 250);
floorText("2400 × 2400 mm", 1180, 2520, 520, "#6b838d");

/* 原料转盘：底座固定，圆盘随数据转动 */
const turntable = new T.Group(); scene.add(turntable);
const ttDisc = new T.Group(); turntable.add(ttDisc);
cyl(96, 104, 80, 0, 40, 0, mat("#697981", 0.6, 0.48), turntable);
cyl(150, 150, 10, 0, 85, 0, mat("#e1e5e5", 0.55, 0.4), ttDisc);
for (let k = 0; k < 3; k++) {
  const a = ((37 + 120 * k) * Math.PI) / 180;
  const r = add(new T.Ring(26, 28, 40), mat("#879da4", 0.3, 0.6), ttDisc);
  r.rotation.x = -Math.PI / 2; r.position.set(Math.cos(a) * 95, 90.6, -Math.sin(a) * 95);
}

/* 二维码板 */
const qrBoard = new T.Group(); qrBoard.position.set(2394, 150, 1200); scene.add(qrBoard);
box(7, 210, 297, 0, 0, 0, mat("#fffdfa", 0.01, 0.6), qrBoard);
box(10, 150, 10, 0, -150, 0, mat("#536577", 0.65, 0.38), qrBoard);
const qrCanvas = document.createElement("canvas"); qrCanvas.width = qrCanvas.height = 256;
const qrTex = new T.CanvasTexture(qrCanvas); qrTex.colorSpace = T.SRGB;
const qrFace = add(new T.Plane(80, 80), new T.Basic({ map: qrTex }), qrBoard);
qrFace.rotation.y = -Math.PI / 2; qrFace.position.x = -4.1;
sprite("二维码板", 2440, 300, 1200, 210);
function drawQR(text) {
  const g = qrCanvas.getContext("2d"); g.fillStyle = "#fff"; g.fillRect(0, 0, 256, 256);
  try {
    const q = (0, QRLIB.default)(0, "M"); q.addData(text); q.make();
    const n = q.getModuleCount(), s = 256 / (n + 8);
    g.fillStyle = "#172c35";
    for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) if (q.isDark(r, c)) g.fillRect((c + 4) * s, (r + 4) * s, Math.ceil(s), Math.ceil(s));
  } catch (e) { g.fillStyle = "#172c35"; g.font = "bold 28px monospace"; g.fillText(text, 10, 128); }
  qrTex.needsUpdate = true;
}

/* ---------------- 小车：CAD 网格 ---------------- */
const robot = new T.Group(); scene.add(robot);
const parts = {};
for (const k of ["fixed", "waist", "upper", "lower", "gripper"]) { parts[k] = new T.Group(); robot.add(parts[k]); parts[k].matrixAutoUpdate = false; }
const MATS = {
  metal: mat("#b8c3ca", 0.7, 0.34), post: mat("#d1ad6c", 0.64, 0.36), inner: mat("#6c8c96", 0.45, 0.53),
  arm: mat("#dc8d35", 0.32, 0.43), grip: mat("#c1cbd0", 0.7, 0.38), gear: mat("#3f5663", 0.52, 0.4),
  servo: mat("#434d5a", 0.2, 0.5), base: mat("#788f9b", 0.6, 0.4), tray: mat("#628d99", 0.4, 0.48),
};
const dark = mat("#242d35", 0.05, 0.76), steel = mat("#c6d2dc", 0.75, 0.31), slate = mat("#536577", 0.65, 0.38);
const gripperMeshes = [];
for (const mesh of MODEL.meshes) {
  let bytes = mesh.bytes;                      // 打包时已把网格数据压缩，页面打开时解压到这里
  if (!bytes) { const bin = atob(mesh.data); bytes = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i); }
  const v = new Int16Array(bytes.buffer, bytes.byteOffset, mesh.count * 6), pos = new Float32Array(mesh.count * 3), nor = new Float32Array(mesh.count * 3);
  for (let i = 0; i < mesh.count; i++) for (let c = 0; c < 3; c++) { pos[i * 3 + c] = v[i * 6 + c] / 100; nor[i * 3 + c] = v[i * 6 + 3 + c] / 32767; }
  const geo = new T.BufferGeometry();
  geo.setAttribute("position", new T.Attr(pos, 3)); geo.setAttribute("normal", new T.Attr(nor, 3));
  const m = add(geo, MATS[mesh.material] || MATS.metal, parts[mesh.group]);
  m.name = mesh.name;
  if (mesh.group === "gripper") { m.matrixAutoUpdate = false; gripperMeshes.push({ m, body: mesh.body }); }
}
const pads = KIN.padAnchors(MODEL.gripper).map((p) => {
  const m = add(new T.Sphere(MODEL.gripper.padRadius, 16, 12), dark, parts.gripper); m.matrixAutoUpdate = false; return { ...p, m };
});
/* 麦克纳姆轮（原动画的示意几何）：模型坐标里车头是 -Z，左边是 -X */
const wheels = [];
function makeWheel(sx, sz) {
  const g = new T.Group(); g.position.set(sx * 126, 30, sz * 78); robot.add(g);
  const rotor = new T.Group(); g.add(rotor);
  cyl(3, 3, 27, sx * -10, 0, 0, steel, g).rotation.z = Math.PI / 2;
  for (const c of [-10, 10]) cyl(20.8, 20.8, 3, c, 0, 0, steel, rotor).rotation.z = Math.PI / 2;
  cyl(16, 16, 20, 0, 0, 0, slate, rotor).rotation.z = Math.PI / 2;
  cyl(7, 7, 31, 0, 0, 0, steel, rotor).rotation.z = Math.PI / 2;
  const o = sx * sz;
  for (let c = 0; c < 8; c++) {
    const a = (c * Math.PI) / 4, prof = [[-16, 3], [-11, 5], [-6, 6.5], [0, 7], [6, 6.5], [11, 5], [16, 3]].map((p) => new T.V2(p[1], p[0]));
    const roller = add(new T.Lathe(prof, 14), dark, rotor);
    roller.position.set(0, Math.cos(a) * 23, Math.sin(a) * 23);
    roller.quaternion.setFromUnitVectors(new T.V3(0, 1, 0), new T.V3(1, -o * Math.sin(a), o * Math.cos(a)).normalize());
  }
  cyl(12.5, 12.5, 37, sx * 82.5, 30, sz * 78, mat("#8f9b9f", 0.7, 0.4), robot).rotation.z = Math.PI / 2;
  return rotor;
}
// 顺序与仿真数据一致：左前、右前、左后、右后
wheels.push(makeWheel(-1, -1), makeWheel(1, -1), makeWheel(-1, 1), makeWheel(1, 1));
/* 车头的显示屏和摄像头 */
const screenCanvas = document.createElement("canvas"); screenCanvas.width = 768; screenCanvas.height = 540;
const screenTex = new T.CanvasTexture(screenCanvas); screenTex.colorSpace = T.SRGB;
box(100, 76, 5, -38, 131, -105, mat("#283e4b", 0.15, 0.45), robot);
for (const x of [-73, -3]) { box(4, 6, 4, x, 92, -100, slate, robot); box(4, 4, 8, x, 95, -103, slate, robot); }
const screenFace = add(new T.Plane(96, 72), new T.Basic({ map: screenTex }), robot); screenFace.position.set(-38, 131, -102.4); screenFace.rotation.y = Math.PI;
box(28, 18, 2, 50, 70, -80, mat("#2d6555", 0.1, 0.6), robot);
cyl(6, 6, 16, 50, 71, -89, dark, robot, 32).rotation.x = Math.PI / 2;
cyl(4.5, 4.5, 0.6, 50, 71, -97.4, mat("#245677", 0.5, 0.18), robot, 32).rotation.x = Math.PI / 2;
const halo = add(new T.Ring(165, 170, 100), new T.Basic({ color: "#218b90", transparent: true, opacity: 0.4, depthWrite: false }));
halo.rotation.x = -Math.PI / 2; halo.castShadow = false;

/* 测距传感器的光束（挂在车上）：扇形 = 视场 25°，长度 = 这一刻的读数；没看到东西时画到最大量程、颜色淡 */
let tofGroup = null, tofBeams = [];
const beamOn = new T.Basic({ color: "#f08a24", transparent: true, opacity: 0.42, depthWrite: false, side: T.DoubleSide });
const beamOff = new T.Basic({ color: "#5fb7a8", transparent: true, opacity: 0.13, depthWrite: false, side: T.DoubleSide });
function sectorGeo(halfDeg) {
  const pts = [], h = (halfDeg * Math.PI) / 180, N = 8;
  for (let k = 0; k < N; k++) {
    const a0 = -h + (2 * h * k) / N, a1 = -h + (2 * h * (k + 1)) / N;
    pts.push(0, 0, 0, Math.sin(a0), 0, -Math.cos(a0), Math.sin(a1), 0, -Math.cos(a1));
  }
  const g = new T.BufferGeometry(); g.setAttribute("position", new T.Attr(new Float32Array(pts), 3)); return g;
}
function buildTof() {
  if (tofGroup) robot.remove(tofGroup);
  tofGroup = new T.Group(); robot.add(tofGroup); tofBeams = [];
  const tf = D.tof; if (!tf || !tf.mounts) return;
  const geo = sectorGeo(tf.fov / 2);
  for (const [mx, my, yaw] of tf.mounts) {
    const g = new T.Group(); g.position.set(-my, tf.z, -mx); g.rotation.y = (yaw * Math.PI) / 180; tofGroup.add(g);
    box(14, 10, 6, 0, 0, 0, mat("#20343c", 0.3, 0.5), g);                 // 传感器小板
    const beam = new T.Mesh(geo, beamOff); beam.castShadow = false; beam.renderOrder = 5; g.add(beam);
    tofBeams.push(beam);
  }
  tofGroup.visible = $("tTof").checked;
}

/* 物料：上 Ø30、下 Ø50、高 60（上下各 8mm 直段），按规则图 */
const matProfile = [[0, 0], [25, 0], [25, 8], [15, 52], [15, 60], [0, 60]].map((p) => new T.V2(p[0], p[1]));
const pieces = Array.from({ length: 6 }, () => {
  const g = new T.Group(); scene.add(g);
  const body = add(new T.Lathe(matProfile, 48), mat("#e14d48", 0.03, 0.42), g);
  const cap = add(new T.Ring(10, 14, 40), steel, g); cap.rotation.x = -Math.PI / 2; cap.position.y = 60.05; cap.castShadow = false;
  return { g, body };
});

/* ---------------- 每份数据各自的物体（圆环、车道、轨迹、越线标记） ---------------- */
let dataGroup = new T.Group(); scene.add(dataGroup);
let trail = null, laneGroup = null, violMarker = null, obsMeshes = [], foundMarks = [];
const RING_R = [26.5, 29, 32.5, 37.5, 42.5, 47.5];
const EDGES = [[0, 1], [1, 2], [3, 4], [4, 5], [6, 7], [7, 8], [0, 3], [3, 6], [1, 4], [4, 7], [2, 5], [5, 8]];
function buildDataObjects() {
  scene.remove(dataGroup);
  dataGroup.traverse((o) => { o.geometry && o.geometry.dispose(); });
  dataGroup = new T.Group(); scene.add(dataGroup);
  for (const [z, n, x, y] of D.field.rings) {
    const [X, Z] = sceneXZ(x, y);
    RING_R.forEach((r, i) => { const m = add(new T.Ring(r - 0.75, r + 0.75, 64), mat("#2b373e", 0, 1), dataGroup); m.rotation.x = -Math.PI / 2; m.position.set(X, 1.4 + i * 0.01, Z); m.castShadow = false; });
    if (z === 0) floorText(n + "号", X, FIELD + 45, 60, "#273a45", 0, dataGroup); else floorText(n + "号", -55, Z, 60, "#273a45", 0, dataGroup);
  }
  laneGroup = new T.Group(); dataGroup.add(laneGroup);
  for (const [a, b] of EDGES) {
    const p = D.field.nodes[a], q = D.field.nodes[b];
    polyline([[p[0], 1.8, FIELD - p[1]], [q[0], 1.8, FIELD - q[1]]], "#3f8a95", 0.55, laneGroup, true);
  }
  laneGroup.visible = $("tLanes").checked;
  // 轨迹：每帧一个点，颜色由青渐变到橙，按当前时间只画到当前帧
  const n = D.frames.length, pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
  const c0 = new T.Color("#11a1a8"), c1 = new T.Color("#e5892b"), c = new T.Color();
  D.frames.forEach((f, i) => { pos[i * 3] = f[1]; pos[i * 3 + 1] = 3; pos[i * 3 + 2] = FIELD - f[2]; c.copy(c0).lerp(c1, i / Math.max(1, n - 1)); col.set([c.r, c.g, c.b], i * 3); });
  const tg = new T.BufferGeometry(); tg.setAttribute("position", new T.Attr(pos, 3)); tg.setAttribute("color", new T.Attr(col, 3));
  trail = new T.Line(tg, new T.LineBasic({ vertexColors: true })); trail.frustumCulled = false; dataGroup.add(trail);
  trail.visible = $("tTrail").checked;
  // 越出车道的位置
  violMarker = null;
  if (D.result && D.result.ended_ms) {
    const [X, Z] = sceneXZ(D.result.viol_x, D.result.viol_y);
    violMarker = new T.Group(); dataGroup.add(violMarker);
    const ring = add(new T.Ring(150, 175, 64), new T.Basic({ color: "#d23c35", transparent: true, opacity: 0.8, depthWrite: false }), violMarker);
    ring.rotation.x = -Math.PI / 2; ring.position.set(X, 4, Z);
    cyl(4, 4, 420, X, 210, Z, new T.Basic({ color: "#d23c35" }), violMarker);
    sprite(`第 ${(D.result.ended_ms / 1000).toFixed(1)} 秒越出车道`, X, 470, Z, 330, "#ffffff", "#d23c35", violMarker);
  }
  // 障碍物（黑色 φ50×100，规则规定），被车碰到后变红
  obsMeshes = [];
  for (const [x, y, hit] of (D.field.obstacles || [])) {
    const [X, Z] = sceneXZ(x, y);
    const m = cyl(25, 25, 100, X, 50, Z, mat("#1d2226", 0.1, 0.7), dataGroup, 28);
    obsMeshes.push({ m, hit });
  }
  // 程序（单片机）发现障碍物时的估计位置：从日志里读，在发现的那一刻出现
  foundMarks = [];
  for (const [t, s] of D.logs) {
    const mm = s.match(/避障：发现障碍物 #(\d+) \((-?\d+), (-?\d+)\)/);
    if (!mm) continue;
    const [X, Z] = sceneXZ(+mm[2], +mm[3]);
    const r = add(new T.Ring(34, 42, 40), new T.Basic({ color: "#2a7fb8", transparent: true, opacity: 0.85, depthWrite: false }), dataGroup);
    r.rotation.x = -Math.PI / 2; r.position.set(X, 2.5, Z); r.castShadow = false;
    foundMarks.push({ t, m: r, x: +mm[2], y: +mm[3] });
  }
  buildTof();
  pieces.forEach((p, i) => p.body.material.color.set((COLORS[D.colors[i]] || ["", "#999"])[1]));
  drawQR(D.task);
}

/* ---------------- 机械臂：把仿真里的关节角换成 CAD 零件的位置 ---------------- */
const ARM = { ...MODEL.arm, shoulderOffset: 0, forearmOffset: 0 };   // 按 CAD 原样（不加原动画的演示隔套）
function graspPoint(q) {        // 与单片机 arm_fk() 完全相同的公式（车体坐标：x 向前，y 向左，z 向上）
  const R = D.robot, side = R.side || 0;
  const r = R.r0 + R.l1 * Math.cos(q[1]) + R.l2 * Math.cos(q[1] + q[2]);
  return [R.bx + r * Math.cos(q[0]) - side * Math.sin(q[0]), R.by + r * Math.sin(q[0]) + side * Math.cos(q[0]),
          R.shz + R.l1 * Math.sin(q[1]) + R.l2 * Math.sin(q[1] + q[2])];
}
const identity = new T.M4();
function poseArm(q, g) {
  const p = graspPoint(q);
  const P = KIN.solvePose(ARM, [-p[1], p[2], -p[0]]);        // 车体坐标 → CAD 坐标（车头 -Z，左 -X）
  for (const k of ["waist", "upper", "lower", "gripper"]) parts[k].matrix.fromArray(P[k]);
  const G = MODEL.gripper, jaw = KIN.solveJaws(G, G.closedAngle + Math.max(0, Math.min(1, g)) * (G.openAngle - G.closedAngle));
  for (const s of gripperMeshes) { if (jaw.transforms[s.body]) s.m.matrix.fromArray(jaw.transforms[s.body]); else s.m.matrix.copy(identity); }
  for (const s of pads) { const v = new T.V3(...s.point).applyMatrix4(new T.M4().fromArray(jaw.transforms[s.body])); s.m.matrix.makeTranslation(v.x, v.y, v.z); }
  return P;
}

/* ---------------- 数据与时间 ---------------- */
let D = null, FR = [], tNow = 0, playing = false, speed = 2, view = "field", lastTs = 0;
let chapters = [], marks = [], screens = [], logEls = [], lastLogIdx = -2, counts = null;

function frameAt(t) {          // 返回 [k, alpha]：t 落在第 k 帧和第 k+1 帧之间
  let lo = 0, hi = FR.length - 1;
  if (t <= FR[0][0]) return [0, 0];
  if (t >= FR[hi][0]) return [hi, 0];
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (FR[m][0] <= t) lo = m; else hi = m; }
  return [lo, (t - FR[lo][0]) / (FR[hi][0] - FR[lo][0])];
}
const lerp = (a, b, u) => a + (b - a) * u;
const lerpAng = (a, b, u) => { let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return a + d * u; };
function stateAt(t) {
  const [k, u] = frameAt(t), a = FR[k], b = FR[Math.min(k + 1, FR.length - 1)];
  const s = { k, t, x: lerp(a[1], b[1], u), y: lerp(a[2], b[2], u), th: lerpAng(a[3], b[3], u),
    q: [lerpAng(a[4], b[4], u), lerp(a[5], b[5], u), lerp(a[6], b[6], u)], g: lerp(a[7], b[7], u), mats: [],
    tt: a.length > 32 ? lerp(a[32], b[32], u) : 0, w: a.length > 36 ? [33, 34, 35, 36].map((i) => lerp(a[i], b[i], u)) : [0, 0, 0, 0],
    tof: a.length > 37 ? a.slice(37) : null };
  for (let i = 0; i < 6; i++) {
    const o = 8 + 4 * i, same = a[o + 3] === b[o + 3];
    s.mats.push({ x: same ? lerp(a[o], b[o], u) : a[o], y: same ? lerp(a[o + 1], b[o + 1], u) : a[o + 1], z: same ? lerp(a[o + 2], b[o + 2], u) : a[o + 2], loc: a[o + 3] });
  }
  const p = FR[Math.max(0, k - 4)], dtv = (a[0] - p[0]) / 1000;
  s.v = dtv > 0 ? Math.hypot(a[1] - p[1], a[2] - p[2]) / dtv : 0;
  return s;
}
function placeDesc(m) {         // 物料在哪里（给清单用）
  if (m.loc !== 2) return LOCS[m.loc] || "—";
  let best = null, bd = 1e9;
  for (const [z, n, x, y] of D.field.rings) { const d = Math.hypot(m.x - x, m.y - y); if (d < bd) { bd = d; best = [z, n]; } }
  if (bd < 60) return (best[0] === 0 ? "粗加工 " : "暂存 ") + best[1] + "号" + (m.z > 30 ? "·第2层" : "");
  return "场地上";
}

/* 由日志得出：阶段章节、问题标记、各时刻的计数 */
function analyseLogs() {
  const dur = FR[FR.length - 1][0];
  chapters = [{ t: 0, title: "上电待机" }]; marks = []; screens = [];
  const ev = []; let late = false;
  for (const [t, s] of D.logs) {
    let m;
    if ((m = s.match(/==== (.+?) 开始/))) { chapters.push({ t, title: m[1] + (late ? "（时间不够，放弃剩余任务）" : "") }); late = false; }
    else if (s.includes("mission: 出发")) chapters.push({ t, title: "出发，去读二维码" });
    else if (s.includes("时间不够了")) late = true;
    if (/\[模拟\] (空夹|物料掉落)|倒了/.test(s)) marks.push({ t, cls: "warn", s });
    if (s.includes("撞到障碍物") || s.includes("挡住了，停车")) marks.push({ t, cls: "bad", s });
    if (s.includes("避障：发现障碍物")) marks.push({ t, cls: "info", s });
    if (s.includes("[模拟] 夹住")) ev.push([t, "g"]);
    if (s.includes("[模拟] 放在地面") || s.includes("[模拟] 叠放")) ev.push([t, "p"]);
    if (s.includes("mission: 任务码")) ev.push([t, "q"]);
    if ((m = s.match(/\[屏幕\] (.*)$/))) screens.push([t, m[1].split("|").map((x) => x.trim())]);
  }
  if (D.result && D.result.ended_ms) marks.push({ t: D.result.ended_ms, cls: "bad", s: "越出车道" });
  chapters = chapters.filter((c) => c.t < dur).sort((a, b) => a.t - b.t);
  counts = ev;
}
function countsAt(t) {
  let g = 0, p = 0, q = false;
  for (const [tt, k] of counts) { if (tt > t) break; if (k === "g") g++; else if (k === "p") p++; else q = true; }
  return { g, p, q };
}
function chapterAt(t) { let i = 0; chapters.forEach((c, j) => { if (c.t <= t) i = j; }); return i; }

/* ---------------- 二维面板 ---------------- */
const mapCv = $("map"), armCv = $("arm");
function fitPanels() {
  const dpr = devicePixelRatio || 1;
  for (const [cv, ratio] of [[mapCv, 1], [armCv, 0.62]]) {
    const w = cv.clientWidth || 300; cv.width = Math.round(w * dpr); cv.height = Math.round(w * ratio * dpr); cv.style.height = w * ratio + "px";
  }
}
function drawMap(s) {
  const g = mapCv.getContext("2d"), W = mapCv.width, pad = W * 0.06, S = (W - 2 * pad) / FIELD;
  const X = (x) => pad + x * S, Y = (y) => pad + (FIELD - y) * S;
  g.fillStyle = css("--panel"); g.fillRect(0, 0, W, W);
  g.fillStyle = "#c9d3d6"; g.fillRect(X(0), Y(FIELD), FIELD * S, FIELD * S);
  g.fillStyle = "#f4edbf"; for (const a of [550, 1400]) for (const b of [550, 1400]) g.fillRect(X(a), Y(b + 450), 450 * S, 450 * S);
  g.fillStyle = "#ffffff"; g.fillRect(X(910), Y(150), 580 * S, 150 * S); g.fillRect(X(0), Y(1490), 150 * S, 580 * S);
  g.fillStyle = "#5375d1"; g.fillRect(X(2100), Y(2400), 300 * S, 300 * S); g.fillRect(X(2100), Y(300), 300 * S, 300 * S);
  g.strokeStyle = "#33454d"; g.lineWidth = Math.max(1, W / 400);
  for (const [, , x, y] of D.field.rings) { g.beginPath(); g.arc(X(x), Y(y), 26 * S, 0, 7); g.stroke(); }
  g.fillStyle = "#e1e5e5"; g.beginPath(); g.arc(X(D.field.tt[0]), Y(D.field.tt[1]), 150 * S, 0, 7); g.fill(); g.stroke();
  // 轨迹
  g.lineWidth = Math.max(1.5, W / 220); g.lineCap = "round";
  for (let i = 4; i <= s.k; i += 4) { const a = FR[i - 4], b = FR[i]; g.strokeStyle = `hsl(${185 - 155 * (i / FR.length)},70%,42%)`; g.beginPath(); g.moveTo(X(a[1]), Y(a[2])); g.lineTo(X(b[1]), Y(b[2])); g.stroke(); }
  if (D.result && D.result.ended_ms && s.t >= D.result.ended_ms) { g.strokeStyle = "#d23c35"; g.lineWidth = Math.max(2, W / 150); g.beginPath(); g.arc(X(D.result.viol_x), Y(D.result.viol_y), 90 * S, 0, 7); g.stroke(); }
  // 障碍物和程序发现的位置
  for (const [x, y, hit] of (D.field.obstacles || [])) { g.fillStyle = hit && s.t >= hit ? "#d23c35" : "#1d2226"; g.beginPath(); g.arc(X(x), Y(y), Math.max(2.5, 25 * S), 0, 7); g.fill(); }
  g.strokeStyle = "#2a7fb8"; g.lineWidth = Math.max(1, W / 300); g.setLineDash([3, 3]);
  for (const f of foundMarks) if (f.t <= s.t) { g.beginPath(); g.arc(X(f.x), Y(f.y), Math.max(5, 45 * S), 0, 7); g.stroke(); }
  g.setLineDash([]);
  // 物料
  s.mats.forEach((m, i) => { if (!m.loc || m.loc === 3 || m.loc === 4) return; g.fillStyle = COLORS[D.colors[i]][1]; g.beginPath(); g.arc(X(m.x), Y(m.y), 28 * S, 0, 7); g.fill(); });
  // 小车
  g.save(); g.translate(X(s.x), Y(s.y)); g.rotate(-s.th);
  g.fillStyle = "rgba(19,127,136,.85)"; g.fillRect(-117 * S, -148 * S, 234 * S, 296 * S);
  g.fillStyle = "#fff"; g.beginPath(); g.moveTo(117 * S, 0); g.lineTo(60 * S, -40 * S); g.lineTo(60 * S, 40 * S); g.fill();
  g.restore();
}
function drawArm(s) {
  const g = armCv.getContext("2d"), w = armCv.width, h = armCv.height, dpr = devicePixelRatio || 1, R = D.robot, [q0, q1, q2] = s.q;
  const top = 44 * dpr, sc = Math.min(w / 480, (h - top - 8 * dpr) / 330), ox = 185 * sc, oy = h - 12 * sc;
  const X = (r) => ox + r * sc, Y = (z) => oy - z * sc;
  g.fillStyle = css("--panel"); g.fillRect(0, 0, w, h);
  g.font = `${12 * dpr}px Microsoft YaHei, sans-serif`; g.textBaseline = "middle";
  const hline = (z, txt) => { g.strokeStyle = css("--line"); g.lineWidth = dpr; g.setLineDash([4 * dpr, 4 * dpr]); g.beginPath(); g.moveTo(0, Y(z)); g.lineTo(w, Y(z)); g.stroke(); g.setLineDash([]); g.fillStyle = css("--quiet"); g.textAlign = "right"; g.fillText(txt, w - 4 * dpr, Y(z) - 8 * dpr); };
  hline(0, "地面"); hline(R.tray, `槽底 ${R.tray}`); hline(R.tt, ""); hline(R.shz, `肩轴 ${R.shz}`);
  g.fillStyle = css("--soft"); g.strokeStyle = css("--quiet"); g.lineWidth = dpr;
  g.fillRect(X(-165), Y(R.tray), 230 * sc, R.tray * sc); g.strokeRect(X(-165), Y(R.tray), 230 * sc, R.tray * sc);
  const re = R.r0 + R.l1 * Math.cos(q1), ze = R.shz + R.l1 * Math.sin(q1), rt = re + R.l2 * Math.cos(q1 + q2), zt = ze + R.l2 * Math.sin(q1 + q2);
  g.strokeStyle = "#dc8d35"; g.lineWidth = 8 * sc; g.lineCap = "round";
  g.beginPath(); g.moveTo(X(0), Y(R.tray)); g.lineTo(X(0), Y(R.shz)); g.lineTo(X(R.r0), Y(R.shz)); g.lineTo(X(re), Y(ze)); g.lineTo(X(rt), Y(zt)); g.stroke();
  g.fillStyle = css("--ink"); for (const [r, z] of [[R.r0, R.shz], [re, ze]]) { g.beginPath(); g.arc(X(r), Y(z), 5 * sc, 0, 7); g.fill(); }
  g.fillStyle = "#0b7f87"; g.beginPath(); g.arc(X(rt), Y(zt), 7 * sc, 0, 7); g.fill();
  g.textAlign = "left"; g.fillStyle = css("--ink");
  g.fillText(`夹持中心离地 ${zt.toFixed(0)} mm · 伸出 ${rt.toFixed(0)} mm`, 4 * dpr, 12 * dpr);
  g.fillStyle = css("--quiet");
  g.fillText(`回转 ${(q0 * 57.3).toFixed(0)}°  肩 ${(q1 * 57.3).toFixed(0)}°  肘 ${(q2 * 57.3).toFixed(0)}°  夹爪${s.g > 0.6 ? "张开" : "合上"}`, 4 * dpr, 30 * dpr);
}
let screenShown = -2;
function drawScreen(t) {          // 车头屏幕：显示单片机程序在这一刻真正写到屏幕上的 4 行字（来自日志里的 [屏幕] 行）
  let k = -1;
  for (let i = 0; i < screens.length; i++) { if (screens[i][0] <= t) k = i; else break; }
  if (k === screenShown) return;
  screenShown = k;
  const g = screenCanvas.getContext("2d"), L = k >= 0 ? screens[k][1] : ["", "", "", ""];
  g.fillStyle = "#162d3a"; g.fillRect(0, 0, 768, 540);
  g.fillStyle = "#b5f2c8"; g.font = "bold 128px Consolas, monospace";
  g.fillText(L[0] || "", 24, 140); g.fillText(L[1] || "", 24, 280);
  g.font = "bold 76px Consolas, monospace"; g.fillStyle = "#d5e9e9";
  g.fillText(L[2] || "", 24, 395); g.fillText(L[3] || "", 24, 495);
  screenTex.needsUpdate = true;
}

/* ---------------- 把一个时刻画出来 ---------------- */
function apply(t) {
  const s = stateAt(t), c = countsAt(t);
  const [X, Z] = sceneXZ(s.x, s.y);
  robot.position.set(X, 0, Z); robot.rotation.y = s.th - Math.PI / 2;
  halo.position.set(X, 1.2, Z);
  poseArm(s.q, s.g);
  wheels.forEach((w, i) => (w.rotation.x = -s.w[i]));
  const [tx, tz] = sceneXZ(D.field.tt[0], D.field.tt[1]); turntable.position.set(tx, 0, tz); ttDisc.rotation.y = s.tt;
  s.mats.forEach((m, i) => {
    const p = pieces[i]; p.g.visible = m.loc !== 0; p.g.rotation.set(0, 0, 0);
    const [mx, mz] = sceneXZ(m.x, m.y); p.g.position.set(mx, m.z, mz);
    if (m.loc === 5) { p.g.rotation.z = Math.PI / 2; p.g.position.y = 25; }
  });
  if (trail) trail.geometry.setDrawRange(0, s.k + 1);
  for (const o of obsMeshes) o.m.material.color.set(o.hit && t >= o.hit ? "#d23c35" : "#1d2226");
  for (const f of foundMarks) f.m.visible = t >= f.t;
  if (D.tof && s.tof) tofBeams.forEach((b, i) => {
    const d = s.tof[i] || 0, L = d > 0 ? d : D.tof.max;
    b.material = d > 0 ? beamOn : beamOff; b.scale.set(L, 1, L);
  });
  if (violMarker) violMarker.visible = !!(D.result && t >= D.result.ended_ms);
  drawScreen(t);
  // 文字面板
  const ci = chapterAt(t);
  $("hudTime").textContent = (t / 1000).toFixed(1) + " s";
  $("hudPhase").textContent = chapters[ci].title;
  $("hudOnboard").textContent = s.mats.filter((m) => m.loc === 3).length;
  $("stageTitle").textContent = chapters[ci].title;
  const scr = screenShown >= 0 ? screens[screenShown][1].join(" ") : "", gm = scr.match(/G(\d+)/), pm = scr.match(/P(\d+)/);
  $("sGrab").textContent = gm ? gm[1] : c.g; $("sPlace").textContent = pm ? pm[1] : c.p; $("sSpeed").textContent = Math.round(s.v);
  $("clock").textContent = `${(t / 1000).toFixed(1)} / ${(FR[FR.length - 1][0] / 1000).toFixed(1)} s`;
  $("seek").value = t;
  [...$("chapters").children].forEach((el, i) => el.classList.toggle("active", i === ci));
  $("inventory").innerHTML = s.mats.map((m, i) => `<div class="item${m.loc === 5 ? " bad" : ""}"><i style="background:${COLORS[D.colors[i]][1]}"></i><b>${COLORS[D.colors[i]][0]}</b><span>${i < 3 ? "一" : "二"}批</span><em>${placeDesc(m)}</em></div>`).join("");
  const ended = D.result && D.result.ended_ms && t >= D.result.ended_ms;
  $("endedFlag").hidden = !ended;
  if (ended) $("endedFlag").textContent = `第 ${(D.result.ended_ms / 1000).toFixed(1)} 秒已越出车道：按规则本轮在那一刻结束，之后的画面只用来看问题`;
  drawMap(s); drawArm(s); updateLog(t);
  return s;
}
function updateLog(t) {
  let k = -1;
  for (let i = 0; i < D.logs.length; i++) { if (D.logs[i][0] <= t) k = i; else break; }
  if (k === lastLogIdx) return;
  logEls.forEach((el, i) => { el.classList.toggle("future", i > k); el.classList.toggle("cur", i === k); });
  const box = $("log"), el = logEls[k];
  if (el && el.style.display !== "none") box.scrollTop = el.offsetTop - box.offsetTop - box.clientHeight / 2;
  lastLogIdx = k;
}

/* ---------------- 相机与渲染 ---------------- */
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix(); setView(view, true); fitPanels();
}
function setView(v, keep) {
  view = v; snapCam = true;
  document.querySelectorAll("[data-view]").forEach((b) => b.classList.toggle("selected", b.dataset.view === v));
  const k = Math.max(1, 1.25 / cam.aspect);
  if (v === "field") { controls.target.set(1200, 0, 1260); cam.position.copy(controls.target).add(new T.V3(2050, 3150, 2500).multiplyScalar(k)); }
  if (v === "top") { controls.target.set(1200, 0, 1200); cam.position.set(1200, 4300 * k, 1200.1); }
  if (!keep && D) render();
}
let stateCache = null, snapCam = true;
function render() {
  if (!D) { renderer.render(scene, cam); return; }
  stateCache = apply(tNow);
  const robotPos = new T.V3(robot.position.x, 140, robot.position.z), k = Math.max(1, 0.95 / cam.aspect);
  const a = snapCam ? 1 : 0.18; snapCam = false;
  if (view === "follow") { controls.target.lerp(robotPos, a); cam.position.lerp(robotPos.clone().add(new T.V3(570, 580, 720).multiplyScalar(k)), a); }
  if (view === "detail") { controls.target.lerp(robotPos, a); cam.position.lerp(robotPos.clone().add(new T.V3(360, 300, 440).multiplyScalar(k)), a); }
  controls.update();
  const w = viewport.clientWidth, h = viewport.clientHeight;
  floatLabels.forEach((l) => (l.visible = view !== "detail"));
  if (violMarker) violMarker.visible = !!(D.result && tNow >= D.result.ended_ms);
  renderer.setScissorTest(false); renderer.setViewport(0, 0, w, h); renderer.render(scene, cam);
  const inset = $("insetLabel");
  if ($("tInset").checked && w > 600 && view !== "detail") {
    const iw = Math.min(330, w * 0.32), ih = iw / 1.36, o = 18;
    insetCam.position.copy(robotPos).add(new T.V3(-330, 290, 400)); insetCam.lookAt(robotPos);
    renderer.setScissorTest(true); renderer.setScissor(o, o, iw, ih); renderer.setViewport(o, o, iw, ih);
    const bg = scene.background; scene.background = new T.Color("#dce7e8");
    floatLabels.forEach((l) => (l.visible = false));
    renderer.clearDepth(); renderer.render(scene, insetCam);
    scene.background = bg; renderer.setScissorTest(false);
    Object.assign(inset.style, { display: "block", width: iw + "px", height: ih + "px" });
  } else inset.style.display = "none";
}
function tick(ts) {
  const dt = lastTs ? Math.min(0.1, (ts - lastTs) / 1000) : 0; lastTs = ts;
  if (playing && D) {
    tNow = Math.min(FR[FR.length - 1][0], tNow + dt * 1000 * speed);
    if (tNow >= FR[FR.length - 1][0]) { playing = false; $("play").textContent = "重新播放"; }
  }
  render();
  requestAnimationFrame(tick);
}

/* ---------------- 载入一份数据 ---------------- */
function fmt(ms) { return (ms / 1000).toFixed(1) + " s"; }
function init(data) {
  D = data; FR = D.frames; tNow = 0; playing = false; lastLogIdx = -2; screenShown = -2;
  if (!FR || FR.length < 2) throw new Error("数据里没有帧");
  $("empty").hidden = true;
  const r = D.result;
  $("subtitle").textContent = `任务码 ${D.task} · 启停区 ${D.zone} · 轮子打滑 ${(D.slip * 100).toFixed(0)}% · ${D.imu ? "装陀螺仪" : "不装陀螺仪"}`;
  $("runStamp").innerHTML = D.run_at ? `本次仿真运行于 <b>${D.run_at}</b>` : "旧格式数据（没有运行时间）";
  const bn = $("banner"); bn.hidden = false;
  if (r && r.ended_ms) { bn.className = "banner bad"; bn.innerHTML = `<b>第 ${fmt(r.ended_ms)} 车身越出车道</b>（位置 ${r.viol_x.toFixed(0)}, ${r.viol_y.toFixed(0)}）：按规则本轮在这一刻结束，之后的动作不计分。${escapeHtml(D.summary.replace(/^【.*?】\s*/, ""))}`; }
  else { bn.className = "banner"; bn.textContent = D.summary; }
  // 成绩卡片
  if (r) {
    $("scoreBig").textContent = r.score;
    const note = $("scoreNote");
    note.className = r.ended_ms ? "bad" : ""; note.textContent = r.ended_ms ? "本轮提前结束" : r.in_zone ? "正常跑完并回到启停区" : "未停回启停区";
    const fillEl = $("timeFill"), used = r.ended_ms || r.time_ms;
    fillEl.style.width = Math.min(100, (used / r.limit_ms) * 100) + "%"; fillEl.className = used > r.limit_ms ? "over" : "";
    $("timeText").textContent = r.ended_ms ? `第 ${fmt(r.ended_ms)} 结束（越线）` : `用时 ${fmt(r.time_ms)} / 限时 ${fmt(r.limit_ms)}`;
    const ret = r.in_zone ? 2 : 0, stat = r.score - (r.qr + 2 * r.correct_grabs + r.rough + r.stash + r.stack + ret);
    const rows = [["读取二维码并显示", r.qr], [`正确抓取 ${r.correct_grabs} 件 × 2`, 2 * r.correct_grabs], ["粗加工区放置", r.rough], ["暂存区放置", r.stash], ["同色码垛", r.stack], ["返回启停区", ret], ["抓取 / 放置数量统计", stat]];
    $("breakdown").innerHTML = rows.map(([a, b]) => `<tr class="${b ? "" : "zero"}"><td>${a}</td><td>${b}</td></tr>`).join("") + `<tr><td>最大放置偏差</td><td>${r.max_err.toFixed(1)} mm</td></tr>`
      + (r.obs ? `<tr class="${r.obs_hit ? "bad" : ""}"><td>障碍物 ${r.obs} 个（摆法 ${r.obs_seed}）· 程序发现 ${r.obs_found}、误报 ${r.obs_false}</td><td>撞到 ${r.obs_hit}</td></tr>` : "");
  }
  // 任务码
  const p = D.task.split("+");
  $("codeValue").textContent = D.task;
  $("codeKey").innerHTML = `<span><b>${p[0]}</b> 第一批颜色顺序</span><span><b>${p[1]}</b> 第一批放几号环</span><span><b>${p[2]}</b> 第二批颜色顺序</span><span><b>${p[3]}</b> 第二批粗加工位置</span>`;
  // 参数
  const C = D.config || {};
  const cfg = [["最高车速", C.nav_vmax, "mm/s", "NAV_VMAX_MMPS"], ["加速度", C.nav_acc, "mm/s²", "NAV_ACC_MMPS2"], ["机械臂速度", C.arm_speed, "步/s", "ARM_SPEED_RAW_PER_S"], ["机械臂安全高度", C.arm_safe_z, "mm", "ARM_SAFE_Z_MM"],
    ["夹爪侧偏", C.tool_side, "mm", "ARM_TOOL_SIDE_MM"], ["肩轴偏移", C.shoulder_r0, "mm", "ARM_SHOULDER_R0_MM"], ["返程时间余量", C.return_reserve_ms, "ms", "MATCH_RETURN_RESERVE_MS"], ["外圈车道离场地边", C.lane_edge, "mm", "field.c LANE_EDGE"]];
  $("config").innerHTML = cfg.filter((c) => c[1] !== undefined).map(([a, v, u, n]) => `<tr><td>${a}<small>${n}</small></td><td>${v} ${u}</td></tr>`).join("") || "<tr><td>旧格式数据，没有参数记录</td><td></td></tr>";
  analyseLogs();
  // 章节
  $("chapters").innerHTML = "";
  chapters.forEach((c, i) => {
    const b = document.createElement("button"); b.className = "chapter";
    b.innerHTML = `<span>${String(i + 1).padStart(2, "0")}</span><b>${escapeHtml(c.title)}</b><small>${fmt(c.t)}</small>`;
    b.onclick = () => seek(c.t); $("chapters").appendChild(b);
  });
  // 时间轴
  const dur = FR[FR.length - 1][0], seekEl = $("seek");
  seekEl.max = dur; seekEl.step = 50;
  $("tlSegs").innerHTML = chapters.map((c, i) => { const e = i + 1 < chapters.length ? chapters[i + 1].t : dur; return `<div style="width:${((e - c.t) / dur) * 100}%" title="${escapeHtml(c.title)}（${fmt(c.t)}）"></div>`; }).join("");
  $("tlMarks").innerHTML = marks.map((m) => `<i class="${m.cls}" style="left:${(m.t / dur) * 100}%" title="${escapeHtml(m.s)}"></i>`).join("");
  const te = $("tlEnded");
  if (r && r.ended_ms) { te.style.display = "block"; te.style.left = (r.ended_ms / dur) * 100 + "%"; te.style.right = "0"; } else te.style.display = "none";
  const lim = $("tlLimit"); lim.style.display = r && r.limit_ms <= dur ? "block" : "none"; if (r) lim.style.left = (r.limit_ms / dur) * 100 + "%";
  // 日志
  const lb = $("log"); lb.innerHTML = "";
  logEls = D.logs.map(([t, s]) => {
    const d = document.createElement("div"); d.textContent = s; d.dataset.t = t;
    const sim = s.includes("[模拟]"), bad = /越出车道|压线|超时|撞到障碍物|挡住了，停车/.test(s), warn = /空夹|掉落|倒了|放弃|失败|无应答/.test(s);
    d.dataset.kind = bad ? "bad" : warn ? "bad" : sim ? "sim" : "prog";
    if (sim) d.classList.add("sim"); if (warn) d.classList.add("warn"); if (bad) d.classList.add("bad"); if (s.includes("====")) d.classList.add("phase");
    d.onclick = () => seek(t); lb.appendChild(d); return d;
  });
  applyFilter(document.querySelector(".filters .selected").dataset.f);
  buildDataObjects();
  $("play").textContent = "播放";
  fitPanels(); setView(view);
}
function escapeHtml(s) { return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]); }
function seek(t) { snapCam = true; tNow = Math.max(0, Math.min(FR[FR.length - 1][0], t)); playing = false; $("play").textContent = "继续播放"; render(); }
function applyFilter(f) {
  document.querySelectorAll(".filters button").forEach((b) => b.classList.toggle("selected", b.dataset.f === f));
  logEls.forEach((el) => { const k = el.dataset.kind; el.style.display = f === "all" || f === k || (f === "sim" && k === "sim") ? "" : "none"; });
  lastLogIdx = -2;
}
function parseReplay(txt) {
  txt = txt.trim();
  const m = txt.match(/^window\.SIM_REPLAY\s*=\s*/);
  if (m) txt = txt.slice(m[0].length);
  else { const i = txt.indexOf("const DATA="); if (i >= 0) { txt = txt.slice(i + 11); txt = txt.slice(0, txt.indexOf(";\n")); } }
  txt = txt.replace(/;\s*$/, "");
  try { return JSON.parse(txt); } catch (e) { throw new Error("读不懂这个文件：请选 sim 文件夹里新生成的 replay_*.js（旧版本生成的文件格式不同，重新跑一次仿真即可）"); }
}
function loadFile(file) {
  const rd = new FileReader();
  rd.onload = () => { try { init(parseReplay(rd.result)); } catch (e) { alertBox(e.message); } };
  rd.readAsText(file, "utf-8");
}
function alertBox(msg) { const bn = $("banner"); bn.hidden = false; bn.className = "banner bad"; bn.innerHTML = `<b>无法载入：</b>${escapeHtml(msg)}`; }

/* ---------------- 交互 ---------------- */
$("play").onclick = () => { if (!D) return; if (tNow >= FR[FR.length - 1][0]) tNow = 0; playing = !playing; $("play").textContent = playing ? "暂停" : "继续播放"; };
$("restart").onclick = () => { if (!D) return; tNow = 0; playing = false; $("play").textContent = "播放"; };
$("speed").onchange = (e) => (speed = Number(e.target.value));
$("seek").oninput = (e) => { if (D) seek(Number(e.target.value)); };
document.querySelectorAll("[data-view]").forEach((b) => (b.onclick = () => setView(b.dataset.view)));
$("tTrail").onchange = (e) => { if (trail) trail.visible = e.target.checked; };
$("tLanes").onchange = (e) => { if (laneGroup) laneGroup.visible = e.target.checked; };
$("tTof").onchange = (e) => { if (tofGroup) tofGroup.visible = e.target.checked; };
$("fullscreen").onclick = () => (document.fullscreenElement ? document.exitFullscreen() : $("stage").requestFullscreen());
document.querySelectorAll(".filters button").forEach((b) => (b.onclick = () => applyFilter(b.dataset.f)));
$("fileInput").onchange = (e) => { if (e.target.files[0]) loadFile(e.target.files[0]); };
for (const ev of ["dragover", "dragenter"]) viewport.addEventListener(ev, (e) => { e.preventDefault(); $("empty").classList.add("drag"); });
viewport.addEventListener("dragleave", () => $("empty").classList.remove("drag"));
viewport.addEventListener("drop", (e) => { e.preventDefault(); $("empty").classList.remove("drag"); if (e.dataTransfer.files[0]) loadFile(e.dataTransfer.files[0]); });
canvas.addEventListener("pointerdown", () => { if (view === "follow" || view === "detail") { view = "custom"; document.querySelectorAll("[data-view]").forEach((b) => b.classList.remove("selected")); } });
window.addEventListener("keydown", (e) => {
  if (!D || e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  if (e.code === "Space") { e.preventDefault(); $("play").click(); }
  if (e.key === "ArrowRight") seek(tNow + 5000);
  if (e.key === "ArrowLeft") seek(tNow - 5000);
  if (["1", "2", "3", "4"].includes(e.key)) setView(["field", "follow", "detail", "top"][Number(e.key) - 1]);
});
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { scene.background = new T.Color(css("--scene-bg")); });
new ResizeObserver(resize).observe(viewport);
resize();
$("loading").classList.add("done");
if (window.SIM_REPLAY) { try { init(window.SIM_REPLAY); } catch (e) { alertBox(e.message); } }
else $("empty").hidden = false;
requestAnimationFrame(tick);
window.SIM3D = { seek, setView, get t() { return tNow; }, get data() { return D; }, poseArm, graspPoint, render, parts, robot };
})();
