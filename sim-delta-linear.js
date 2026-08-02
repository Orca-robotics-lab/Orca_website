/* ========================================================================
   LINEAR DELTA SIMULATOR  ·  Orca Robotics
   3-PUU linear (Rostock / Kossel style) parallel manipulator
   ------------------------------------------------------------------------
   Three vertical towers spaced 120° apart. Each tower carries a prismatic
   CARRIAGE that slides up/down (the driven joint). A fixed-length diagonal
   rod (parallelogram pair) ties each carriage to the moving platform, so
   the three rods act as the legs of three spheres.

   Geometry (origin at tower-circle centre, Z up, ground at Z = 0):
     • R   tower-circle radius   (carriage joint sits at radius R)
     • r   effector offset       (platform joint sits at radius r)
     • L   diagonal rod length   (carriage joint → platform joint)
     • H   tower height          (carriage travel 0 … H)

   Because the platform joint sits r inboard in the same tower direction,
   each leg behaves like a sphere centred on a "virtual" carriage joint at
   horizontal radius (R − r):
       Aᵢ = ((R−r)cosφᵢ , (R−r)sinφᵢ , zᵢ)
   and the effector centre E = (x,y,z) satisfies
       (x−Aᵢx)² + (y−Aᵢy)² + (z−zᵢ)² = L²        for i = 1,2,3

   INVERSE  E → zᵢ :  zᵢ = z + √( L² − (x−Aᵢx)² − (y−Aᵢy)² )   (carriage above)
   FORWARD  zᵢ → E :  trilaterate three spheres (linear solve + quadratic),
                      take the lower-Z root (platform hangs below carriages).
   ====================================================================== */

const deg2rad = d => d * Math.PI / 180;
const fmt = (n, p = 3) => (Math.abs(n) < 1e-10 ? 0 : n).toFixed(p);

const PHI = [90, 210, 330].map(deg2rad);              // tower azimuths
const ARM_COL = [0x2A6FDB, 0x1F8A5B, 0xC7468A];       // blue / green / pink
const MM = 100;                                        // mm per internal world unit

const state = {
  R: 1.4, r: 0.3, L: 3.0, H: 4.4,
  carriage: [3.6, 3.6, 3.6],                          // carriage heights (world units)
  targetX: 0.4, targetY: 0.3, targetZ: 0.9,
  ikSolution: null
};

function readInputs() {
  const g = id => parseFloat(document.getElementById(id).value) || 0;
  state.R = g('R') / MM; state.r = g('r') / MM; state.L = g('L') / MM; state.H = g('H') / MM;
  state.targetX = g('target-x') / MM; state.targetY = g('target-y') / MM; state.targetZ = g('target-z') / MM;
}

/* ── robot frame (X,Y,Z, Z up) → three.js world (Y up), ground at Y=0 ── */
function robotToWorld(X, Y, Z) { return [X, Z, -Y]; }

/* virtual carriage-joint horizontal position for tower i */
function towerXY(i) {
  const d = state.R - state.r;
  return { x: d * Math.cos(PHI[i]), y: d * Math.sin(PHI[i]) };
}
/* actual carriage-joint horizontal position (radius R) */
function carriageXY(i) {
  return { x: state.R * Math.cos(PHI[i]), y: state.R * Math.sin(PHI[i]) };
}

/* ========================================================================
   INVERSE KINEMATICS  —  (x,y,z) → (z₁,z₂,z₃)
   Each carriage sits L above the platform joint along its tower line:
       zᵢ = z + √( L² − hdᵢ² ),   hdᵢ² = (x−Aᵢx)² + (y−Aᵢy)²
   ====================================================================== */
function inverseKinematics(x, y, z) {
  const carriages = [], reasons = [];
  let reachable = true;
  for (let i = 0; i < 3; i++) {
    const A = towerXY(i);
    const hd2 = (x - A.x) * (x - A.x) + (y - A.y) * (y - A.y);
    const k = state.L * state.L - hd2;
    if (k < 0) { reachable = false; carriages.push(NaN); reasons.push('rod too short'); }
    else { carriages.push(z + Math.sqrt(k)); reasons.push(null); }
  }
  return { reachable, carriages, reasons, x, y, z };
}

// physically reachable: every carriage exists AND sits within travel [0, H]
function physReach(x, y, z) {
  const s = inverseKinematics(x, y, z);
  if (!s.reachable) return false;
  return s.carriages.every(c => c >= -1e-6 && c <= state.H + 1e-6);
}

/* ========================================================================
   FORWARD KINEMATICS  —  (z₁,z₂,z₃) → (x,y,z)
   Trilateration of three spheres of radius L centred on the virtual
   carriage joints Aᵢ = (Aᵢx, Aᵢy, zᵢ).
   ====================================================================== */
function forwardKinematics(carriages) {
  const c = carriages || state.carriage;
  const A = [0, 1, 2].map(i => { const t = towerXY(i); return { x: t.x, y: t.y, z: c[i] }; });

  const w = A.map(p => p.x * p.x + p.y * p.y + p.z * p.z);
  // (1)-(2) and (1)-(3) → two linear equations in x,y,z
  const a1 = A[0].x - A[1].x, b1 = A[0].y - A[1].y, e1 = A[0].z - A[1].z, d1 = (w[0] - w[1]) / 2;
  const a2 = A[0].x - A[2].x, b2 = A[0].y - A[2].y, e2 = A[0].z - A[2].z, d2 = (w[0] - w[2]) / 2;

  const det = a1 * b2 - a2 * b1;
  if (Math.abs(det) < 1e-12) return { reachable: false, reason: 'degenerate geometry', A };

  // x = px + qx·z ,  y = py + qy·z
  const px = (d1 * b2 - d2 * b1) / det, qx = (e2 * b1 - e1 * b2) / det;
  const py = (a1 * d2 - a2 * d1) / det, qy = (a2 * e1 - a1 * e2) / det;

  // substitute into sphere 1 → quadratic in z
  const Px = px - A[0].x, Py = py - A[0].y;
  const Aq = qx * qx + qy * qy + 1;
  const Bq = 2 * (Px * qx + Py * qy - A[0].z);
  const Cq = Px * Px + Py * Py + A[0].z * A[0].z - state.L * state.L;

  const disc = Bq * Bq - 4 * Aq * Cq;
  if (disc < 0) return { reachable: false, reason: 'carriages too far apart — rods cannot close', A };

  const z0 = (-Bq - Math.sqrt(disc)) / (2 * Aq);   // lower root → platform below carriages
  const x0 = px + qx * z0;
  const y0 = py + qy * z0;
  return { reachable: true, P: { x: x0, y: y0, z: z0 }, A, coef: { px, qx, py, qy, Aq, Bq, Cq, disc } };
}

/* ========================================================================
   THREE.JS SCENE
   ====================================================================== */
let scene, camera, renderer;
let robotGroup, wvGroup = null, wvVisible = false, wvDirty = true;
let targetMarker;

function initThree() {
  const canvas = document.getElementById('canvas');
  const w = canvas.clientWidth, h = canvas.clientHeight;

  scene = new THREE.Scene();
  scene.background = null;
  scene.fog = new THREE.Fog(0xECECEA, 20, 46);

  camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 100);
  camera.position.set(6.5, 4.6, 8);

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  scene.add(new THREE.AmbientLight(0xffffff, 0.85));
  const key = new THREE.DirectionalLight(0xffffff, 1.05);
  key.position.set(6, 12, 5);
  key.castShadow = true;
  key.shadow.camera.left = -10; key.shadow.camera.right = 10;
  key.shadow.camera.top = 10; key.shadow.camera.bottom = -10;
  key.shadow.mapSize.width = 1024; key.shadow.mapSize.height = 1024;
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xE87722, 0.28);
  fill.position.set(-4, 4, -3);
  scene.add(fill);

  const grid = new THREE.GridHelper(16, 32, 0xB8B8B6, 0xD8D8D6);
  grid.material.transparent = true; grid.material.opacity = 0.7;
  scene.add(grid);

  const ground = new THREE.Mesh(new THREE.PlaneGeometry(16, 16), new THREE.ShadowMaterial({ opacity: 0.32 }));
  ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true;
  scene.add(ground);

  // world axes at origin
  const axL = 2.0;
  scene.add(makeLine([0, 0.01, 0], [axL, 0.01, 0], 0xCC3D4F, 3));     // X
  scene.add(makeLine([0, 0.01, 0], [0, 0.01, -axL], 0x2E9E5B, 3));    // Y (robot +Y → world −Z)
  scene.add(makeLine([0, 0.01, 0], [0, axL, 0], 0x2A6FDB, 3));        // Z up

  robotGroup = new THREE.Group();
  scene.add(robotGroup);

  targetMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.12, 20, 16),
    new THREE.MeshStandardMaterial({ color: 0xC7468A, emissive: 0x3a0f24, emissiveIntensity: 0.6, metalness: 0.3, roughness: 0.4 })
  );
  targetMarker.visible = false;
  scene.add(targetMarker);
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.2, 0.26, 28),
    new THREE.MeshBasicMaterial({ color: 0xC7468A, side: THREE.DoubleSide, transparent: true, opacity: 0.9 })
  );
  ring.rotation.x = -Math.PI / 2; ring.position.y = -0.16;
  targetMarker.add(ring);

  setupCameraControls(canvas);
  window.addEventListener('resize', resizeRenderer);
  const vp = document.querySelector('.viewport');
  if (window.ResizeObserver && vp) { new ResizeObserver(() => resizeRenderer()).observe(vp); }
  animate();
}

function makeLine(a, b, color, width = 2) {
  const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a), new THREE.Vector3(...b)]);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, linewidth: width }));
}

function resizeRenderer() {
  const vp = document.querySelector('.viewport');
  const w = vp ? vp.clientWidth : 0, h = vp ? vp.clientHeight : 0;
  if (w > 0 && h > 0) { camera.aspect = w / h; camera.updateProjectionMatrix(); renderer.setSize(w, h); }
}

function setupCameraControls(canvas) {
  const target = new THREE.Vector3(0, 2.0, 0);
  const offset = new THREE.Vector3().subVectors(camera.position, target);
  let radius = offset.length();
  let phi = Math.atan2(offset.z, offset.x);
  let theta = Math.acos(offset.y / radius);
  let dragging = false, mode = 'orbit', sx = 0, sy = 0;
  const upd = () => {
    camera.position.x = target.x + radius * Math.sin(theta) * Math.cos(phi);
    camera.position.z = target.z + radius * Math.sin(theta) * Math.sin(phi);
    camera.position.y = target.y + radius * Math.cos(theta);
    camera.lookAt(target);
  };
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  canvas.addEventListener('mousedown', e => {
    dragging = true; sx = e.clientX; sy = e.clientY;
    mode = (e.button === 2 || e.button === 1 || (e.button === 0 && e.shiftKey)) ? 'pan' : 'orbit';
  });
  window.addEventListener('mouseup', () => dragging = false);
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    const dx = e.clientX - sx, dy = e.clientY - sy; sx = e.clientX; sy = e.clientY;
    if (mode === 'orbit') {
      phi -= dx * 0.006;
      theta = Math.max(0.08, Math.min(Math.PI - 0.08, theta - dy * 0.006));
    } else {
      const fwd = new THREE.Vector3().subVectors(target, camera.position).normalize();
      const right = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
      const up = new THREE.Vector3().crossVectors(right, fwd).normalize();
      const ps = radius * 0.0018;
      target.addScaledVector(right, -dx * ps); target.addScaledVector(up, dy * ps);
    }
    upd();
  });
  let lt = null;
  canvas.addEventListener('touchstart', e => { lt = [...e.touches].map(t => ({ x: t.clientX, y: t.clientY })); }, { passive: true });
  canvas.addEventListener('touchmove', e => {
    if (!lt) return;
    const t = [...e.touches].map(t => ({ x: t.clientX, y: t.clientY }));
    if (t.length === 1 && lt.length === 1) {
      phi -= (t[0].x - lt[0].x) * 0.006;
      theta = Math.max(0.08, Math.min(Math.PI - 0.08, theta - (t[0].y - lt[0].y) * 0.006));
      upd();
    } else if (t.length === 2 && lt.length === 2) {
      const d0 = Math.hypot(lt[0].x - lt[1].x, lt[0].y - lt[1].y);
      const d1 = Math.hypot(t[0].x - t[1].x, t[0].y - t[1].y);
      radius = Math.max(3, Math.min(26, radius * (d0 / d1)));
      upd();
    }
    lt = t;
  }, { passive: true });
  canvas.addEventListener('touchend', () => lt = null);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    radius = Math.max(3, Math.min(26, radius + e.deltaY * 0.012));
    upd();
  }, { passive: false });
  upd();
}

/* ── shared materials ── */
const MAT = {
  tower:    () => new THREE.MeshStandardMaterial({ color: 0x2A3A52, metalness: 0.6, roughness: 0.4 }),
  frame:    () => new THREE.MeshStandardMaterial({ color: 0x3E5370, metalness: 0.55, roughness: 0.45 }),
  ee:       () => new THREE.MeshStandardMaterial({ color: 0xE87722, metalness: 0.6, roughness: 0.3, emissive: 0x331100, emissiveIntensity: 0.6 }),
  joint:    () => new THREE.MeshStandardMaterial({ color: 0x111418, metalness: 0.4, roughness: 0.5 })
};

function clearGroup(g) {
  while (g.children.length) {
    const c = g.children[0];
    if (c.geometry) c.geometry.dispose();
    if (c.material) { Array.isArray(c.material) ? c.material.forEach(m => m.dispose()) : c.material.dispose(); }
    g.remove(c);
  }
}

/* cylinder between two world points */
function rod(p, q, radius, mat) {
  const a = new THREE.Vector3(...p), b = new THREE.Vector3(...q);
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  const geo = new THREE.CylinderGeometry(radius, radius, len, 14);
  const m = new THREE.Mesh(geo, mat);
  m.position.copy(a).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  m.castShadow = true;
  return m;
}
function ball(p, radius, mat) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), mat);
  m.position.set(...p); m.castShadow = true; return m;
}
function boxAt(p, w, hgt, d, mat, yawDir) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, hgt, d), mat);
  m.position.set(...p);
  if (yawDir) m.rotation.y = Math.atan2(yawDir[2], yawDir[0]);
  m.castShadow = true; return m;
}

/* ========================================================================
   REBUILD the linear delta from current carriage heights + geometry
   ====================================================================== */
function rebuildRobot(skipWV) {
  clearGroup(robotGroup);

  const fk = forwardKinematics(state.carriage);
  const Hw = state.H, r = state.r;

  // tower-top frame (triangle connecting the three tower tops)
  const tops = [];
  for (let i = 0; i < 3; i++) {
    const C = carriageXY(i);
    tops.push(robotToWorld(C.x, C.y, Hw));
  }
  for (let i = 0; i < 3; i++) {
    robotGroup.add(rod(tops[i], tops[(i + 1) % 3], 0.05, MAT.frame()));
    // base feet ring
    const C = carriageXY(i);
    const foot = robotToWorld(C.x, C.y, 0);
    robotGroup.add(rod(foot, robotToWorld(carriageXY((i + 1) % 3).x, carriageXY((i + 1) % 3).y, 0), 0.05, MAT.frame()));
  }

  let eeWorld = null;
  const effJointsW = [];

  for (let i = 0; i < 3; i++) {
    const C = carriageXY(i);
    const col_i = ARM_COL[i];
    const rodMat = new THREE.MeshStandardMaterial({ color: col_i, metalness: 0.45, roughness: 0.4, transparent: true, opacity: 0.92 });

    // vertical tower rail
    const railBot = robotToWorld(C.x, C.y, 0);
    const railTop = robotToWorld(C.x, C.y, Hw);
    robotGroup.add(rod(railBot, railTop, 0.07, MAT.tower()));

    // carriage block riding the rail at height zᵢ
    const zc = Math.max(0, Math.min(Hw, state.carriage[i]));
    const cw = robotToWorld(C.x, C.y, zc);
    const inwardDir = [-Math.cos(PHI[i]), 0, Math.sin(PHI[i])]; // points toward centre in world XZ
    const carriage = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.3, 0.22),
      new THREE.MeshStandardMaterial({ color: col_i, metalness: 0.5, roughness: 0.35 }));
    carriage.position.set(...cw);
    carriage.rotation.y = -PHI[i] + Math.PI / 2;
    carriage.castShadow = true;
    robotGroup.add(carriage);

    if (fk.reachable) {
      const P = fk.P;
      // platform joint (radius r inboard, same tower direction) at effector height
      const ej = { x: P.x + r * Math.cos(PHI[i]), y: P.y + r * Math.sin(PHI[i]), z: P.z };
      const ejW = robotToWorld(ej.x, ej.y, ej.z);
      effJointsW.push(ejW);
      // carriage joint (radius R) at carriage height
      const cjW = robotToWorld(C.x, C.y, zc);
      // parallelogram: two parallel diagonal rods offset tangentially
      const tang = new THREE.Vector3(-Math.sin(PHI[i]), 0, -Math.cos(PHI[i])).normalize();
      const off = 0.11;
      const ox = tang.x * off, oz = tang.z * off;
      robotGroup.add(rod([cjW[0] + ox, cjW[1], cjW[2] + oz], [ejW[0] + ox, ejW[1], ejW[2] + oz], 0.045, rodMat));
      robotGroup.add(rod([cjW[0] - ox, cjW[1], cjW[2] - oz], [ejW[0] - ox, ejW[1], ejW[2] - oz], 0.045, rodMat));
      robotGroup.add(ball(cjW, 0.1, MAT.joint()));
      robotGroup.add(ball(ejW, 0.09, MAT.joint()));
    }
  }

  // ── moving platform + tool ──
  if (fk.reachable) {
    const P = fk.P;
    eeWorld = robotToWorld(P.x, P.y, P.z);
    const plat = new THREE.Mesh(new THREE.CylinderGeometry(r + 0.13, r + 0.13, 0.1, 3), MAT.ee());
    plat.position.set(...eeWorld); plat.castShadow = true;
    robotGroup.add(plat);
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.1, 0.3, 18),
      new THREE.MeshStandardMaterial({ color: 0xffa33c, metalness: 0.5, roughness: 0.35, emissive: 0x553311, emissiveIntensity: 0.6 }));
    tip.position.set(eeWorld[0], eeWorld[1] - 0.22, eeWorld[2]);
    tip.rotation.x = Math.PI; tip.castShadow = true;
    robotGroup.add(tip);
    effJointsW.forEach(ejW => robotGroup.add(rod(eeWorld, ejW, 0.03, MAT.ee())));
  }

  if (wvVisible && !skipWV) buildWorkspaceViz();
  return fk;
}

/* ========================================================================
   WORKSPACE VISUALISATION — sampled reachable point cloud
   ====================================================================== */
function sampleWorkspace() {
  const RAD = ((state.R - state.r) + state.L) * 1.02;
  const zBot = 0, zTop = state.H * 0.94;
  const NR = 38, NA = 80, NZ = 54;
  const pts = [];
  let count = 0, total = 0;
  for (let zi = 0; zi <= NZ; zi++) {
    const z = zBot + (zTop - zBot) * zi / NZ;
    for (let ri = 1; ri <= NR; ri++) {
      const rad = RAD * ri / NR;
      const na = Math.max(6, Math.round(NA * ri / NR));
      for (let ai = 0; ai < na; ai++) {
        const ang = 2 * Math.PI * ai / na;
        const x = rad * Math.cos(ang), y = rad * Math.sin(ang);
        if (physReach(x, y, z)) pts.push([x, y, z]);
      }
    }
    if (physReach(0, 0, z)) pts.push([0, 0, z]);
  }
  const N = 12000;
  for (let k = 0; k < N; k++) {
    const rad = RAD * Math.sqrt(Math.random());
    const ang = 2 * Math.PI * Math.random();
    const z = zBot + (zTop - zBot) * Math.random();
    total++;
    if (physReach(rad * Math.cos(ang), rad * Math.sin(ang), z)) count++;
  }
  const boxVol = Math.PI * RAD * RAD * (zTop - zBot);
  const vol = boxVol * count / total;
  return { pts, vol, RAD, zTop, zBot };
}

let wvCache = null;
function buildWorkspaceViz() {
  if (wvGroup) { clearGroup(wvGroup); scene.remove(wvGroup); }
  wvGroup = new THREE.Group();
  scene.add(wvGroup);
  if (!wvCache || wvDirty) { wvCache = sampleWorkspace(); wvDirty = false; }

  const positions = [];
  wvCache.pts.forEach(([x, y, z]) => { const w = robotToWorld(x, y, z); positions.push(w[0], w[1], w[2]); });
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  const cloud = new THREE.Points(geo, new THREE.PointsMaterial({
    color: 0xE87722, size: 0.055, transparent: true, opacity: 0.38,
    sizeAttenuation: true, depthWrite: false
  }));
  cloud.renderOrder = -1;
  wvGroup.add(cloud);
  wvGroup.visible = wvVisible;
}

function toggleWorkspaceViz() {
  wvVisible = !wvVisible;
  const btn = document.getElementById('wv-toggle-btn');
  if (wvVisible) { buildWorkspaceViz(); btn.innerHTML = '&#9711; Hide Workspace'; btn.style.background = 'rgba(232,119,34,0.28)'; }
  else { if (wvGroup) wvGroup.visible = false; btn.innerHTML = '&#9711; Show Workspace'; btn.style.background = 'rgba(232,119,34,0.12)'; }
}

let _lastVW = 0, _lastVH = 0;
function animate() {
  requestAnimationFrame(animate);
  const vp = document.querySelector('.viewport');
  if (vp) {
    const w = vp.clientWidth, h = vp.clientHeight;
    if (w > 0 && h > 0 && (w !== _lastVW || h !== _lastVH)) {
      _lastVW = w; _lastVH = h;
      camera.aspect = w / h; camera.updateProjectionMatrix(); renderer.setSize(w, h);
    }
  }
  renderer.render(scene, camera);
}

/* ========================================================================
   OVERLAYS
   ====================================================================== */
function updateOverlays(fk) {
  const setEE = (id, v) => document.getElementById(id).textContent = (v == null ? '—' : fmt(v * MM, 1));
  if (fk.reachable) { setEE('ee-x', fk.P.x); setEE('ee-y', fk.P.y); setEE('ee-z', fk.P.z); }
  else { setEE('ee-x', null); setEE('ee-y', null); setEE('ee-z', null); }
  for (let i = 0; i < 3; i++) {
    document.getElementById('cz-' + (i + 1)).textContent = fmt(state.carriage[i] * MM, 1);
  }
  const reach = ((state.R - state.r) + state.L);
  document.getElementById('workspace-info').textContent =
    `reach · radial ${fmt(reach * MM, 0)} mm · towers ${fmt(state.H * MM, 0)} mm`;
}

/* ========================================================================
   TRAJECTORY PLAYER
   ====================================================================== */
const TRAJ = {
  circle:  { label: 'Circle',   rad: 0.72, f: p => { const a = 2 * Math.PI * p; return { x: Math.cos(a), y: Math.sin(a), nz: 0 }; } },
  figure8: { label: 'Figure-8', rad: 0.80, f: p => { const a = 2 * Math.PI * p; return { x: Math.sin(a), y: 0.55 * Math.sin(2 * a), nz: 0 }; } },
  spiral:  { label: 'Spiral',   rad: 0.78, f: p => { const u = p < 0.5 ? p * 2 : (1 - p) * 2; const a = 2 * Math.PI * 3 * u; return { x: u * Math.cos(a), y: u * Math.sin(a), nz: 0 }; } },
  helix:   { label: 'Helix',    rad: 0.55, vspan: true, taper: true, f: p => { const a = 2 * Math.PI * 3 * p; return { x: Math.cos(a), y: Math.sin(a), nz: p }; } },
  square:  { label: 'Square',   rad: 0.66, f: p => { const s = (p * 4) % 4; let x, y; if (s < 1) { x = -1 + 2 * s; y = -1; } else if (s < 2) { x = 1; y = -1 + 2 * (s - 1); } else if (s < 3) { x = 1 - 2 * (s - 2); y = 1; } else { x = -1; y = 1 - 2 * (s - 3); } return { x, y, nz: 0 }; } },
  rose:    { label: 'Rose',     rad: 0.80, f: p => { const a = 2 * Math.PI * p; const rr = Math.cos(3 * a); return { x: rr * Math.cos(a), y: rr * Math.sin(a), nz: 0 }; } }
};

let trajGroup = null, trajMarker = null;
const traj = { name: null, playing: false, phase: 0, speedMul: 1, period: 7000, last: 0, raf: null, fit: null };

// Probe the reachable envelope: the z-slice with the widest all-azimuth radius.
function fitWorkspace() {
  const az = [0, 1, 2, 3, 4, 5].map(k => k * Math.PI / 3);
  let best = { Zc: state.H * 0.4, Rw: 0.5 };
  const zTop = state.H * 0.92, zBot = 0.15;
  for (let z = zBot; z <= zTop; z += 0.08) {
    let rmax = 0;
    const lim = (state.R - state.r) + state.L;
    for (let r = 0; r <= lim; r += 0.04) {
      if (az.every(a => physReach(r * Math.cos(a), r * Math.sin(a), z))) rmax = r; else break;
    }
    if (rmax > best.Rw) best = { Zc: z, Rw: rmax };
  }
  return best;
}

function trajTarget(name, p) {
  const def = TRAJ[name], n = def.f(p);
  let rad = def.rad * traj.fit.Rw;
  if (def.taper) rad *= 1 - 0.28 * n.nz;
  const zspan = def.vspan ? Math.min(1.1, state.H * 0.32) : 0;
  return { x: rad * n.x, y: rad * n.y, z: traj.fit.Zc + n.nz * zspan };
}

function buildTrajPath(name) {
  if (trajGroup) { clearGroup(trajGroup); scene.remove(trajGroup); }
  trajGroup = new THREE.Group(); scene.add(trajGroup);
  const N = 480, pts = [];
  for (let i = 0; i <= N; i++) { const t = trajTarget(name, i / N); pts.push(new THREE.Vector3(...robotToWorld(t.x, t.y, t.z))); }
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  trajGroup.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0x7C4DD0, transparent: true, opacity: 0.9 })));
  trajMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.1, 18, 14),
    new THREE.MeshStandardMaterial({ color: 0xE87722, emissive: 0x3a1500, emissiveIntensity: 0.7, metalness: 0.3, roughness: 0.4 })
  );
  trajGroup.add(trajMarker);
}

function stepTrajTo(p) {
  const t = trajTarget(traj.name, p);
  const sol = inverseKinematics(t.x, t.y, t.z);
  if (sol.reachable) { state.carriage = sol.carriages.slice(); for (let i = 0; i < 3; i++) setCarriageUI(i, state.carriage[i] * MM); }
  const fk = rebuildRobot(true); updateOverlays(fk);
  if (trajMarker) trajMarker.position.set(...robotToWorld(t.x, t.y, t.z));
  const prog = document.getElementById('traj-prog');
  if (prog) {
    prog.className = 'ik-mini ' + (sol.reachable ? 'ok' : 'err');
    prog.textContent = `${TRAJ[traj.name].label} · ${Math.round(p * 100)}%` + (sol.reachable ? '' : ' · point out of reach');
  }
}

function setTrajPlaying(on) {
  traj.playing = on && !!traj.name;
  const btn = document.getElementById('traj-play');
  if (btn) btn.textContent = traj.playing ? '❚❚ Pause' : '▶ Play';
  if (traj.playing) { traj.last = performance.now(); loopTraj(); }
  else if (traj.raf) { cancelAnimationFrame(traj.raf); traj.raf = null; }
}

function loopTraj() {
  if (!traj.playing) return;
  traj.raf = requestAnimationFrame(loopTraj);
  const now = performance.now(), dt = now - traj.last; traj.last = now;
  traj.phase = (traj.phase + (dt / traj.period) * traj.speedMul) % 1;
  if (traj.phase < 0) traj.phase += 1;
  stepTrajTo(traj.phase);
}

function selectTraj(name) {
  readInputs();
  traj.name = name; traj.phase = 0; traj.fit = fitWorkspace();
  buildTrajPath(name);
  document.querySelectorAll('.traj-btn').forEach(b => b.classList.toggle('active', b.dataset.traj === name));
  stepTrajTo(0);
  setTrajPlaying(true);
}

function stopTraj() {
  setTrajPlaying(false);
  traj.name = null; traj.phase = 0;
  if (trajGroup) { clearGroup(trajGroup); scene.remove(trajGroup); trajGroup = null; trajMarker = null; }
  document.querySelectorAll('.traj-btn').forEach(b => b.classList.remove('active'));
  const prog = document.getElementById('traj-prog');
  if (prog) { prog.className = 'ik-mini'; prog.textContent = 'select a path — the platform traces it automatically.'; }
}

function pauseTrajForManual() { if (traj.playing) setTrajPlaying(false); }

/* ========================================================================
   WIRING
   ====================================================================== */
function refresh() {
  readInputs();
  const fk = rebuildRobot();
  updateOverlays(fk);
}

// reflect a carriage height (mm) into both its slider and number input
function setCarriageUI(i, mm) {
  const v = Math.round(mm);
  document.getElementById(`j${i + 1}-slider`).value = v;
  document.getElementById(`j${i + 1}-num`).value = Number.isInteger(mm) ? mm : +mm.toFixed(1);
}

// keep carriage slider/number ranges in sync with tower height H
function syncCarriageRanges() {
  const Hmm = Math.round(state.H * MM);
  for (let i = 0; i < 3; i++) {
    const s = document.getElementById(`j${i + 1}-slider`), n = document.getElementById(`j${i + 1}-num`);
    s.max = Hmm; n.max = Hmm;
    if (parseFloat(s.value) > Hmm) { s.value = Hmm; n.value = Hmm; state.carriage[i] = state.H; }
  }
}

function applyIK(sol) {
  const mini = document.getElementById('ik-status-mini');
  if (!sol.reachable) {
    mini.textContent = '✗ unreachable — target lies outside the workspace.';
    mini.className = 'ik-mini err';
    return;
  }
  const outOfTravel = sol.carriages.some(c => c < -1e-6 || c > state.H + 1e-6);
  state.carriage = sol.carriages.slice();
  for (let i = 0; i < 3; i++) setCarriageUI(i, state.carriage[i] * MM);
  if (outOfTravel) {
    mini.textContent = `△ rods close, but a carriage exceeds travel — z = ${state.carriage.map(c => fmt(c * MM, 0)).join(', ')} mm`;
    mini.className = 'ik-mini err';
  } else {
    mini.textContent = `✓ solved — z = ${state.carriage.map(c => fmt(c * MM, 1)).join(', ')} mm`;
    mini.className = 'ik-mini ok';
  }
  refresh();
}

function wireEvents() {
  // geometry sliders ↔ number inputs (mm)
  const geo = [['R', 'R-slider'], ['r', 'r-slider'], ['L', 'L-slider'], ['H', 'H-slider']];
  geo.forEach(([num, sl]) => {
    const n = document.getElementById(num), s = document.getElementById(sl);
    const onGeo = () => {
      readInputs();
      syncCarriageRanges();
      wvDirty = true; if (wvVisible) buildWorkspaceViz();
      if (traj.name) { traj.fit = fitWorkspace(); buildTrajPath(traj.name); }
      refresh();
    };
    s.addEventListener('input', () => { n.value = s.value; onGeo(); });
    n.addEventListener('input', () => { s.value = n.value; onGeo(); });
  });

  // carriage heights: slider ↔ direct number input (mm)
  for (let i = 0; i < 3; i++) {
    const s = document.getElementById(`j${i + 1}-slider`);
    const n = document.getElementById(`j${i + 1}-num`);
    const drive = mm => {
      const v = Math.max(0, Math.min(state.H * MM, mm));
      state.carriage[i] = v / MM;
      const fk = rebuildRobot(); updateOverlays(fk);
      return v;
    };
    s.addEventListener('input', () => { pauseTrajForManual(); const v = drive(parseFloat(s.value) || 0); n.value = v; });
    n.addEventListener('input', () => {
      const raw = parseFloat(n.value);
      if (isNaN(raw)) return;
      pauseTrajForManual();
      const v = drive(raw); s.value = Math.round(v);
    });
    n.addEventListener('change', () => { n.value = Math.max(0, Math.min(state.H * MM, parseFloat(n.value) || 0)); });
  }

  document.getElementById('btn-solve').addEventListener('click', () => {
    pauseTrajForManual();
    readInputs();
    const sol = inverseKinematics(state.targetX, state.targetY, state.targetZ);
    state.ikSolution = sol;
    targetMarker.position.set(...robotToWorld(state.targetX, state.targetY, state.targetZ));
    targetMarker.visible = true;
    applyIK(sol);
  });

  document.getElementById('btn-reset').addEventListener('click', () => {
    stopTraj();
    const def = { R: 140, r: 30, L: 300, H: 440 };   // mm
    Object.entries(def).forEach(([k, v]) => {
      document.getElementById(k).value = v;
      document.getElementById(k + '-slider').value = v;
    });
    readInputs();
    syncCarriageRanges();
    state.carriage = [3.6, 3.6, 3.6];
    for (let i = 0; i < 3; i++) setCarriageUI(i, 360);
    ['target-x', 'target-y', 'target-z'].forEach((id, k) => document.getElementById(id).value = [40, 30, 90][k]);
    targetMarker.visible = false;
    state.ikSolution = null; wvDirty = true; wvCache = null;
    if (wvVisible) buildWorkspaceViz();
    const mini = document.getElementById('ik-status-mini');
    mini.textContent = 'enter a target (X, Y, Z in mm) and click “solve ik” — the carriages move to the pose.';
    mini.className = 'ik-mini';
    refresh();
  });

  // trajectory player
  document.querySelectorAll('.traj-btn').forEach(b => {
    b.addEventListener('click', () => selectTraj(b.dataset.traj));
  });
  document.getElementById('traj-play').addEventListener('click', () => {
    if (!traj.name) { selectTraj('circle'); return; }
    setTrajPlaying(!traj.playing);
  });
  document.getElementById('traj-stop').addEventListener('click', stopTraj);
  const sp = document.getElementById('traj-speed');
  sp.addEventListener('input', () => {
    traj.speedMul = parseFloat(sp.value) || 1;
    document.getElementById('traj-speed-val').textContent = traj.speedMul.toFixed(2).replace(/0$/, '') + '\u00d7';
  });
}

function boot() {
  initThree();
  wireEvents();
  readInputs();
  syncCarriageRanges();
  refresh();
}
if (document.readyState === 'loading') window.addEventListener('DOMContentLoaded', boot);
else boot();
