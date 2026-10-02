/* ========================================================================
   DELTA ROBOT SIMULATOR  ·  Orca Robotics
   3-RUU parallel manipulator — forward & inverse kinematics
   ------------------------------------------------------------------------
   Kinematics follow the classic delta-robot tutorial by mzavatsky
   (after Zsombor-Murray), forums.trossenrobotics.com — i.e. the
   delta_calcForward / delta_calcAngleYZ / delta_calcInverse routines,
   ported to JavaScript here.
   ------------------------------------------------------------------------
   Tutorial geometry (origin at centre of the fixed base triangle, Z up,
   end-effector hangs below so z₀ < 0):
     • f   side of the fixed base triangle   (motors at the side midpoints)
     • e   side of the moving platform triangle
     • rf  upper joint length  (driven "bicep")
     • re  parallelogram / forearm length

   The on-screen controls use the equivalent radius form; the apothem of
   an equilateral triangle of side s is s/(2√3), so:
     R = f/(2√3)  (shoulder radius)      ⇒  f  = 2√3·R
     r = e/(2√3)  (platform radius)      ⇒  e  = 2√3·r
     L = rf       (upper arm)            ⇒  rf = L
     ℓ = re       (forearm)              ⇒  re = ℓ

   Arm 1 lies in the YZ plane (shoulder on −Y); arms 2 & 3 are the same
   solution with the target rotated by ±120° about Z.
   ====================================================================== */

const deg2rad = d => d * Math.PI / 180;
const rad2deg = r => r * 180 / Math.PI;
const fmt = (n, p = 3) => (Math.abs(n) < 1e-10 ? 0 : n).toFixed(p);

const PHI = [270, 30, 150].map(deg2rad);              // arm azimuths — arm 1 on −Y (tutorial frame)
const ARM_COL = [0x2A6FDB, 0x1F8A5B, 0xC7468A];       // blue / green / pink
const ARM_HEX = ['#2A6FDB', '#1F8A5B', '#C7468A'];

// trigonometric constants from the tutorial
const SQRT3 = Math.sqrt(3);
const TAN30 = 1 / SQRT3, TAN60 = SQRT3, SIN30 = 0.5;
const COS120 = -0.5, SIN120 = SQRT3 / 2;

// radius-form controls (R, ℓ, …) → tutorial triangle sides (f, e, rf, re)
function tutorialGeom() {
  return { f: 2 * SQRT3 * state.R, e: 2 * SQRT3 * state.r, rf: state.L, re: state.l };
}
const BASE_Y = 3.7;                                   // world height of base plate
const TH_MIN = -40, TH_MAX = 110;                     // motor travel limits (deg)
const MM = 100;                                       // mm per internal world unit (UI shows mm; math stays in world units)

// A point is physically reachable if every arm has SOME in-range motor solution
// (either assembly branch counts — each arm reaches its foot independently).
function physReach(x, y, z) {
  const s = inverseKinematics(x, y, z);
  if (!s.reachable) return false;
  return s.arms.every(a => a.theta >= TH_MIN - 1e-6 && a.theta <= TH_MAX + 1e-6);
}

const state = {
  R: 1.6, L: 1.2, l: 2.6, r: 0.5,
  theta: [22, 22, 22],                                // motor angles (deg)
  targetX: 0.4, targetY: 0.3, targetZ: -2.2,
  ikSolution: null
};

function readInputs() {
  const g = id => parseFloat(document.getElementById(id).value) || 0;
  // geometry + target inputs are in mm; internal kinematics work in world units
  state.R = g('R') / MM; state.L = g('L') / MM; state.l = g('Ll') / MM; state.r = g('r') / MM;
  state.targetX = g('target-x') / MM; state.targetY = g('target-y') / MM; state.targetZ = g('target-z') / MM;
}

/* ── robot frame (X,Y,Z, Z up) → three.js world (Y up) ── */
function robotToWorld(X, Y, Z) { return [X, BASE_Y + Z, -Y]; }

/* ========================================================================
   ELBOW position for arm i given motor angle θ (deg)
   ====================================================================== */
function elbow(i, thetaDeg) {
  const t = deg2rad(thetaDeg);
  const rad = state.R + state.L * Math.cos(t);
  return {
    x: rad * Math.cos(PHI[i]),
    y: rad * Math.sin(PHI[i]),
    z: -state.L * Math.sin(t)
  };
}

/* ========================================================================
   FORWARD KINEMATICS  —  (θ₁,θ₂,θ₃) → (x₀,y₀,z₀)
   ------------------------------------------------------------------------
   Three-spheres derivation (verbatim from the supplied sheet).  Each
   forearm of length rₑ ties its knee (xᵢ,yᵢ,zᵢ) to the platform centre, so
   E₀ lies on three spheres of radius rₑ:

       (x−xᵢ)² + (y−yᵢ)² + (z−zᵢ)² = rₑ²            (1)(2)(3)

   With wᵢ = xᵢ²+yᵢ²+zᵢ² and arm 1 in the YZ plane (x₁=0), subtracting the
   spheres pairwise gives the linear system

       x₂x + (y₁−y₂)y + (z₁−z₂)z = (w₁−w₂)/2        (4) = (1)−(2)
       x₃x + (y₁−y₃)y + (z₁−z₃)z = (w₁−w₃)/2        (5) = (1)−(3)

   solved for x and y as functions of z:

       x = a₁z + b₁    (7)        y = a₂z + b₂    (8)

       a₁ = ⅟d[(z₂−z₁)(y₃−y₁) − (z₃−z₁)(y₂−y₁)]
       a₂ = −⅟d[(z₂−z₁)x₃ − (z₃−z₁)x₂]
       b₁ = −1/2d[(w₂−w₁)(y₃−y₁) − (w₃−w₁)(y₂−y₁)]
       b₂ =  1/2d[(w₂−w₁)x₃ − (w₃−w₁)x₂]
       d  = (y₂−y₁)x₃ − (y₃−y₁)x₂

   Substituting (7)(8) into (1) leaves a single quadratic in z:

       (a₁²+a₂²+1)z² + 2(a₁b₁ + a₂(b₂−y₁) − z₁)z
                     + (b₁² + (b₂−y₁)² + z₁² − rₑ²) = 0

   The smaller (more-negative) root is the physical solution — the platform
   hangs below the base.
   ====================================================================== */
function forwardKinematics(thetaDeg) {
  const th = thetaDeg || state.theta;
  const { f, e, rf, re } = tutorialGeom();
  const t = (f - e) * TAN30 / 2;        // base apothem − effector apothem  (= R − r)
  const dtr = Math.PI / 180;
  const th1 = th[0] * dtr, th2 = th[1] * dtr, th3 = th[2] * dtr;

  // knee points (xᵢ,yᵢ,zᵢ) — already shifted in by the platform offset → sphere centres
  const x1 = 0;
  const y1 = -(t + rf * Math.cos(th1));
  const z1 = -rf * Math.sin(th1);

  const y2 = (t + rf * Math.cos(th2)) * SIN30;
  const x2 = y2 * TAN60;
  const z2 = -rf * Math.sin(th2);

  const y3 = (t + rf * Math.cos(th3)) * SIN30;
  const x3 = -y3 * TAN60;
  const z3 = -rf * Math.sin(th3);

  const J = [{ x: x1, y: y1, z: z1 }, { x: x2, y: y2, z: z2 }, { x: x3, y: y3, z: z3 }];

  // wᵢ = xᵢ² + yᵢ² + zᵢ²
  const w1 = x1 * x1 + y1 * y1 + z1 * z1;
  const w2 = x2 * x2 + y2 * y2 + z2 * z2;
  const w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // d = (y₂−y₁)x₃ − (y₃−y₁)x₂
  const d = (y2 - y1) * x3 - (y3 - y1) * x2;
  if (Math.abs(d) < 1e-12) return { reachable: false, reason: 'degenerate geometry', J, t, re };

  // x = a₁z + b₁   (7)
  const a1 = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / d;
  const b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / (2 * d);
  // y = a₂z + b₂   (8)
  const a2 = -((z2 - z1) * x3 - (z3 - z1) * x2) / d;
  const b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / (2 * d);

  // (a₁²+a₂²+1)z² + 2(a₁b₁ + a₂(b₂−y₁) − z₁)z + (b₁²+(b₂−y₁)²+z₁²−rₑ²) = 0
  const A = a1 * a1 + a2 * a2 + 1;
  const B = 2 * (a1 * b1 + a2 * (b2 - y1) - z1);
  const Cc = b1 * b1 + (b2 - y1) * (b2 - y1) + z1 * z1 - re * re;

  const disc = B * B - 4 * A * Cc;
  if (disc < 0) return { reachable: false, reason: 'non-existing position — forearms cannot close', J, t, re };

  const z0 = -0.5 * (B + Math.sqrt(disc)) / A;   // smaller root → platform below base
  const x0 = a1 * z0 + b1;
  const y0 = a2 * z0 + b2;
  return {
    reachable: true, P: { x: x0, y: y0, z: z0 }, J, t, re,
    coef: { w1, w2, w3, d, a1, a2, b1, b2, A, B, C: Cc, disc }
  };
}

/* ========================================================================
   INVERSE KINEMATICS (one arm) — (x₀,y₀,z₀) → θᵢ        [ delta_calcAngleYZ ]
   Joint F1J1 only rotates in the YZ plane (circle, centre F1, radius rf).
   The forearm makes E'1J1 a circle of radius re about the projection of the
   foot. J1 is the lower intersection of those two circles; θ follows from it.
   ====================================================================== */
function calcAngleYZ(x0, y0, z0) {
  const { f, e, rf, re } = tutorialGeom();
  const y1 = -0.5 * TAN30 * f;          // shoulder F on the −Y axis  (= −R)
  const y0s = y0 - 0.5 * TAN30 * e;     // shift platform centre to the arm's foot
  if (Math.abs(z0) < 1e-9) return { reachable: false, y1, y0s };

  // the foot lies on the line z = a + b·y inside the arm's YZ plane
  const a = (x0 * x0 + y0s * y0s + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  const b = (y1 - y0s) / z0;
  // discriminant of the circle/line intersection
  const d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);   // = −(a+b·y1)² + rf²(b²+1)
  if (d < 0) return { reachable: false, a, b, d, y1, y0s };

  const yj = (y1 - a * b - Math.sqrt(d)) / (b * b + 1);   // outer (lower-Y) intersection → J
  const zj = a + b * yj;
  const theta = 180 * Math.atan(-zj / (y1 - yj)) / Math.PI + (yj > y1 ? 180 : 0);
  return { reachable: true, theta, a, b, d, yj, zj, y1, y0s };
}

/* (x₀,y₀,z₀) → (θ₁,θ₂,θ₃)   — arm 1 direct, arms 2 & 3 with target rotated ±120° */
function inverseKinematics(x, y, z) {
  const arms = [
    calcAngleYZ(x, y, z),                                                 // arm 1  (YZ plane)
    calcAngleYZ(x * COS120 + y * SIN120, y * COS120 - x * SIN120, z),     // rotate +120°
    calcAngleYZ(x * COS120 - y * SIN120, y * COS120 + x * SIN120, z)      // rotate −120°
  ];
  return { reachable: arms.every(a => a.reachable), arms, x, y, z };
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
  scene.fog = new THREE.Fog(0xECECEA, 18, 40);

  camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 100);
  camera.position.set(5.5, 4.2, 7);

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  scene.add(new THREE.AmbientLight(0xffffff, 0.85));
  const key = new THREE.DirectionalLight(0xffffff, 1.05);
  key.position.set(6, 11, 5);
  key.castShadow = true;
  key.shadow.camera.left = -9; key.shadow.camera.right = 9;
  key.shadow.camera.top = 9; key.shadow.camera.bottom = -9;
  key.shadow.mapSize.width = 1024; key.shadow.mapSize.height = 1024;
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xE87722, 0.28);
  fill.position.set(-4, 3, -3);
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

  // target marker (small ring) lives in world space
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
  // keep the renderer matched to the viewport box (handles panel collapse + transitions)
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
  const target = new THREE.Vector3(0, 1.7, 0);
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
      radius = Math.max(2.5, Math.min(24, radius * (d0 / d1)));
      upd();
    }
    lt = t;
  }, { passive: true });
  canvas.addEventListener('touchend', () => lt = null);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    radius = Math.max(2.5, Math.min(24, radius + e.deltaY * 0.012));
    upd();
  }, { passive: false });
  upd();
}

/* ── shared materials ── */
const MAT = {
  baseTop:  () => new THREE.MeshStandardMaterial({ color: 0x3E5370, metalness: 0.55, roughness: 0.45, transparent: true, opacity: 0.45, side: THREE.DoubleSide }),
  motor:    () => new THREE.MeshStandardMaterial({ color: 0x2A3A52, metalness: 0.7, roughness: 0.35 }),
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

/* ========================================================================
   REBUILD the delta robot from current θ + geometry
   ====================================================================== */
function rebuildRobot(skipWV) {
  clearGroup(robotGroup);

  const fk = forwardKinematics(state.theta);
  const Rr = state.R, L = state.L, r = state.r;

  // ── base plate (hexagonal prism) at top ──
  const plate = new THREE.Mesh(
    new THREE.CylinderGeometry(Rr + 0.55, Rr + 0.65, 0.22, 3),
    MAT.baseTop()
  );
  plate.rotation.y = 0;                  // a triangle vertex points toward each arm
  plate.position.set(0, BASE_Y + 0.11, 0);
  plate.castShadow = true; plate.receiveShadow = true;
  robotGroup.add(plate);



  let eeWorld = null, footW = [];

  for (let i = 0; i < 3; i++) {
    const u = [Math.cos(PHI[i]), Math.sin(PHI[i]), 0];
    const col_i = ARM_COL[i];
    const upMat = new THREE.MeshStandardMaterial({ color: col_i, metalness: 0.5, roughness: 0.35 });
    const loMat = new THREE.MeshStandardMaterial({ color: col_i, metalness: 0.45, roughness: 0.4, transparent: true, opacity: 0.92 });

    // shoulder & elbow in robot frame
    const S = { x: Rr * u[0], y: Rr * u[1], z: 0 };
    const E = elbow(i, state.theta[i]);
    const Sw = robotToWorld(S.x, S.y, S.z);
    const Ew = robotToWorld(E.x, E.y, E.z);

    // motor housing at shoulder
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.34, 22), MAT.motor());
    motor.position.set(...Sw);
    // orient drum axis tangentially (perpendicular to radial & vertical)
    const tang = new THREE.Vector3(-u[1], 0, -u[0]); // world tangential (robot dir (−sinφ,cosφ,0) → world)
    motor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tang);
    motor.castShadow = true;
    robotGroup.add(motor);

    // upper arm
    robotGroup.add(rod(Sw, Ew, 0.085, upMat));
    robotGroup.add(ball(Sw, 0.13, MAT.joint()));
    robotGroup.add(ball(Ew, 0.12, MAT.joint()));

    if (fk.reachable) {
      const P = fk.P;
      const foot = { x: P.x + r * u[0], y: P.y + r * u[1], z: P.z };
      const Fw = robotToWorld(foot.x, foot.y, foot.z);
      footW.push(Fw);
      // parallelogram forearm: two parallel rods offset tangentially
      const off = 0.12;
      const ox = tang.x * off, oz = tang.z * off;
      robotGroup.add(rod([Ew[0] + ox, Ew[1], Ew[2] + oz], [Fw[0] + ox, Fw[1], Fw[2] + oz], 0.05, loMat));
      robotGroup.add(rod([Ew[0] - ox, Ew[1], Ew[2] - oz], [Fw[0] - ox, Fw[1], Fw[2] - oz], 0.05, loMat));
      robotGroup.add(ball(Fw, 0.1, MAT.joint()));
    }
  }

  // ── moving platform + EE ──
  if (fk.reachable) {
    const P = fk.P;
    eeWorld = robotToWorld(P.x, P.y, P.z);
    const plat = new THREE.Mesh(new THREE.CylinderGeometry(r + 0.14, r + 0.14, 0.1, 3), MAT.ee());
    plat.rotation.y = 0;                 // vertex points toward each arm/foot
    plat.position.set(...eeWorld); plat.castShadow = true;
    robotGroup.add(plat);
    // tool tip cone pointing down
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.1, 0.3, 18), new THREE.MeshStandardMaterial({ color: 0xffa33c, metalness: 0.5, roughness: 0.35, emissive: 0x553311, emissiveIntensity: 0.6 }));
    tip.position.set(eeWorld[0], eeWorld[1] - 0.22, eeWorld[2]);
    tip.rotation.x = Math.PI; tip.castShadow = true;
    robotGroup.add(tip);
    // connect platform centre to each foot
    footW.forEach(Fw => robotGroup.add(rod(eeWorld, Fw, 0.035, MAT.ee())));
  }

  if (wvVisible && !skipWV) buildWorkspaceViz();
  return fk;
}

/* ========================================================================
   WORKSPACE VISUALISATION — sampled reachable point cloud
   ====================================================================== */
function sampleWorkspace() {
  const reach = state.R - state.r + state.L + state.l;
  const RAD = reach * 1.05;
  const zTop = 0.05, zBot = -(state.L + state.l) * 1.02;   // platform stays at / below the base plane
  const NR = 40, NA = 84, NZ = 56;
  const pts = [];
  let count = 0, total = 0;
  const cellV = (RAD / NR) * (RAD / NR) * 0; // not used; volume via Monte-Carlo below
  // cylindrical sampling for the point cloud
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
    // include axis point
    if (physReach(0, 0, z)) pts.push([0, 0, z]);
  }
  // Monte-Carlo volume estimate inside bounding cylinder
  const box = { r: RAD, zTop, zBot };
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
    sizeAttenuation: true, depthWrite: false   // blend as haze; never occlude the solid platform
  }));
  cloud.renderOrder = -1;                       // draw the cloud behind the robot/EE
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
   RIGHT-PANEL RENDERERS
   ====================================================================== */
function updateOverlays(fk) {
  // end-effector readout (shown in mm)
  const setEE = (id, v) => document.getElementById(id).textContent = (v == null ? '—' : fmt(v * MM, 1));
  if (fk.reachable) { setEE('ee-x', fk.P.x); setEE('ee-y', fk.P.y); setEE('ee-z', fk.P.z); }
  else { setEE('ee-x', null); setEE('ee-y', null); setEE('ee-z', null); }

  const reach = state.R - state.r + state.L + state.l;
  document.getElementById('workspace-info').textContent =
    `reach · radial ${fmt(reach * MM, 0)} mm · z ${fmt(-(state.L + state.l) * MM, 0)} mm`;
}

/* ========================================================================
   TRAJECTORY PLAYER — preset end-effector paths driven through IK
   ------------------------------------------------------------------------
   Each preset is a normalised parametric shape f(p), p∈[0,1). It is fitted
   to the reachable envelope (centre Zc, working radius Rw probed live) and
   replayed by solving IK every frame to drive the three motors.
   ====================================================================== */
const TRAJ = {
  circle:  { label: 'Circle',   rad: 0.72, f: p => { const a = 2 * Math.PI * p; return { x: Math.cos(a), y: Math.sin(a), nz: 0 }; } },
  figure8: { label: 'Figure-8', rad: 0.80, f: p => { const a = 2 * Math.PI * p; return { x: Math.sin(a), y: 0.55 * Math.sin(2 * a), nz: 0 }; } },
  spiral:  { label: 'Spiral',   rad: 0.78, f: p => { const u = p < 0.5 ? p * 2 : (1 - p) * 2; const a = 2 * Math.PI * 3 * u; return { x: u * Math.cos(a), y: u * Math.sin(a), nz: 0 }; } },
  helix:   { label: 'Helix',    rad: 0.55, vspan: true, taper: true, f: p => { const a = 2 * Math.PI * 3 * p; return { x: Math.cos(a), y: Math.sin(a), nz: -p }; } },
  square:  { label: 'Square',   rad: 0.66, f: p => { const s = (p * 4) % 4; let x, y; if (s < 1) { x = -1 + 2 * s; y = -1; } else if (s < 2) { x = 1; y = -1 + 2 * (s - 1); } else if (s < 3) { x = 1 - 2 * (s - 2); y = 1; } else { x = -1; y = 1 - 2 * (s - 3); } return { x, y, nz: 0 }; } },
  rose:    { label: 'Rose',     rad: 0.80, f: p => { const a = 2 * Math.PI * p; const rr = Math.cos(3 * a); return { x: rr * Math.cos(a), y: rr * Math.sin(a), nz: 0 }; } }
};

let trajGroup = null, trajMarker = null;
const traj = { name: null, playing: false, phase: 0, speedMul: 1, period: 7000, last: 0, raf: null, fit: null };

// Probe the reachable envelope: the z-slice with the widest all-azimuth radius.
function fitWorkspace() {
  const az = [0, 1, 2, 3, 4, 5].map(k => k * Math.PI / 3);
  let best = { Zc: -(state.L + state.l) * 0.5, Rw: 0.6 };
  const zTop = -0.3, zBot = -(state.L + state.l) * 0.96;
  for (let z = zTop; z >= zBot; z -= 0.08) {
    let rmax = 0;
    for (let r = 0; r <= 2.6; r += 0.04) {
      if (az.every(a => physReach(r * Math.cos(a), r * Math.sin(a), z))) rmax = r; else break;
    }
    if (rmax > best.Rw) best = { Zc: z, Rw: rmax };
  }
  return best;
}

function trajTarget(name, p) {
  const def = TRAJ[name], n = def.f(p);
  let rad = def.rad * traj.fit.Rw;
  if (def.taper) rad *= 1 - 0.28 * (-n.nz);            // shrink radius as the coil descends
  const zspan = def.vspan ? Math.min(1.1, (state.L + state.l) * 0.30) : 0;
  return { x: rad * n.x, y: rad * n.y, z: traj.fit.Zc + n.nz * zspan };   // helix nz∈[0,-1] → from Zc downward
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
  if (sol.reachable) { state.theta = sol.arms.map(a => a.theta); for (let i = 0; i < 3; i++) setAngleUI(i, state.theta[i]); }
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

// pause an active trajectory when the user grabs a manual control
function pauseTrajForManual() { if (traj.playing) setTrajPlaying(false); }

/* ========================================================================
   WIRING
   ====================================================================== */
function refresh() {
  readInputs();
  const fk = rebuildRobot();
  updateOverlays(fk);
}

// reflect a motor angle into both its slider and number input
function setAngleUI(i, v) {
  document.getElementById(`j${i + 1}-slider`).value = Math.round(v);
  document.getElementById(`j${i + 1}-num`).value = Number.isInteger(v) ? v : +v.toFixed(1);
}

function applyIK(sol) {
  const mini = document.getElementById('ik-status-mini');
  if (!sol.reachable) {
    mini.textContent = '✗ unreachable — target lies outside the workspace.';
    mini.className = 'ik-mini err';
    return;
  }
  state.theta = sol.arms.map(a => a.theta);
  for (let i = 0; i < 3; i++) setAngleUI(i, state.theta[i]);
  mini.textContent = `✓ solved — θ = ${state.theta.map(t => fmt(t, 1)).join('°, ')}°`;
  mini.className = 'ik-mini ok';
  refresh();
}

function wireEvents() {
  // geometry sliders ↔ number inputs (mm)
  const geo = [['R', 'R-slider'], ['L', 'L-slider'], ['Ll', 'Ll-slider'], ['r', 'r-slider']];
  geo.forEach(([num, sl]) => {
    const n = document.getElementById(num), s = document.getElementById(sl);
    const onGeo = () => {
      wvDirty = true; if (wvVisible) buildWorkspaceViz();
      if (traj.name) { traj.fit = fitWorkspace(); buildTrajPath(traj.name); }   // refit path to new geometry
      refresh();
    };
    s.addEventListener('input', () => { n.value = s.value; onGeo(); });
    n.addEventListener('input', () => { s.value = n.value; onGeo(); });
  });

  // motor angles: slider ↔ direct number input
  for (let i = 0; i < 3; i++) {
    const s = document.getElementById(`j${i + 1}-slider`);
    const n = document.getElementById(`j${i + 1}-num`);
    const drive = v => {
      v = Math.max(TH_MIN, Math.min(TH_MAX, v));
      state.theta[i] = v;
      const fk = rebuildRobot(); updateOverlays(fk);
      return v;
    };
    s.addEventListener('input', () => { pauseTrajForManual(); const v = drive(parseFloat(s.value) || 0); n.value = v; });
    n.addEventListener('input', () => {
      const raw = parseFloat(n.value);
      if (isNaN(raw)) return;                 // let the user finish typing "-" etc.
      pauseTrajForManual();
      const v = drive(raw); s.value = Math.round(v);
    });
    n.addEventListener('change', () => { n.value = Math.max(TH_MIN, Math.min(TH_MAX, parseFloat(n.value) || 0)); });
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
    const def = { R: 160, L: 120, Ll: 260, r: 50 };   // mm
    Object.entries(def).forEach(([k, v]) => {
      document.getElementById(k).value = v;
      document.getElementById(k + '-slider').value = v;
    });
    state.theta = [22, 22, 22];
    for (let i = 0; i < 3; i++) setAngleUI(i, 22);
    ['target-x', 'target-y', 'target-z'].forEach((id, k) => document.getElementById(id).value = [40, 30, -220][k]);
    targetMarker.visible = false;
    state.ikSolution = null; wvDirty = true; wvCache = null;
    if (wvVisible) buildWorkspaceViz();
    const mini = document.getElementById('ik-status-mini');
    mini.textContent = 'enter a target (X, Y, Z in mm) and click “solve ik” — the arms move to the pose.';
    mini.className = 'ik-mini';
    refresh();
  });

  // trajectory player
  document.querySelectorAll('.traj-btn').forEach(b => {
    b.addEventListener('click', () => selectTraj(b.dataset.traj));
  });
  document.getElementById('traj-play').addEventListener('click', () => {
    if (!traj.name) { selectTraj('circle'); return; }   // nothing chosen yet → start with a circle
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
  refresh();
}
if (document.readyState === 'loading') window.addEventListener('DOMContentLoaded', boot);
else boot();   // bundled/inlined: DOM is already parsed
