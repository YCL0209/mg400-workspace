/**
 * Three.js orthographic top-down scene — M1 draws the static reachable
 * workspace (annulus minus J1 rear dead-zone wedge), a coordinate grid, and
 * X/Y axes. M2 will add the live arm pose + FOV polygon.
 *
 * Coordinate convention (PHASE2 design §6):
 *   World +X = robot forward, +Y = robot left, +Z = up.
 *   Camera looks down -Z, with camera "up" vector aligned to world +X so the
 *   robot's forward direction renders as screen "up" and the robot's left
 *   side renders as screen left. Grid is on the z=0 plane.
 */

import * as THREE from "three";

const DEG = Math.PI / 180;

export function initScene(rootEl) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x0a0a0a);
  rootEl.appendChild(renderer.domElement);

  const scene = new THREE.Scene();

  // OrthographicCamera over the z=0 plane.
  // Camera "up" = world +X → robot forward appears as screen "up".
  const camera = new THREE.OrthographicCamera();
  camera.position.set(0, 0, 1000);
  camera.up.set(1, 0, 0);
  camera.lookAt(0, 0, 0);

  const sizing = { viewHalfMm: 520 }; // covers outer 440 mm with margin

  const resize = () => {
    const w = rootEl.clientWidth;
    const h = rootEl.clientHeight;
    renderer.setSize(w, h, false);
    const aspect = w / h;
    const v = sizing.viewHalfMm;
    // Camera up = world +X, so the camera's "vertical" axis (top - bottom) is
    // world +X. Then "horizontal" axis spans world +Y. Frustum height bounds
    // world +X span; width bounds world +Y span by aspect.
    camera.top = v;
    camera.bottom = -v;
    camera.right = v * aspect;
    camera.left = -v * aspect;
    camera.near = -1000;
    camera.far = 2000;
    camera.updateProjectionMatrix();
  };
  resize();
  window.addEventListener("resize", resize);

  // Group containers — applyWorkspace clears these on each new workspace msg.
  const workspaceGroup = new THREE.Group();
  scene.add(workspaceGroup);

  const render = () => renderer.render(scene, camera);
  render();
  // Re-render on any visibility change; M1 has no animation loop.
  const observer = new ResizeObserver(() => { resize(); render(); });
  observer.observe(rootEl);

  return { scene, camera, renderer, workspaceGroup, render };
}

export function applyWorkspace(ctx, msg) {
  const g = ctx.workspaceGroup;
  // Clear previous workspace primitives.
  while (g.children.length) g.remove(g.children[0]);

  // 1) Reachable annulus (outer arc minus inner arc) with J1 dead-zone wedge cut out.
  const [j1Min, j1Max] = msg.j1_range_deg;
  const inner = msg.annulus_inner_mm;
  const outer = msg.annulus_outer_mm;
  const annulusMesh = buildAnnulusSector(inner, outer, j1Min * DEG, j1Max * DEG);
  g.add(annulusMesh);

  // 2) Coordinate grid + axes.
  g.add(buildGrid(msg.grid_step_mm, outer + 80));
  g.add(buildAxes(outer + 60));

  // 3) Origin dot (base / J1 axis).
  const originDot = new THREE.Mesh(
    new THREE.CircleGeometry(6, 24),
    new THREE.MeshBasicMaterial({ color: 0xffffff }),
  );
  originDot.position.set(0, 0, 0.5);
  g.add(originDot);

  ctx.render();
}

/**
 * Reachable annular sector from J1 angle ``a0`` to ``a1`` (radians, CCW).
 * Filled region between ``innerR`` and ``outerR``.
 */
function buildAnnulusSector(innerR, outerR, a0, a1) {
  const shape = new THREE.Shape();
  // Outer arc CCW from a0 to a1.
  shape.absarc(0, 0, outerR, a0, a1, false);
  // Line in to inner radius at a1.
  shape.lineTo(innerR * Math.cos(a1), innerR * Math.sin(a1));
  // Inner arc CW back from a1 to a0.
  shape.absarc(0, 0, innerR, a1, a0, true);
  shape.lineTo(outerR * Math.cos(a0), outerR * Math.sin(a0));

  const geo = new THREE.ShapeGeometry(shape, 96);
  const mat = new THREE.MeshBasicMaterial({
    color: 0x1f7a4f,
    transparent: true,
    opacity: 0.18,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.z = 0;

  // Outline pass — same shape, drawn as a closed line so the boundary is
  // crisper than the filled mesh alone.
  const points = shape.getPoints(192);
  const outlineGeo = new THREE.BufferGeometry().setFromPoints(
    points.map((p) => new THREE.Vector3(p.x, p.y, 0.1)),
  );
  const outline = new THREE.LineLoop(
    outlineGeo,
    new THREE.LineBasicMaterial({ color: 0x4fd99b }),
  );

  const group = new THREE.Group();
  group.add(mesh);
  group.add(outline);
  return group;
}

function buildGrid(stepMm, halfSpanMm) {
  const group = new THREE.Group();
  const mat = new THREE.LineBasicMaterial({ color: 0x222222 });
  const halfSpan = Math.ceil(halfSpanMm / stepMm) * stepMm;

  for (let v = -halfSpan; v <= halfSpan; v += stepMm) {
    // Lines parallel to world Y (at constant world X = v).
    const pointsX = [
      new THREE.Vector3(v, -halfSpan, -0.1),
      new THREE.Vector3(v, halfSpan, -0.1),
    ];
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pointsX), mat));
    // Lines parallel to world X (at constant world Y = v).
    const pointsY = [
      new THREE.Vector3(-halfSpan, v, -0.1),
      new THREE.Vector3(halfSpan, v, -0.1),
    ];
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pointsY), mat));
  }
  return group;
}

function buildAxes(lengthMm) {
  const group = new THREE.Group();
  // +X axis — robot forward (red, draws toward screen "up" since camera.up = +X).
  group.add(buildLine([0, 0, 0.2], [lengthMm, 0, 0.2], 0xff5555));
  // +Y axis — robot left (green, draws toward screen "left").
  group.add(buildLine([0, 0, 0.2], [0, lengthMm, 0.2], 0x55ff55));
  return group;
}

function buildLine(a, b, color) {
  const geo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(...a),
    new THREE.Vector3(...b),
  ]);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
}
