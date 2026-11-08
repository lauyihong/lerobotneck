// three.js view of the MuJoCo model: one THREE.Mesh per visual geom, poses copied from MjData every frame.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, CYLINDER: 5, BOX: 6, MESH: 7 };
const VISUAL_GROUPS = new Set([0, 1, 2]);   // 3 = collision hulls (hidden), 4 = sites

export class View {
  constructor(sim, canvas) {
    this.sim = sim;
    const r = this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFShadowMap;
    r.outputColorSpace = THREE.SRGBColorSpace;
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0xdfe3ea);
    scene.fog = new THREE.Fog(0xdfe3ea, 2.5, 7.5);

    // MuJoCo is z-up; everything robot-related lives in `world`, rotated so +z points up on screen
    this.world = new THREE.Group(); this.world.rotation.x = -Math.PI / 2; scene.add(this.world);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(-1.2, 2.0, 1.0); sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.1, far: 6 });
    sun.shadow.bias = -0.0005;
    scene.add(sun);

    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 30);
    this.camera.position.set(-1.25, 0.65, 0.45);   // in front of the robot (it faces world −x), a little to its left
    this.orbit = new OrbitControls(this.camera, canvas);
    this.orbit.target.set(-0.12, 0.22, 0); this.orbit.enableDamping = true; this.orbit.update();

    this.geoms = []; this.pickables = [];
    this.buildGeoms();

    // the neck D435i colour camera (picture-in-picture). MuJoCo cameras look along −z with +y up, same as three.js.
    const m = sim.m;
    this.camId = sim.mj.mj_name2id(m, sim.mj.mjtObj.mjOBJ_CAMERA.value, 'neck_color');
    this.neckCam = new THREE.PerspectiveCamera(m.cam_fovy[this.camId], 4 / 3, 0.01, 10);
    this.neckCam.matrixAutoUpdate = false;
    this.world.add(this.neckCam);
    this.pip = { on: true, w: 240 };

    this.markers = {};
  }

  buildGeoms() {
    const { m } = this.sim;
    const cache = new Map();
    for (let g = 0; g < m.ngeom; g++) {
      const type = m.geom_type[g];
      if (!VISUAL_GROUPS.has(m.geom_group[g])) continue;
      const rgba = [0, 1, 2, 3].map((k) => m.geom_rgba[4 * g + k]);
      let geo;
      if (type === GEOM.MESH) {
        const id = m.geom_dataid[g];
        if (!cache.has(id)) cache.set(id, meshGeometry(m, id));
        geo = cache.get(id);
      } else if (type === GEOM.PLANE) {
        this.addFloor(g); continue;
      } else if (type === GEOM.SPHERE) geo = new THREE.SphereGeometry(m.geom_size[3 * g], 24, 16);
      else if (type === GEOM.BOX) geo = new THREE.BoxGeometry(2 * m.geom_size[3 * g], 2 * m.geom_size[3 * g + 1], 2 * m.geom_size[3 * g + 2]);
      else if (type === GEOM.CYLINDER) { geo = new THREE.CylinderGeometry(m.geom_size[3 * g], m.geom_size[3 * g], 2 * m.geom_size[3 * g + 1], 24); geo.rotateX(Math.PI / 2); }
      else if (type === GEOM.CAPSULE) { geo = new THREE.CapsuleGeometry(m.geom_size[3 * g], 2 * m.geom_size[3 * g + 1], 6, 16); geo.rotateX(Math.PI / 2); }
      else continue;
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace),
        roughness: 0.62, metalness: 0.05, transparent: rgba[3] < 1, opacity: rgba[3],
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = true; mesh.receiveShadow = true;
      mesh.matrixAutoUpdate = false;
      mesh.userData = { geom: g, body: m.geom_bodyid[g], baseColor: mat.color.clone() };
      this.world.add(mesh);
      this.geoms.push(mesh); this.pickables.push(mesh);
    }
  }

  addFloor(g) {
    const { m } = this.sim;
    const c = document.createElement('canvas'); c.width = c.height = 256;
    const x = c.getContext('2d');
    for (let i = 0; i < 8; i++) for (let k = 0; k < 8; k++) { x.fillStyle = (i + k) % 2 ? '#c9c7c1' : '#d6d4ce'; x.fillRect(i * 32, k * 32, 32, 32); }
    const tex = new THREE.CanvasTexture(c); tex.wrapS = tex.wrapT = THREE.RepeatWrapping; tex.repeat.set(12, 12);
    tex.colorSpace = THREE.SRGBColorSpace;
    // drawn much larger than the physics plane so the neck camera sees a horizon instead of the void
    const s = Math.max(8, m.geom_size[3 * g] || 0);
    tex.repeat.set(s / 0.6, s / 0.6);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(2 * s, 2 * s), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.9 }));
    floor.position.set(m.geom_pos[3 * g], m.geom_pos[3 * g + 1], m.geom_pos[3 * g + 2]);
    floor.receiveShadow = true;
    this.world.add(floor);
  }

  marker(name, color, radius = 0.012) {
    if (!this.markers[name]) {
      const mk = new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 14),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.55, depthTest: false }));
      mk.renderOrder = 10; this.world.add(mk); this.markers[name] = mk;
    }
    return this.markers[name];
  }

  highlightBody(body) {
    for (const mesh of this.geoms) {
      const on = body != null && mesh.userData.body === body;
      mesh.material.emissive?.set(on ? 0x2a6df4 : 0x000000);
      mesh.material.emissiveIntensity = on ? 0.45 : 0;
    }
  }

  sync() {
    const { d } = this.sim;
    const xp = d.geom_xpos, xm = d.geom_xmat;
    const M = new THREE.Matrix4();
    for (const mesh of this.geoms) {
      const g = mesh.userData.geom, a = 9 * g, p = 3 * g;
      M.set(xm[a], xm[a + 1], xm[a + 2], xp[p],
        xm[a + 3], xm[a + 4], xm[a + 5], xp[p + 1],
        xm[a + 6], xm[a + 7], xm[a + 8], xp[p + 2],
        0, 0, 0, 1);
      mesh.matrix.copy(M); mesh.matrixWorldNeedsUpdate = true;
    }
    const c = this.camId, cp = d.cam_xpos, cm = d.cam_xmat;
    this.neckCam.matrix.set(cm[9 * c], cm[9 * c + 1], cm[9 * c + 2], cp[3 * c],
      cm[9 * c + 3], cm[9 * c + 4], cm[9 * c + 5], cp[3 * c + 1],
      cm[9 * c + 6], cm[9 * c + 7], cm[9 * c + 8], cp[3 * c + 2], 0, 0, 0, 1);
    this.neckCam.matrixWorldNeedsUpdate = true;
  }

  resize() {
    const cv = this.renderer.domElement; const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.floor(w * this.renderer.getPixelRatio()) || cv.height !== Math.floor(h * this.renderer.getPixelRatio())) {
      this.renderer.setSize(w, h, false);
    }
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    this.pip.w = Math.round(Math.min(280, Math.max(110, w * 0.24)));
  }

  draw(extraHelpers = []) {
    this.resize(); this.orbit.update(); this.sync();
    const r = this.renderer; const cv = r.domElement; const w = cv.clientWidth, h = cv.clientHeight;
    r.setScissorTest(false); r.setViewport(0, 0, w, h); r.render(this.scene, this.camera);
    if (this.pip.on) {
      // neck camera view, bottom-left corner; helpers (gizmo, markers) are hidden in it
      const pw = this.pip.w, ph = Math.round(pw * 3 / 4), x = 10, y = 10;
      const hidden = [...extraHelpers, ...Object.values(this.markers)].filter((o) => o.visible);
      hidden.forEach((o) => { o.visible = false; });
      r.setScissorTest(true); r.setScissor(x, y, pw, ph); r.setViewport(x, y, pw, ph);
      r.render(this.scene, this.neckCam);
      r.setScissorTest(false);
      hidden.forEach((o) => { o.visible = true; });
    }
  }

  pipRect() { const pw = this.pip.w, ph = Math.round(pw * 3 / 4); return { left: 10, bottom: 10, width: pw, height: ph }; }
}

function meshGeometry(m, id) {
  const va = m.mesh_vertadr[id], vn = m.mesh_vertnum[id], fa = m.mesh_faceadr[id], fn = m.mesh_facenum[id];
  const V = m.mesh_vert, F = m.mesh_face;
  const pos = new Float32Array(fn * 9);
  for (let f = 0; f < fn; f++) for (let k = 0; k < 3; k++) {
    const v = F[3 * (fa + f) + k]; if (v >= vn) continue;
    pos.set([V[3 * (va + v)], V[3 * (va + v) + 1], V[3 * (va + v) + 2]], 9 * f + 3 * k);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.computeVertexNormals();   // non-indexed → flat facets, which suits the printed parts
  geo.computeBoundingSphere();
  return geo;
}
