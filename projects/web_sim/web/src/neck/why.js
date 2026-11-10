// Page 4: why active perception. Same desk scene, same D435i camera model, six ways to hold the camera (fixed,
// XLeRobot 2-axis head, stock SO101 + wrist camera, SO101-X = the camera holder on the stock gripper, LeRobotNeck,
// LeRobotNeck-02 = a longer J3-J4 link). Coloured dots = surface points the camera sees from at least one pose it can
// reach, grey = never seen. All numbers are precomputed by scripts/make_coverage.py (why/coverage.json); this page only
// shows them and moves the hardware through each setup's best views. The two variants have no bodies of their own in
// the LeRobotNeck model: SO101-X draws the neck's holder + D435i on the stock moving jaw (same frame), LeRobotNeck-02
// shifts everything after J3 along the link and draws a spacer.
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { t, tr } from './i18n.js';
import { GITHUB } from './app.js';

export const WHY_VIEW = { pos: [-1.12, 1.02, 0.78], target: [-0.26, 0.26, 0.0] };
const ORDER = ['fixed', 'head2', 'so101', 'so101x', 'neck', 'neck02'];
const NECKS = ['neck', 'neck02'], STOCKS = ['so101', 'so101x'];
const REGIONS = ['all', 'desk', 'box_inside', 'box_outside', 'block', 'sticker', 'objects'];
const RATE = 75 * Math.PI / 180;   // rad/s, hardware moves between views
const HOLD = 0.9;                  // s at each view

function decode(s, T) {
  const b = atob(s); const u = new Uint8Array(b.length);
  for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i);
  return new T(u.buffer);
}
const quat = (wxyz) => new THREE.Quaternion(wxyz[1], wxyz[2], wxyz[3], wxyz[0]);

function dotTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 32;
  const x = c.getContext('2d'); x.fillStyle = '#fff'; x.beginPath(); x.arc(16, 16, 14, 0, 2 * Math.PI); x.fill();
  return new THREE.CanvasTexture(c);
}

export function whyPage(ctx) {
  const { sim, view, el } = ctx;
  let data = null, S = {}, ui = null, active = false, sel = 'fixed', upright = false, open = false;
  let sweep = null, reveal = Infinity, label = '';
  const cur = {}; let goal = null;
  const root = new THREE.Group(); root.visible = false; view.world.add(root);
  let seenPts = null, blindPts = null;
  const head = { root: new THREE.Group(), panJ: new THREE.Group(), tiltJ: new THREE.Group(), axis: {} };
  root.add(head.root);
  const camOff = { head: new THREE.Matrix4(), so101: new THREE.Matrix4() };
  const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x16a34a }));
  frustum.matrixAutoUpdate = false; root.add(frustum);
  const camM = new THREE.Matrix4();
  const spacer = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshStandardMaterial({ color: 0x0891b2, roughness: 0.55, transparent: true, opacity: 0.85 }));
  spacer.matrixAutoUpdate = false; spacer.visible = false; spacer.castShadow = true; root.add(spacer);
  const spacerLocal = new THREE.Matrix4();
  const tmp = { a: new THREE.Matrix4(), b: new THREE.Matrix4(), v: new THREE.Vector3() };

  fetch(new URL('why/coverage.json', ctx.ROOT)).then((r) => r.json()).then((d) => { data = d; setup(); if (active) { build(); select(sel); } })
    .catch((e) => console.warn('coverage', e));

  function setup() {
    for (const s of data.setups) {
      S[s.id] = { ...s, first: decode(s.first, Int8Array), up: decode(s.upright, Uint8Array) };
    }
    const n = data.points.n, P = decode(data.points.pos_mm, Int16Array), N = decode(data.points.normal, Int8Array);
    const pos = new Float32Array(3 * n);
    for (let i = 0; i < 3 * n; i++) pos[i] = P[i] / 1000 + (N[i] / 127) * 0.002;   // 2 mm off the surface
    const attr = new THREE.BufferAttribute(pos, 3), tex = dotTexture();
    const mk = (color, size, opacity) => {
      const g = new THREE.BufferGeometry(); g.setAttribute('position', attr); g.setIndex([]);
      const p = new THREE.Points(g, new THREE.PointsMaterial({ color, size, map: tex, alphaTest: 0.5, transparent: opacity < 1, opacity }));
      root.add(p); return p;
    };
    blindPts = mk(0x6b7280, 0.0036, 0.4);
    seenPts = mk(0x16a34a, 0.0068, 1);
    // XLeRobot head: pan about z, tilt about the tilt link y (frames from the upstream URDF), meshes from why/
    const h = data.head;
    const panG = new THREE.Group(); panG.position.set(...h.pan.pos); panG.quaternion.copy(quat(h.pan.quat));
    const tiltG = new THREE.Group(); tiltG.position.set(...h.tilt.pos); tiltG.quaternion.copy(quat(h.tilt.quat));
    head.axis = { pan: new THREE.Vector3(...h.pan.axis), tilt: new THREE.Vector3(...h.tilt.axis) };
    head.root.add(panG); panG.add(head.panJ); head.panJ.add(tiltG); tiltG.add(head.tiltJ);
    const loader = new STLLoader();
    for (const m of h.meshes) {
      loader.load(new URL(`why/${m.file}`, ctx.ROOT).href, (geo) => {
        geo.computeVertexNormals();
        const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: m.file === 'tophead6.stl' ? 0x8d9096 : 0x44474e, roughness: 0.5 }));
        mesh.position.set(...m.pos); mesh.quaternion.copy(quat(m.quat)); mesh.scale.setScalar(m.scale);
        mesh.castShadow = mesh.receiveShadow = true;
        (m.link === 'pan' ? head.panJ : head.tiltJ).add(mesh);
      });
    }
    // camera mounts: head = head_camera_link looking along +x, image up +z; SO101 = the XLeRobot wrist-camera pose
    const Rh = new THREE.Matrix4().makeBasis(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0));
    camOff.head.copy(Rh).setPosition(...h.camera_link_pos);
    camOff.so101.compose(new THREE.Vector3(...data.so101_cam.pos), quat(data.so101_cam.quat), new THREE.Vector3(1, 1, 1));
    // frustum drawn 0.16 m deep
    const c = data.camera, L = 0.16, ty = Math.tan((c.vfov * Math.PI / 180) / 2) * L, tx = ty * c.aspect;
    const k = [[tx, ty], [-tx, ty], [-tx, -ty], [tx, -ty]].map(([x, y]) => [x, y, -L]);
    const v = []; for (let i = 0; i < 4; i++) v.push(0, 0, 0, ...k[i], ...k[i], ...k[(i + 1) % 4]);
    frustum.geometry.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
    // LeRobotNeck-02 spacer: a box in the J3-J4 link frame (concept part, no CAD yet)
    const sp = data.neck02.spacer;
    spacerLocal.compose(new THREE.Vector3(...sp.pos), new THREE.Quaternion(), new THREE.Vector3(...sp.half.map((x) => 2 * x)));
  }
  // world matrix of a MuJoCo body
  function bodyM(name, out) {
    const d = sim.d, b = sim.bodyId(name), R = d.xmat, p = d.xpos;
    return out.set(R[9 * b], R[9 * b + 1], R[9 * b + 2], p[3 * b], R[9 * b + 3], R[9 * b + 4], R[9 * b + 5], p[3 * b + 1],
      R[9 * b + 6], R[9 * b + 7], R[9 * b + 8], p[3 * b + 2], 0, 0, 0, 1);
  }

  // ---------- state
  const sb = (id) => S[id];
  function mask(s) {
    const n = data.points.n, m = new Uint8Array(n);
    if (upright) { for (let i = 0; i < n; i++) m[i] = (s.up[i >> 3] >> (7 - (i & 7))) & 1; }
    else for (let i = 0; i < n; i++) m[i] = s.first[i] >= 0 && s.first[i] <= reveal ? 1 : 0;
    return m;
  }
  function paint() {
    if (!seenPts || !S[sel]) return;
    const m = mask(S[sel]), a = [], b = [];
    for (let i = 0; i < m.length; i++) (m[i] ? a : b).push(i);
    seenPts.geometry.setIndex(a); blindPts.geometry.setIndex(b);
    seenPts.material.color.set(S[sel].color); frustum.material.color.set(S[sel].color);
  }
  function showHardware(id) {
    const holder = data?.so101x.holder, hide = data?.so101x.hide;
    for (const g of view.geoms) {
      const b = g.userData.bodyName ?? '';
      if (b.startsWith('neck_')) g.visible = NECKS.includes(id) || (id === 'so101x' && b === holder);
      else if (b.startsWith('stock_')) {
        g.visible = STOCKS.includes(id) && !(id === 'so101x' && g.userData.meshName === hide);
        if (g.visible) { g.material.opacity = 1; g.material.transparent = false; }
      }
    }
    head.root.visible = id === 'fixed' || id === 'head2';
    spacer.visible = id === 'neck02';
  }
  function applyPose(q) {
    const neck = {}, aux = {};
    for (const [n, v] of Object.entries(q)) {
      if (n === 'head_pan') head.panJ.quaternion.setFromAxisAngle(head.axis.pan, v);
      else if (n === 'head_tilt') head.tiltJ.quaternion.setFromAxisAngle(head.axis.tilt, v);
      else if (n.startsWith('stock_')) aux[n] = v; else neck[n] = v;
    }
    if (Object.keys(neck).length) sim.setCommand(neck);
    if (Object.keys(aux).length) sim.setAux(aux);
  }
  function select(id) {
    sel = id; showHardware(id);
    const s = S[id]; if (!s) return;
    for (const k of Object.keys(cur)) delete cur[k];
    Object.assign(cur, s.views[0].q); applyPose(cur);
    startSweep();
    build();
  }
  function startSweep() {
    const s = S[sel]; if (!s) return;
    if (upright) { sweep = null; reveal = Infinity; paint(); setLabel(); return; }
    reveal = -1; sweep = { k: 0, phase: 'move', t: 0 }; goal = s.views[0].q; paint(); setLabel();
  }
  function showAll() { sweep = null; reveal = Infinity; paint(); setLabel(); }
  function setLabel() {
    const s = S[sel]; if (!s) return;
    const one = s.views.length <= 1;
    label = one ? t('why.onepose') : sweep ? t('why.view', { k: sweep.k + 1, n: s.views.length })
      : t(upright ? 'why.allup' : 'why.allposes', { n: (upright ? s.upright_poses : s.poses).toLocaleString() });
    const nowEl = ui?.querySelector('#why-now'); if (nowEl) nowEl.textContent = label;
    const pb = ui?.querySelector('#why-play');
    if (pb) { pb.hidden = one; pb.textContent = sweep ? `❚❚ ${t('why.stop')}` : `▶ ${t('why.sweep', { n: s.views.length })}`; }
  }

  // ---------- UI
  function fmtFindings() {
    const st = (id, r = 'all') => (upright ? S[id].stats_upright : S[id].stats)[r];
    const v = { n: data.tasks.n, ext: (100 * data.neck02.extend[2]).toFixed(1) };
    for (const id of ORDER) {
      Object.assign(v, { [id]: st(id), [`${id}Box`]: st(id, 'box_inside'), [`${id}Sticker`]: st(id, 'sticker'),
        [`${id}Reach`]: S[id].tasks.reached, [`${id}Level`]: S[id].tasks.level, [`${id}Own`]: S[id].own_img, [`${id}Own25`]: S[id].own_img25 });
    }
    return ['f1', 'f2', 'f3', 'f4', 'f5', 'f6'].map((k) => el('p', {}, t(`why.${k}`, v)));
  }
  function build() {
    if (!active) return;
    if (!data) { document.querySelector('#ui').replaceChildren(el('section', { class: 'why' })); return; }
    const stats = (id) => (upright ? S[id].stats_upright : S[id].stats);
    const setups = el('div', { class: 'setups' }, ORDER.map((id) => {
      const s = S[id];
      return el('button', { class: `setup${sel === id ? ' on' : ''}`, style: `--c:${s.color}`, onclick: () => select(id) },
        el('i', {}), el('b', {}, tr(s.name)), el('em', {}, `${stats(id).all}%`),
        el('span', {}, `${t('why.dof', { n: s.dof })} · ${tr(s.short)}`));
    }));
    const bars = el('div', { class: 'bars' }, ORDER.map((id) => el('div', { class: 'barrow' },
      el('span', {}, tr(S[id].name)),
      el('div', { class: 'track' }, el('div', { style: `width:${stats(id).all}%;background:${S[id].color}` })),
      el('b', {}, `${stats(id).all}%`))));
    const table = el('table', { class: 'cov' },
      el('thead', {}, el('tr', {}, el('th', {}, t('why.region')), ORDER.map((id) => el('th', { style: `color:${S[id].color}` }, t(`why.col.${id}`))))),
      el('tbody', {}, REGIONS.map((r) => el('tr', { class: r === 'all' ? 'tot' : '' }, el('td', {}, t(`why.r.${r}`)),
        ORDER.map((id) => { const p = stats(id)[r]; return el('td', { style: `--p:${p / 100};--c:${S[id].color}`, class: id === sel ? 'sel' : '' }, `${Math.round(p)}%`); })))),
      // moving the camera to a requested view, and how much of the image the robot itself covers
      el('tbody', { class: 'tasks' }, el('tr', { class: 'gap' }, el('td', { colspan: ORDER.length + 1 })), [
        ['reached', (s) => s.tasks.reached / s.tasks.n, (s) => `${s.tasks.reached}`],
        ['level', (s) => s.tasks.level / s.tasks.n, (s) => `${s.tasks.level}`],
        ['own', () => 0, (s) => `${Math.round(s.own_img)}%`],
      ].map(([k, p, txt]) => el('tr', {}, el('td', {}, t(`why.t.${k}`, { n: data.tasks.n })),
        ORDER.map((id) => el('td', { style: `--p:${p(S[id])};--c:${S[id].color}`, class: id === sel ? 'sel' : '' }, txt(S[id])))))));
    const meth = el('details', { class: 'meth' }, el('summary', {}, t('why.how')),
      el('ul', {}, ['m1', 'm2', 'm3', 'm5', 'm4'].map((k) => el('li', {}, t(`why.${k}`, {
        n: data.points.n.toLocaleString(), head: S.head2.poses.toLocaleString(), arm: S.neck.poses.toLocaleString(), t: data.tasks.n,
      })))),el('p', { class: 'small muted' }, el('a', { href: `${GITHUB}/blob/main/projects/web_sim/scripts/make_coverage.py`, rel: 'noopener' }, 'make_coverage.py')));
    ui = el('section', { class: 'why' },
      el('div', { class: 'pip-frame', id: 'why-pipf' }, el('span', {}, t('why.cam', { s: tr(S[sel].name) }))),
      el('div', { class: 'legend' }, el('i', { style: `background:${S[sel].color}` }), t('why.legend.seen'), el('i', { class: 'grey' }), t('why.legend.blind')),
      el('aside', { class: `panel${open ? ' open' : ''}` },
        el('div', { class: 'head' }, el('h2', {}, t('why.title')),
          el('button', { class: 'btn ghost sheet-toggle', onclick: () => { open = !open; build(); } }, open ? `▾ ${t('play.less')}` : `▴ ${t('why.more')}`)),
        el('p', { class: 'muted sub' }, t('why.sub')),
        setups,
        el('div', { class: 'transport' },
          el('button', { class: 'btn primary', id: 'why-play', disabled: upright, onclick: () => (sweep ? showAll() : startSweep()) }, ''),
          el('div', { class: 'now', id: 'why-now' }, label)),
        el('label', { class: 'toggle' }, el('input', { type: 'checkbox', checked: upright, onchange: (e) => { upright = e.target.checked; startSweep(); build(); } }), t('why.upright')),
        el('div', { class: 'more' }, el('h3', {}, t('why.compare')), bars, table, el('div', { class: 'find' }, fmtFindings()), meth),
        el('p', { class: 'small muted foot' }, t('why.foot'))));
    document.querySelector('#ui').replaceChildren(ui);
    setLabel(); placePip();
    ui.querySelector('.setup.on')?.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  }
  function placePip() {
    const r = view.pipRect(); const f = ui?.querySelector('#why-pipf');
    if (f) Object.assign(f.style, { left: `${r.left - 2}px`, bottom: `${r.bottom - 2}px`, width: `${r.width + 4}px`, height: `${r.height + 4}px` });
  }

  // camera pose of the selected setup (world = MuJoCo frame), also used for the inset
  function postSync() {
    if (!data || !S[sel]) return;
    if (sel === 'neck') camM.copy(view.neckCam.matrix);
    else if (sel === 'neck02') {
      // longer J3-J4 link: J4 and everything after it (and the J4 motor) move along the link by `extend`
      const L = bodyM(data.neck02.link, tmp.a), moved = data.neck02.moved;
      const w = tmp.v.set(...data.neck02.extend).applyMatrix3(new THREE.Matrix3().setFromMatrix4(L));
      tmp.b.makeTranslation(w.x, w.y, w.z);
      for (const g of view.geoms) {
        const b = g.userData.bodyName;
        if (moved.includes(b) || (b === data.neck02.link && g.userData.meshName === data.neck02.motor)) g.matrix.premultiply(tmp.b);
      }
      camM.copy(view.neckCam.matrix).premultiply(tmp.b);
      spacer.matrix.copy(L).multiply(spacerLocal); spacer.matrixWorldNeedsUpdate = true;
    } else if (sel === 'so101x') {
      // the neck's holder + D435i, moved from the neck holder frame onto the stock moving jaw (same joint frame)
      const T = bodyM(data.so101x.jaw, tmp.a).multiply(bodyM(data.so101x.holder, tmp.b).invert());
      for (const g of view.geoms) if (g.userData.bodyName === data.so101x.holder) g.matrix.premultiply(T);
      camM.copy(view.neckCam.matrix).premultiply(T);
    } else if (sel === 'so101') {
      camM.copy(bodyM('stock_fixed_jaw', tmp.a)).multiply(camOff.so101);
    } else {
      head.root.updateMatrixWorld(true);
      const inv = new THREE.Matrix4().copy(root.matrixWorld).invert();
      camM.copy(inv).multiply(head.tiltJ.matrixWorld).multiply(camOff.head);
    }
    frustum.matrix.copy(camM); frustum.matrixWorldNeedsUpdate = true;
    view.neckCam.matrix.copy(camM); view.neckCam.matrixWorldNeedsUpdate = true;
  }

  return {
    enter() {
      active = true; root.visible = true; view.pip.on = true; view.pip.scale = 1.15; view.postSync = postSync;
      ctx.flyTo(WHY_VIEW.pos, WHY_VIEW.target, 1.4);
      if (data) select(sel); else build();
    },
    exit() {
      active = false; root.visible = false; sweep = null; open = false; view.pip.on = false; view.pip.scale = 1; view.pip.top = null; view.postSync = null;
      for (const g of view.geoms) { const b = g.userData.bodyName ?? ''; if (b.startsWith('neck_')) g.visible = true; else if (b.startsWith('stock_')) g.visible = false; }
      sim.setPose(sim.home);
    },
    update(dt) {
      view.pip.top = innerWidth <= 760 ? 54 : 62;
      if (!data || !S[sel]) return;
      if (sweep) {
        if (sweep.phase === 'move') {
          let done = true;
          for (const [n, g] of Object.entries(goal)) {
            const e = g - (cur[n] ?? g), s = Math.max(-RATE * dt, Math.min(RATE * dt, e));
            cur[n] = (cur[n] ?? g) + s; if (Math.abs(e) > 1e-3) done = false;
          }
          applyPose(cur);
          if (done) { reveal = sweep.k; paint(); sweep.phase = 'hold'; sweep.t = 0; }
        } else if ((sweep.t += dt) > HOLD) {
          const s = S[sel];
          if (sweep.k + 1 < s.views.length) { sweep.k += 1; sweep.phase = 'move'; goal = s.views[sweep.k].q; setLabel(); }
          else showAll();
        }
      } else applyPose(cur);
      placePip();
    },
    lang() { build(); },
    get helpers() { return [frustum]; },
  };
}
