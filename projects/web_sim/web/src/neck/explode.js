// Page 3: exploded view. A standard SO101 arm turns into LeRobotNeck in 7 steps: stock arm -> apart -> mark the two
// parts that change -> swap in the new J3-J4 link (our design) and XLeRobot's soft-gripper jaw -> J4 axis turned 90°
// -> add the camera -> assemble.
//
// The MuJoCo model holds both chains at the same base: the neck (neck_*) and a hidden stock SO101 (stock_*), whose
// J1-J3 coincide with the neck's. Every frame, after View.sync() has copied the assembled poses, postSync() moves each
// part: exploded along its chain, morphed between the stock and the neck layout (parts both have: servos, wrist,
// shoulder, upper arm), faded / flown in or out (parts only one of them has).
import * as THREE from 'three';
import { DEG } from '../sim.js';
import { t } from './i18n.js';
import { HOME_VIEW } from './home.js';

const NECK = ['neck_shoulder', 'neck_upper_arm', 'neck_link34', 'neck_wrist', 'neck_cam_tilt_housing', 'neck_cam_holder'];
const STOCK = ['stock_shoulder', 'stock_upper_arm', 'stock_lower_arm', 'stock_wrist', 'stock_fixed_jaw', 'stock_moving_jaw'];
// neck mesh <- stock mesh with the same part (morphed from the stock to the neck layout)
const PAIRS = [
  ['neck_shoulder/Rotation_Pitch', 'stock_shoulder/Rotation_Pitch'], ['neck_shoulder/Rotation_Pitch_Motor', 'stock_shoulder/Rotation_Pitch_Motor'],
  ['neck_upper_arm/Upper_Arm', 'stock_upper_arm/Upper_Arm'], ['neck_upper_arm/Upper_Arm_Motor', 'stock_upper_arm/Upper_Arm_Motor'],
  ['neck_link34/Lower_Arm_Motor', 'stock_lower_arm/Lower_Arm_Motor'],
  ['neck_wrist/Wrist_Pitch_Roll', 'stock_wrist/Wrist_Pitch_Roll'], ['neck_wrist/Wrist_Pitch_Roll_Motor', 'stock_wrist/Wrist_Pitch_Roll_Motor'],
  ['neck_cam_tilt_housing/Fixed_Jaw_Motor', 'stock_fixed_jaw/Fixed_Jaw_Motor'],
];
const OLD = ['stock_lower_arm/Lower_Arm', 'stock_fixed_jaw/Fixed_Jaw', 'stock_moving_jaw/Moving_Jaw'];
// replacements: the J3-J4 link is the one new design (orange); the J6 housing is XLeRobot's soft-gripper fixed jaw
const NEW = ['neck_link34/wallerobot_new_wrist_dof_0_1', 'neck_cam_tilt_housing/cam_tilt_housing_softfin'];
const FROM_XLE = 'neck_cam_tilt_housing/cam_tilt_housing_softfin';
const CAM = ['neck_cam_holder/tophead5', 'neck_cam_holder/tophead6'];

const COLOR = { print: 0xecebe6, servo: 0x1d1e21, fresh: 0xff6a13, xle: 0x56657c, cam: 0x8d9096, warn: 0xe5484d };
// pose shown on this page (degrees): neck and the stock arm share J1-J3
const POSE = { neck_j1: 0, neck_j2: 90, neck_j3: 32, neck_j4: 0, neck_j5: 0, neck_j6: 0 };
const STOCK_POSE = { stock_shoulder_pan: 0, stock_shoulder_lift: 90, stock_elbow_flex: 32, stock_wrist_flex: 0, stock_wrist_roll: 0, stock_gripper: 0 };
export const NECK_VIEW = { pos: [-0.86, 0.92, 1.12], target: [-0.08, 0.78, 0.0] };   // tall: the (exploded) stock arm
const CLOSE_VIEW = { pos: [-0.62, 0.82, 0.8], target: [-0.07, 0.68, 0.0] };        // the neck (shorter than the arm)
const AXIS_VIEW = { pos: [-0.72, 1.02, 0.32], target: [-0.07, 0.64, 0.0] };       // looks along J3 x J4: both axes in view
const VIEWS = [NECK_VIEW, NECK_VIEW, NECK_VIEW, NECK_VIEW, AXIS_VIEW, CLOSE_VIEW, CLOSE_VIEW];

// e explode 0..1, w stock->neck 0..1, h mark the changing parts, mj moving jaw shown, c camera parts shown,
// ax J3/J4 axes shown, anim 0 none / 1 J4 swing / 2 tilt + nod
const STEPS = [
  { e: 0, w: 0, h: 0, mj: 1, c: 0, ax: 0, anim: 0, labels: [] },
  { e: 1, w: 0, h: 0, mj: 1, c: 0, ax: 0, anim: 0,
    labels: [['neck_shoulder/Rotation_Pitch', 'part.Rotation_Pitch', 'so101'], ['neck_upper_arm/Upper_Arm', 'part.Upper_Arm', 'so101'],
      ['stock_lower_arm/Lower_Arm', 'part.Lower_Arm', 'so101'], ['neck_wrist/Wrist_Pitch_Roll', 'part.Wrist_Pitch_Roll', 'so101'],
      ['stock_fixed_jaw/Fixed_Jaw', 'part.Fixed_Jaw', 'so101'], ['stock_moving_jaw/Moving_Jaw', 'part.Moving_Jaw', 'so101'],
      ['neck_upper_arm/Upper_Arm_Motor', 'part.servo', null]] },
  { e: 1, w: 0, h: 1, mj: 0, c: 0, ax: 0, anim: 0,
    labels: [['stock_lower_arm/Lower_Arm', 'part.Lower_Arm', 'out'], ['stock_fixed_jaw/Fixed_Jaw', 'part.Fixed_Jaw', 'out'],
      ['stock_moving_jaw/Moving_Jaw', 'part.Moving_Jaw', 'out']] },
  { e: 1, w: 1, h: 0, mj: 0, c: 0, ax: 0, anim: 0,
    labels: [['neck_link34/wallerobot_new_wrist_dof_0_1', 'part.wallerobot_new_wrist_dof_0_1', 'new'],
      ['neck_cam_tilt_housing/cam_tilt_housing_softfin', 'part.cam_tilt_housing_softfin', 'swap']] },
  { e: 0.45, w: 1, h: 0, mj: 0, c: 0, ax: 1, anim: 1, labels: [] },
  { e: 1, w: 1, h: 0, mj: 0, c: 1, ax: 0, anim: 0,
    labels: [['neck_cam_holder/tophead5', 'part.tophead5', 'xlerobot'], ['neck_cam_holder/tophead6', 'part.tophead6', 'xlerobot'],
      ['world/base_tower', 'part.base_tower', 'xlerobot']] },
  { e: 0, w: 1, h: 0, mj: 0, c: 1, ax: 0, anim: 2, labels: [] },
];
const TAG = { so101: 'tag.so101', out: 'tag.out', new: 'tag.new', xlerobot: 'tag.xlerobot', swap: 'tag.xlerobot' };

export function neckPage(ctx) {
  const { sim, view, el } = ctx;
  const { mj, m, d } = sim;
  let idx = null, step = 0, auto = null, ui = null, clock = 0, active = false;
  const S = { e: 0, w: 1, h: 0, mj: 0, c: 1, ax: 0, p: 0 };   // current (smoothed) state
  const labelBox = document.querySelector('#labels');
  const arrows = {};

  // ---- lookup tables (built once the View exists)
  function build() {
    if (idx) return;
    const by = {};
    for (const g of view.geoms) by[`${g.userData.bodyName}/${g.userData.meshName}`] = g;
    const bid = (n) => mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY.value, n);
    const chainOf = (key) => {
      const b = key.split('/')[0];
      if (NECK.includes(b)) return ['neck', NECK.indexOf(b)];
      if (STOCK.includes(b)) return ['stock', STOCK.indexOf(b)];
      return [null, -1];
    };
    idx = { by, neck: NECK.map(bid), stock: STOCK.map(bid), chainOf,
      j3: mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT.value, 'neck_j3'), j4: mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT.value, 'neck_j4') };
    for (const [k, c] of [['j3', 0x2a6df4], ['j4', COLOR.fresh]]) {
      const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.2, c, 0.03, 0.018);
      a.line.material.depthTest = false; a.cone.material.depthTest = false; a.renderOrder = 20; a.visible = false;
      view.world.add(a); arrows[k] = a;
    }
  }

  // ---- explode offsets: cumulative along each chain (MuJoCo world coords)
  const v3 = (a, i) => new THREE.Vector3(a[3 * i], a[3 * i + 1], a[3 * i + 2]);
  function chainOffsets(bodies, e) {
    const out = []; const acc = new THREE.Vector3(0, 0, 0.03 * e);
    for (let k = 0; k < bodies.length; k++) {
      if (k > 0) {
        const dir = v3(d.xpos, bodies[k]).sub(v3(d.xpos, bodies[k - 1]));
        if (dir.length() < 1e-4) dir.set(0, 0, 1);
        acc.addScaledVector(dir.normalize(), 0.036 * e);
      }
      out.push(acc.clone());
    }
    return out;
  }
  function servoOffset(mesh, e) {
    if (!/Motor|topbase2/.test(mesh.userData.meshName)) return new THREE.Vector3();
    const j = m.body_jntadr[mesh.userData.body];
    if (j < 0) return new THREE.Vector3();
    return v3(d.xaxis, j).multiplyScalar(0.035 * e);
  }
  const _p = new THREE.Vector3(), _q = new THREE.Quaternion(), _s = new THREE.Vector3();
  const _p2 = new THREE.Vector3(), _q2 = new THREE.Quaternion();
  function placed(mesh, off) {   // assembled matrix (from sync) shifted by `off`
    const M = mesh.matrix.clone(); M.elements[12] += off.x; M.elements[13] += off.y; M.elements[14] += off.z; return M;
  }
  function setOpacity(mesh, a) {
    mesh.material.transparent = a < 0.999; mesh.material.opacity = a; mesh.visible = a > 0.01;
    mesh.material.depthWrite = a > 0.5;
  }
  const paint = (mesh, hex, k) => mesh.material.color.copy(mesh.userData.baseColor).lerp(new THREE.Color(hex).convertSRGBToLinear(), k);
  const roleColor = (key) => {
    const mn = key.split('/')[1];
    if (key === FROM_XLE) return COLOR.xle;
    if (NEW.includes(key)) return COLOR.fresh;
    if (mn === 'tophead6') return COLOR.cam;
    if (key.startsWith('world/') && mn !== 'topbase2') return COLOR.xle;
    if (mn === 'tophead5') return COLOR.xle;
    if (/Motor|topbase2/.test(mn)) return COLOR.servo;
    return COLOR.print;
  };

  function postSync() {
    const { by } = idx; const e = S.e, w = S.w;
    const offN = chainOffsets(idx.neck, e), offS = chainOffsets(idx.stock, e);
    const ofs = (key, mesh) => {
      const [ch, k] = idx.chainOf(key);
      return (ch === 'neck' ? offN[k] : ch === 'stock' ? offS[k] : new THREE.Vector3()).clone().add(servoOffset(mesh, e));
    };
    // hide every stock mesh first; the ones that are drawn (OLD) are turned on below
    for (const g of view.geoms) if (g.userData.bodyName?.startsWith('stock_')) g.visible = false;
    for (const [nk, sk] of PAIRS) {
      const n = by[nk], s = by[sk]; if (!n || !s) continue;
      const A = placed(s, ofs(sk, s)), B = placed(n, ofs(nk, n));
      A.decompose(_p, _q, _s); B.decompose(_p2, _q2, _s);
      _p.lerp(_p2, w); _q.slerp(_q2, w);
      n.matrix.compose(_p, _q, _s); n.matrixWorldNeedsUpdate = true;
      paint(n, roleColor(nk), S.p); setOpacity(n, 1);
    }
    for (const key of OLD) {
      const s = by[key]; if (!s) continue;
      const fly = new THREE.Vector3(0.02, 0.32 * w, 0.06 * w);
      s.matrix.copy(placed(s, ofs(key, s).add(fly))); s.matrixWorldNeedsUpdate = true;
      const jaw = key.endsWith('Moving_Jaw') ? S.mj : 1;
      setOpacity(s, Math.max(0, 1 - 1.6 * w) * jaw);
      paint(s, COLOR.warn, S.h * S.p); s.material.emissive.set(S.h > 0.05 ? COLOR.warn : 0);
      s.material.emissiveIntensity = 0.35 * S.h * (0.6 + 0.4 * Math.sin(clock * 5));
      if (S.h < 0.05) paint(s, COLOR.print, S.p);
    }
    for (const key of NEW) {
      const n = by[key]; if (!n) continue;
      const fly = new THREE.Vector3(0, -0.32 * (1 - w), 0.06 * (1 - w));
      n.matrix.copy(placed(n, ofs(key, n).add(fly))); n.matrixWorldNeedsUpdate = true;
      setOpacity(n, Math.min(1, Math.max(0, 1.6 * w - 0.6))); paint(n, roleColor(key), Math.max(S.p, 0.6));
    }
    for (const key of CAM) {
      const n = by[key]; if (!n) continue;
      const fly = new THREE.Vector3(-0.05 * (1 - S.c), 0, 0.28 * (1 - S.c));
      n.matrix.copy(placed(n, ofs(key, n).add(fly))); n.matrixWorldNeedsUpdate = true;
      setOpacity(n, S.c); paint(n, roleColor(key), S.p);
    }
    for (const g of view.geoms) if (g.userData.bodyName === 'world') paint(g, roleColor(`world/${g.userData.meshName}`), S.p);
    // J3 / J4 axes (neck), through the joint anchors, shifted with their bodies
    for (const [k, jid, ci] of [['j3', idx.j3, 2], ['j4', idx.j4, 3]]) {
      const a = arrows[k]; a.visible = S.ax > 0.05;
      if (!a.visible) continue;
      const ax = v3(d.xaxis, jid), p = v3(d.xanchor, jid).add(offN[ci]).addScaledVector(ax, -0.1);
      a.position.copy(p); a.setDirection(ax); a.scale.setScalar(Math.max(0.01, S.ax));
    }
  }

  // ---- labels (HTML, projected every frame)
  const _c = new THREE.Vector3();
  function drawLabels() {
    const spec = STEPS[step].labels;
    const r = view.renderer.domElement.getBoundingClientRect();
    view.world.updateMatrixWorld();
    const items = spec.map(([key, txt, tag]) => {
      const mesh = idx.by[key]; if (!mesh || !mesh.visible || mesh.material.opacity < 0.3) return null;
      mesh.geometry.boundingSphere || mesh.geometry.computeBoundingSphere();
      _c.copy(mesh.geometry.boundingSphere.center).applyMatrix4(mesh.matrix).applyMatrix4(view.world.matrixWorld).project(view.camera);
      if (_c.z > 1) return null;
      return { x: r.left + (_c.x + 1) / 2 * r.width, y: r.top + (1 - _c.y) / 2 * r.height, txt, tag };
    });
    if (S.ax > 0.5) for (const k of ['j3', 'j4']) {
      const a = arrows[k]; _c.set(0, 0.2, 0).applyMatrix4(a.matrixWorld).project(view.camera);
      items.push({ x: r.left + (_c.x + 1) / 2 * r.width, y: r.top + (1 - _c.y) / 2 * r.height, raw: k === 'j3' ? 'J3' : 'J4 ⟂ J3', tag: k });
    }
    // keep labels from sitting on top of each other: walk them top to bottom, push down by at least 26 px
    const shown = items.filter(Boolean).sort((a, b) => a.y - b.y);
    for (let i = 1; i < shown.length; i++) if (shown[i].y - shown[i - 1].y < 26 && Math.abs(shown[i].x - shown[i - 1].x) < 220) shown[i].y = shown[i - 1].y + 26;
    labelBox.replaceChildren(...shown.map((it) => el('div', {
      class: `lbl ${it.tag ?? ''}`, style: `left:${it.x.toFixed(0)}px;top:${it.y.toFixed(0)}px`,
    }, el('i'), el('span', {}, it.raw ?? t(it.txt), it.tag && TAG[it.tag] ? el('small', {}, t(TAG[it.tag])) : null))));
  }

  // ---- UI
  function render() {
    const s = step + 1;
    ui = el('section', { class: 'neck' },
      el('div', { class: 'story' },
        el('small', {}, `${s} / ${STEPS.length}`),
        el('h2', {}, t(`neck.s${s}.t`)), el('p', {}, t(`neck.s${s}.b`)),
        el('div', { class: 'dots' }, STEPS.map((_, i) => el('button', { class: i === step ? 'on' : '', 'aria-label': `${i + 1}`, onclick: () => goStep(i) }))),
        el('div', { class: 'row' },
          el('button', { class: 'btn', disabled: step === 0, onclick: () => goStep(step - 1) }, t('neck.prev')),
          step < STEPS.length - 1
            ? el('button', { class: 'btn primary', onclick: () => goStep(step + 1) }, t('neck.next'))
            : el('button', { class: 'btn primary', onclick: () => ctx.go('play') }, t('neck.try')),
          el('button', { class: 'btn ghost', onclick: toggleAuto }, auto ? t('neck.stop') : t('neck.auto'))),
        step === STEPS.length - 1 ? el('button', { class: 'btn ghost back', onclick: () => ctx.go('home') }, `← ${t('neck.home')}`) : null));
    document.querySelector('#ui').replaceChildren(ui);
  }
  function goStep(i, { keepAuto = false } = {}) {
    const prevView = VIEWS[step];
    step = Math.max(0, Math.min(STEPS.length - 1, i));
    if (VIEWS[step] !== prevView) ctx.flyTo(VIEWS[step].pos, VIEWS[step].target, 1.4);
    if (!keepAuto && auto) { clearInterval(auto); auto = null; }
    history.replaceState(history.state, '', `${ctx.href('neck')}#step-${step + 1}`);
    render();
  }
  function toggleAuto() {
    if (auto) { clearInterval(auto); auto = null; render(); return; }
    if (step === STEPS.length - 1) goStep(0);
    auto = setInterval(() => { if (step >= STEPS.length - 1) { clearInterval(auto); auto = null; render(); } else goStep(step + 1, { keepAuto: true }); }, 4500);
    render();
  }

  return {
    enter(prev) {
      build(); active = true;
      const h = /#step-(\d+)/.exec(location.hash); step = h ? Math.max(0, Math.min(STEPS.length - 1, +h[1] - 1)) : 0;
      // arriving from the robot: start from "assembled neck" so the neck visibly turns back into a stock SO101
      Object.assign(S, prev ? { e: 0, w: 1, h: 0, mj: 0, c: 1, ax: 0, p: 0 } : { ...STEPS[step], p: 1 });
      sim.speed = 60 * DEG;
      sim.setTargets(Object.fromEntries(Object.entries(POSE).map(([n, v]) => [n, v * DEG])));
      sim.setAux(Object.fromEntries(Object.entries(STOCK_POSE).map(([n, v]) => [n, v * DEG])));
      ctx.flyTo(VIEWS[step].pos, VIEWS[step].target, prev ? 1.8 : 0.01);
      view.postSync = postSync;
      render();
      if (step === 0 && !h) history.replaceState(history.state, '', ctx.href('neck'));
    },
    exit(next) {
      active = false; if (auto) { clearInterval(auto); auto = null; }
      view.postSync = null; labelBox.replaceChildren();
      for (const a of Object.values(arrows)) a.visible = false;
      for (const g of view.geoms) {
        g.material.color.copy(g.userData.baseColor); g.material.emissive.set(0); g.material.emissiveIntensity = 0;
        setOpacity(g, g.userData.baseOpacity ?? 1);
        if (g.userData.bodyName?.startsWith('stock_')) g.visible = false;
      }
      if (next === 'home') ctx.flyTo(HOME_VIEW.pos, HOME_VIEW.target, 1.6);
    },
    update(dt) {
      clock += dt;
      const T = STEPS[step]; const k = 1 - Math.exp(-dt * 3.2);
      for (const key of ['e', 'w', 'h', 'mj', 'c', 'ax']) S[key] += (T[key] - S[key]) * k;
      S.p += (1 - S.p) * k;
      // small motions that show what the step is about
      const base = Object.fromEntries(Object.entries(POSE).map(([n, v]) => [n, v * DEG]));
      if (T.anim === 1) sim.setTargets({ ...base, neck_j4: Math.sin(clock * 1.3) * 35 * DEG });
      else if (T.anim === 2) sim.setTargets({ ...base, neck_j4: Math.sin(clock * 0.9) * 25 * DEG, neck_j6: base.neck_j6 + Math.sin(clock * 0.6) * 20 * DEG, neck_j1: Math.sin(clock * 0.35) * 30 * DEG });
      else sim.setTargets(base);
      // the stock arm follows J1-J3 so the shared parts stay on top of each other
      sim.setAux({ stock_shoulder_pan: sim.cmd.neck_j1, stock_shoulder_lift: sim.cmd.neck_j2, stock_elbow_flex: sim.cmd.neck_j3 });
    },
    afterDraw() { if (active) drawLabels(); },
    lang() { render(); },
  };
}
