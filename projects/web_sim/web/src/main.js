// Wallerobot web sim — entry point: wires the MuJoCo sim, the three.js view, keyboard / mouse / route control and the panel.
// Pure simulation in the browser: this app never opens a serial port or talks to the robot, ROS or any server.
import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { Sim, ARMS, DEG, worldToBase, baseToWorld } from './sim.js';
import { View } from './render.js';
import { RoutePlayer, Recorder } from './routes.js';

// site root relative to this page (sim/index.html sets data-root="../"; the Artifact build has no data-root)
const ROOT = document.documentElement.dataset.root ?? './';

const $ = (s) => document.querySelector(s);
const el = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };

const LABEL = {
  shoulder_pan: '肩部水平', shoulder_lift: '抬臂', elbow_flex: '肘', wrist_flex: '腕俯仰', wrist_roll: '腕旋转', gripper: '夹爪',
  neck_j1: 'J1 转头', neck_j2: 'J2 抬脖', neck_j3: 'J3 肘', neck_j4: 'J4 侧摆', neck_j5: 'J5 相机滚转', neck_j6: 'J6 相机俯仰',
};
const label = (n) => LABEL[n] ?? LABEL[n.replace(/^(left|right)_arm_/, '')] ?? n;
const ARM_NAME = { left: '左臂', right: '右臂' };
const ARM_COLOR = { left: 0xe8711a, right: 0x1a9e6b };
const GRIP_OPEN = 45 * DEG;
const DESK_Z = -0.015;              // world z of the lowest IK goal (the desk plane is at −0.034)

// "ready": neck standing, arms raised in front — a comfortable start for keyboard / mouse control (degrees)
const READY = { neck_j2: 90, neck_j3: 150, neck_j4: 0, neck_j5: 0, neck_j6: -25 };
for (const a of ['left', 'right']) Object.assign(READY, { [`${a}_arm_shoulder_pan`]: 0, [`${a}_arm_shoulder_lift`]: 100, [`${a}_arm_elbow_flex`]: 90, [`${a}_arm_wrist_flex`]: 0 });

const EE_SPEED = 0.10;              // m/s for keyboard / jog-pad moves
const PITCH_SPEED = 60 * DEG;       // rad/s
const JOINT_SPEED = 45 * DEG;       // rad/s for direct joint keys

const KEYMAP = {
  left: { KeyW: [1, 0, 0], KeyS: [-1, 0, 0], KeyA: [0, 1, 0], KeyD: [0, -1, 0], KeyQ: [0, 0, 1], KeyE: [0, 0, -1],
    pitch: { KeyR: 1, KeyF: -1 }, roll: { KeyZ: 1, KeyC: -1 }, grip: 'KeyV' },
  right: { KeyI: [1, 0, 0], KeyK: [-1, 0, 0], KeyJ: [0, 1, 0], KeyL: [0, -1, 0], KeyU: [0, 0, 1], KeyO: [0, 0, -1],
    pitch: { KeyY: 1, KeyH: -1 }, roll: { KeyN: 1, KeyM: -1 }, grip: 'KeyB' },
};

async function main() {
  const loadingText = $('#loading-text');
  let sim;
  try {
    sim = await Sim.create(`${ROOT}model/`, (f) => { loadingText.textContent = `正在加载模型… ${Math.round(f * 100)}%`; });
  } catch (err) {
    loadingText.textContent = `加载失败：${err.message ?? err}`;
    console.error(err); return;
  }
  const view = new View(sim, $('#view'));
  const player = new RoutePlayer(sim);
  const rec = new Recorder(sim);
  const state = { selected: null, physics: true, gizmoArm: null, dragging: false };
  // per-arm IK goal; rebuilt from the joint targets whenever something other than the IK moved the arm
  const goals = { left: null, right: null };
  const armKey = (arm) => ARMS[arm].joints.slice(0, 4).map((n) => sim.target[n].toFixed(5)).join(',');

  function goalFor(arm) {
    const g = goals[arm];
    if (g && g.key === armKey(arm)) return g;
    const ee = sim.targetEE(arm);
    return (goals[arm] = { pos: ee.pos, pitch: ee.pitch, key: armKey(arm) });
  }
  /**
   * Move an arm's IK goal. strict (keyboard / jog pad): a step the arm cannot make is refused, so the hand stops at
   * the edge of the workspace instead of drifting. Otherwise (gizmo, typed numbers): go as close as possible, solving
   * repeatedly because one IK call moves each joint at most 0.25 rad.
   */
  function solve(arm, pos, pitch, { strict = false } = {}) {
    const g = goalFor(arm);
    pos = [pos[0], pos[1], Math.max(DESK_Z, pos[2])];
    const run = (pt) => {
      let r;
      for (let k = 0; k < (strict ? 1 : 12); k++) {
        r = sim.ik(arm, pos, pt);
        const moved = Object.entries(r.q).some(([n, v]) => Math.abs(v - sim.target[n]) > 1e-4);
        if (strict && r.err > 0.004) break;
        sim.setTargets(r.q);
        if (r.err < 0.001 || !moved) break;
      }
      return r;
    };
    const before = Object.fromEntries(ARMS[arm].joints.map((n) => [n, sim.target[n]]));
    let r = run(pitch);
    if (!strict && r.err > 0.004 && pitch != null) { sim.setTargets(before); r = run(null); pitch = null; }   // position first: give up the wrist pitch
    const stuck = r.err > 0.004;
    if (!strict || !stuck) {
      g.pos = stuck ? r.pos : pos;
      g.pitch = pitch == null || Math.abs(r.pitch - pitch) > 0.05 ? r.pitch : pitch;
    }
    g.key = armKey(arm);
    ui.warn(arm, stuck ? (strict ? '已到可达范围的边缘' : `够不到目标（差 ${(r.err * 1000).toFixed(0)} mm），已停在最近处`) : '');
    return r;
  }
  function manual() { if (player.playing) { player.pause(); ui.routeState(); } }

  // ---------------------------------------------------------------- gizmo (drag the end effector)
  const tc = new TransformControls(view.camera, view.renderer.domElement);
  tc.setMode('translate'); tc.setSpace('local'); tc.setSize(0.7);
  const proxy = new THREE.Object3D(); proxy.rotation.z = Math.PI;   // local axes = base frame (x forward, y left, z up)
  view.world.add(proxy);
  const gizmo = tc.getHelper(); view.scene.add(gizmo);
  tc.addEventListener('dragging-changed', (e) => { view.orbit.enabled = !e.value; state.dragging = e.value; if (e.value) manual(); });
  tc.addEventListener('objectChange', () => {
    if (!state.gizmoArm) return;
    const g = goalFor(state.gizmoArm);
    solve(state.gizmoArm, [proxy.position.x, proxy.position.y, proxy.position.z], g.pitch);
  });
  function setGizmo(arm) {
    state.gizmoArm = arm;
    if (arm) tc.attach(proxy); else tc.detach();
    for (const a of Object.keys(ARMS)) ui.armBtn(a, a === arm);
  }

  // ---------------------------------------------------------------- picking (click a part → select its joint)
  const ray = new THREE.Raycaster(); let downAt = null;
  const canvas = view.renderer.domElement;
  canvas.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
  canvas.addEventListener('pointerup', (e) => {
    if (!downAt || state.dragging || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5) return;
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), view.camera);
    const hit = ray.intersectObjects(view.pickables, false)[0];
    if (!hit) return;
    let body = hit.object.userData.body;
    while (body > 0 && sim.m.body_jntnum[body] === 0) body = sim.m.body_parentid[body];
    if (body <= 0) { select(null); setGizmo(null); return; }
    const jname = sim.joints.find((j) => j.body === body)?.name;
    select(jname);
    const arm = jname?.startsWith('left_arm') ? 'left' : jname?.startsWith('right_arm') ? 'right' : null;
    setGizmo(arm);
  });

  function select(name) {
    state.selected = name;
    view.highlightBody(name ? sim.byName[name].body : null);
    ui.selectJoint(name);
  }

  // ---------------------------------------------------------------- keyboard
  const down = new Set();
  const typing = () => ['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName) && document.activeElement.type !== 'range';
  window.addEventListener('keydown', (e) => {
    if (typing()) return;
    if (e.key === '?') { toggleHelp(); return; }
    if (e.code === 'Escape') { $('#help').hidden = true; return; }
    const handled = keyAction(e);
    if (handled || isHoldKey(e.code)) { e.preventDefault(); }
    if (isHoldKey(e.code)) { if (!down.has(e.code)) manual(); down.add(e.code); }
  });
  window.addEventListener('keyup', (e) => down.delete(e.code));
  window.addEventListener('blur', () => down.clear());

  const HOLD = new Set(['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Comma', 'Period']);
  for (const a of Object.values(KEYMAP)) { for (const k of Object.keys(a)) if (k.startsWith('Key')) HOLD.add(k); Object.keys(a.pitch).forEach((k) => HOLD.add(k)); Object.keys(a.roll).forEach((k) => HOLD.add(k)); }
  const isHoldKey = (c) => HOLD.has(c);

  function keyAction(e) {
    if (e.repeat) return ['Space', 'BracketLeft', 'BracketRight'].includes(e.code);
    switch (e.code) {
      case 'KeyX': manual(); resetHome(); return true;
      case 'KeyG': manual(); readyPose(); return true;
      case 'Space': ui.togglePlay(); return true;
      case 'KeyP': view.pip.on = !view.pip.on; $('#pip-label').hidden = $('#pip-frame').hidden = !view.pip.on; return true;
      case 'BracketLeft': case 'BracketRight': {
        const i = sim.joints.findIndex((j) => j.name === state.selected);
        const n = sim.joints.length; const k = e.code === 'BracketRight' ? (i + 1) % n : (i - 1 + n) % n;
        select(sim.joints[i < 0 ? 0 : k].name); return true;
      }
      default:
        for (const arm of Object.keys(KEYMAP)) if (e.code === KEYMAP[arm].grip) { manual(); toggleGrip(arm); return true; }
    }
    return false;
  }

  function toggleGrip(arm) {
    const n = ARMS[arm].joints[5];
    sim.setTarget(n, sim.target[n] > 10 * DEG ? sim.byName[n].lower : GRIP_OPEN);
  }

  function handleKeys(dt) {
    if (!down.size) return;
    const fast = down.has('ShiftLeft') || down.has('ShiftRight') ? 2.5 : 1;
    for (const arm of Object.keys(KEYMAP)) {
      const km = KEYMAP[arm]; const v = [0, 0, 0]; let dp = 0, dr = 0;
      for (const [k, dir] of Object.entries(km)) if (k.startsWith('Key') && Array.isArray(dir) && down.has(k)) dir.forEach((x, i) => { v[i] += x; });
      for (const [k, s] of Object.entries(km.pitch)) if (down.has(k)) dp += s;
      for (const [k, s] of Object.entries(km.roll)) if (down.has(k)) dr += s;
      if (v.some((x) => x) || dp) jogArm(arm, v.map((x) => x * EE_SPEED * fast * dt), dp * PITCH_SPEED * fast * dt);
      if (dr) { const n = ARMS[arm].joints[4]; sim.setTarget(n, sim.target[n] + dr * JOINT_SPEED * fast * dt); }
    }
    const jog = (n, s) => sim.setTarget(n, sim.target[n] + s * JOINT_SPEED * fast * dt);
    if (down.has('ArrowLeft')) jog('neck_j1', 1);
    if (down.has('ArrowRight')) jog('neck_j1', -1);
    if (down.has('ArrowUp')) jog('neck_j6', 1);
    if (down.has('ArrowDown')) jog('neck_j6', -1);
    if (state.selected && down.has('Period')) jog(state.selected, 1);
    if (state.selected && down.has('Comma')) jog(state.selected, -1);
  }

  // dBase: displacement in the base frame (m); dPitch: change of tool elevation (rad)
  function jogArm(arm, dBase, dPitch = 0) {
    const g = goalFor(arm);
    const dw = baseToWorld(dBase);
    solve(arm, [g.pos[0] + dw[0], g.pos[1] + dw[1], g.pos[2] + dw[2]], Math.max(-1.5, Math.min(1.5, g.pitch + dPitch)), { strict: true });
  }

  function resetHome() { player.stop(); sim.setTargets(sim.home); ui.routeState(); }
  function readyPose() { player.stop(); sim.setTargets(Object.fromEntries(Object.entries(READY).map(([n, v]) => [n, v * DEG]))); ui.routeState(); }

  // ---------------------------------------------------------------- panel
  const ui = buildPanel(sim, { jogArm, solve, goalFor, setGizmo, toggleGrip, select, manual, player, rec, resetHome, readyPose, state });
  function toggleHelp() { $('#help').hidden = !$('#help').hidden; }
  $('#help-btn').addEventListener('click', toggleHelp);
  $('#help-close').addEventListener('click', toggleHelp);
  $('#help').addEventListener('click', (e) => { if (e.target.id === 'help') toggleHelp(); });
  $('#pause-physics').addEventListener('click', (e) => { state.physics = !state.physics; e.target.textContent = state.physics ? '暂停物理' : '继续物理'; });

  // ---------------------------------------------------------------- loop
  $('#loading').remove();
  const markers = Object.fromEntries(Object.keys(ARMS).map((a) => [a, view.marker(`ee_${a}`, ARM_COLOR[a], 0.011)]));
  let last = performance.now(), fpsT = 0, frames = 0, uiT = 0;
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    advance(dt);
    requestAnimationFrame(frame);
  }
  function advance(dt) {
    handleKeys(dt);
    const lbl = player.update(dt);
    if (lbl != null) ui.routeLabel(lbl);
    if (state.physics) sim.step(dt);
    for (const a of Object.keys(ARMS)) { const p = sim.targetEE(a).pos; markers[a].position.set(...p); }
    if (state.gizmoArm && !state.dragging) proxy.position.set(...goalFor(state.gizmoArm).pos);
    view.draw([gizmo]);
    const pr = view.pipRect(); const pf = $('#pip-frame');
    $('#pip-label').style.bottom = `${pr.bottom + pr.height + 4}px`;
    Object.assign(pf.style, { left: `${pr.left - 2}px`, bottom: `${pr.bottom - 2}px`, width: `${pr.width + 4}px`, height: `${pr.height + 4}px` });
    frames++; fpsT += dt; uiT += dt;
    if (fpsT > 0.5) { $('#hud-fps').textContent = `${Math.round(frames / fpsT)} fps`; frames = 0; fpsT = 0; }
    if (uiT > 0.1) { ui.refresh(); uiT = 0; }
  }
  requestAnimationFrame(frame);
  setTimeout(readyPose, 600);   // the robot starts in its rest pose and "wakes up" into the ready pose
  window.wallerobot = { sim, view, player, rec, ui, advance, tc, proxy };   // handy from the browser console (advance(dt) = one frame)
}

// ======================================================================= panel
function buildPanel(sim, ctx) {
  const { player, rec } = ctx;
  const rows = {}; const ee = {};

  // speed
  const sp = $('#speed'), spv = $('#speed-val');
  sp.addEventListener('input', () => { sim.speed = sp.value * DEG; spv.textContent = `${sp.value}°/s`; });
  $('#reset').addEventListener('click', () => { ctx.manual(); ctx.resetHome(); });
  $('#ready').addEventListener('click', () => { ctx.manual(); ctx.readyPose(); });

  // end effector cards
  const eeBox = $('#ee');
  for (const arm of Object.keys(ARMS)) {
    const card = el('div', 'arm'); card.style.setProperty('--c', arm === 'left' ? 'var(--left)' : 'var(--right)');
    const head = el('div', 'arm-head'); head.append(el('b', null, ARM_NAME[arm]));
    const btns = el('div', 'row');
    const drag = el('button', 'btn ghost', '拖动手柄'); drag.addEventListener('click', () => ctx.setGizmo(ctx.state.gizmoArm === arm ? null : arm));
    const grip = el('button', 'btn ghost', '夹爪开/合'); grip.addEventListener('click', () => { ctx.manual(); ctx.toggleGrip(arm); });
    btns.append(drag, grip); head.append(btns); card.append(head);
    const xyz = el('div', 'xyz'); const inputs = [];
    ['x', 'y', 'z'].forEach((ax, i) => {
      const lab = el('label', null, ax); const inp = el('input'); inp.type = 'number'; inp.step = '5';
      inp.addEventListener('change', () => {
        ctx.manual();
        const g = ctx.goalFor(arm); const b = worldToBase(g.pos); b[i] = Number(inp.value) / 1000;
        ctx.solve(arm, baseToWorld(b), g.pitch);
        inp.blur();
      });
      inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') inp.blur(); });
      lab.append(inp); xyz.append(lab); inputs.push(inp);
    });
    card.append(xyz);
    const jog = el('div', 'jog');
    [['x−', [-1, 0, 0]], ['x+', [1, 0, 0]], ['y−', [0, -1, 0]], ['y+', [0, 1, 0]], ['z−', [0, 0, -1]], ['z+', [0, 0, 1]]].forEach(([t, v]) => {
      const b = el('button', null, t); holdButton(b, (dt) => ctx.jogArm(arm, v.map((x) => x * EE_SPEED * dt)), ctx.manual); jog.append(b);
    });
    [['俯−', -1], ['俯+', 1]].forEach(([t, s]) => {
      const b = el('button', null, t); holdButton(b, (dt) => ctx.jogArm(arm, [0, 0, 0], s * PITCH_SPEED * dt), ctx.manual); jog.append(b);
    });
    [['转−', -1], ['转+', 1]].forEach(([t, s]) => {
      const n = ARMS[arm].joints[4];
      const b = el('button', null, t); holdButton(b, (dt) => sim.setTarget(n, sim.target[n] + s * JOINT_SPEED * dt), ctx.manual); jog.append(b);
    });
    card.append(jog);
    const warn = el('div', 'ik-warn'); card.append(warn);
    eeBox.append(card);
    ee[arm] = { inputs, warn, drag };
  }

  // joints
  const jBox = $('#joints');
  const groups = [['左臂', (n) => n.startsWith('left_arm')], ['脖子', (n) => n.startsWith('neck')], ['右臂', (n) => n.startsWith('right_arm')]];
  for (const [title, f] of groups) {
    jBox.append(el('div', 'group-title', title));
    for (const j of sim.joints.filter((x) => f(x.name))) {
      const row = el('div', 'joint'); const nm = el('span', 'name', label(j.name)); nm.title = j.name;
      nm.addEventListener('click', () => ctx.select(j.name));
      const r = el('input'); r.type = 'range'; r.min = (j.lower / DEG).toFixed(1); r.max = (j.upper / DEG).toFixed(1); r.step = '0.5';
      const num = el('input'); num.type = 'number'; num.step = '1'; num.min = r.min; num.max = r.max;
      r.addEventListener('input', () => { ctx.manual(); sim.setTarget(j.name, r.value * DEG); num.value = Number(r.value).toFixed(1); });
      r.addEventListener('pointerdown', () => ctx.select(j.name));
      num.addEventListener('change', () => { ctx.manual(); sim.setTarget(j.name, num.value * DEG); num.blur(); });
      num.addEventListener('keydown', (e) => { if (e.key === 'Enter') num.blur(); });
      row.append(nm, r, num); jBox.append(row);
      rows[j.name] = { row, r, num };
    }
  }

  // routes
  const sel = $('#route-select'); const routes = [];
  const desc = $('#route-desc'); const playBtn = $('#route-play');
  fetch(`${ROOT}routes/index.json`).then((r) => r.json()).then(async (idx) => {
    for (const f of idx.routes) {
      try { const rt = await (await fetch(`${ROOT}routes/${f}`)).json(); addRoute(rt); } catch (err) { console.warn('route', f, err); }
    }
    showDesc();
  });
  function addRoute(rt, select = false) {
    routes.push(rt); const o = el('option', null, rt.name); o.value = String(routes.length - 1); sel.append(o);
    if (select) { sel.value = o.value; showDesc(); }
  }
  function showDesc() { const rt = routes[Number(sel.value)]; desc.textContent = rt?.description ?? ''; if (rt) $('#route-home').checked = rt.from !== 'current'; }
  sel.addEventListener('change', () => { player.stop(); routeState(); showDesc(); });
  function togglePlay() {
    if (player.playing) { player.pause(); routeState(); return; }
    const rt = routes[Number(sel.value)]; if (!rt) return;
    if (player.route !== rt || player.t === 0 || player.t >= player.plan.T) player.load(rt, { fromHome: $('#route-home').checked });
    player.loop = $('#route-loop').checked;
    player.play(); routeState();
  }
  playBtn.addEventListener('click', togglePlay);
  $('#route-stop').addEventListener('click', () => { player.stop(); routeState(); $('#hud-route').textContent = ''; });
  $('#route-loop').addEventListener('change', (e) => { player.loop = e.target.checked; });
  function routeState() { playBtn.textContent = player.playing ? '❚❚ 暂停' : '▶ 播放'; }

  // recorder
  const recBtns = ['#rec-play', '#rec-save', '#rec-clear'].map((s) => $(s));
  const recCount = () => { $('#rec-count').textContent = `${rec.waypoints.length} 个点`; recBtns.forEach((b) => { b.disabled = !rec.waypoints.length; }); };
  $('#rec-add').addEventListener('click', () => { rec.add(); recCount(); });
  $('#rec-clear').addEventListener('click', () => { rec.clear(); recCount(); });
  $('#rec-play').addEventListener('click', () => {
    ctx.manual(); player.load(rec.toRoute(), { fromHome: false }); player.loop = $('#route-loop').checked; player.play(); routeState();
  });
  $('#rec-save').addEventListener('click', () => {
    const blob = new Blob([JSON.stringify(rec.toRoute(), null, 1)], { type: 'application/json' });
    const a = el('a'); a.href = URL.createObjectURL(blob); a.download = `wallerobot_route_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '')}.json`;
    a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });
  $('#route-file').addEventListener('change', async (e) => {
    const f = e.target.files[0]; if (!f) return;
    try {
      const rt = JSON.parse(await f.text());
      if (!Array.isArray(rt.waypoints) || !rt.waypoints.length) throw new Error('没有 waypoints');
      rt.name = `导入：${rt.name ?? f.name}`; addRoute(rt, true);
    } catch (err) { alert(`无法读取这个路线文件：${err.message}`); }
    e.target.value = '';
  });

  return {
    togglePlay, routeState,
    routeLabel(t) { $('#hud-route').textContent = t ? `路线：${t}` : ''; $('#route-bar').style.width = `${player.progress() * 100}%`; if (!player.playing) routeState(); },
    warn(arm, t) { ee[arm].warn.textContent = t; },
    armBtn(arm, on) { ee[arm].drag.classList.toggle('on', on); },
    selectJoint(name) {
      for (const [n, r] of Object.entries(rows)) r.row.classList.toggle('sel', n === name);
      if (name) rows[name].row.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    },
    refresh() {
      const act = document.activeElement;
      for (const [n, r] of Object.entries(rows)) {
        const v = sim.target[n] / DEG;
        if (act !== r.r) r.r.value = v.toFixed(1);
        if (act !== r.num) r.num.value = v.toFixed(1);
      }
      for (const arm of Object.keys(ARMS)) {
        const b = worldToBase(sim.targetEE(arm).pos);
        ee[arm].inputs.forEach((inp, i) => { if (act !== inp) inp.value = (b[i] * 1000).toFixed(0); });
      }
      $('#route-bar').style.width = `${player.progress() * 100}%`;
    },
  };
}

// a button that keeps calling fn(dt) while it is held (mouse or touch)
function holdButton(btn, fn, onStart) {
  let raf = 0, last = 0;
  const tick = (now) => { fn(Math.min(0.05, (now - last) / 1000)); last = now; raf = requestAnimationFrame(tick); };
  const start = (e) => { e.preventDefault(); try { btn.setPointerCapture(e.pointerId); } catch { /* synthetic events */ } onStart?.(); fn(0.03); last = performance.now(); cancelAnimationFrame(raf); raf = requestAnimationFrame(tick); };   // a tap = one small step
  const stop = () => { cancelAnimationFrame(raf); raf = 0; };
  btn.addEventListener('pointerdown', start);
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) => btn.addEventListener(ev, stop));
}

main();
