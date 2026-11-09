// Page 2: neck simulator. Demo cards play one route each; pause at any time, change joints with the sliders, by
// clicking a part + keys, then resume: the route is re-planned from the adjusted pose (RoutePlayer.replanFromCurrent).
import * as THREE from 'three';
import { DEG } from '../sim.js';
import { RoutePlayer, Recorder } from '../routes.js';
import { t, tr } from './i18n.js';
import { READY, deg } from './app.js';

export const PLAY_VIEW = { pos: [-0.78, 0.72, 0.5], target: [-0.05, 0.5, 0.07] };
const JOINT_SPEED = 50 * DEG;

export function playPage(ctx) {
  const { sim, view, el } = ctx;
  const player = new RoutePlayer(sim);
  const rec = new Recorder(sim);
  const demos = [];   // {id, route}
  let fullPlan = null, ui = null, active = null, active_id = null, paused = false, pauseQ = null, adjusted = new Set(), sel = 0, big = false;
  const keys = new Set();
  const names = sim.joints.map((j) => j.name);

  fetch(new URL('routes_neck/index.json', ctx.ROOT)).then((r) => r.json()).then(async (idx) => {
    for (const id of idx.demos) {
      try { demos.push({ id, route: await (await fetch(new URL(`routes_neck/${id}.json`, ctx.ROOT))).json() }); }
      catch (e) { console.warn('route', id, e); }
    }
    if (ui) build();
  });

  // ---------- UI
  const sliders = {};
  function build() {
    const cards = el('div', { class: 'cards' }, demos.map((d) => el('button', {
      class: `card${active_id === d.id ? ' on' : ''}`, onclick: () => startDemo(d),
    }, el('b', {}, tr(d.route.name)), el('span', {}, tr(d.route.description)))));

    const transport = el('div', { class: 'transport' },
      el('button', { class: 'btn primary', id: 'pp', onclick: togglePlay, disabled: !active }, ''),
      el('button', { class: 'btn', onclick: restart, disabled: !active }, t('play.restart')),
      el('div', { class: 'progress' }, el('div', { id: 'bar' })),
      el('div', { class: 'now', id: 'now' }, active ? '' : t('play.idle')));
    const wps = el('div', { class: 'wps', id: 'wps' }, active ? active.waypoints.map((w, i) =>
      el('button', { class: 'wp', 'data-i': i, onclick: () => jumpTo(i) }, tr(w.label))) : []);
    const note = el('div', { class: 'note', id: 'note' });

    const joints = el('div', { class: 'joints' }, sim.joints.map((j, i) => {
      const lo = Math.round(j.lower / DEG), hi = Math.round(j.upper / DEG);
      const r = el('input', { type: 'range', min: lo, max: hi, step: 0.5, value: (sim.target[j.name] / DEG).toFixed(1),
        oninput: (e) => setJoint(j.name, +e.target.value * DEG) });
      const n = el('input', { type: 'number', min: lo, max: hi, step: 1, value: (sim.target[j.name] / DEG).toFixed(0),
        onchange: (e) => setJoint(j.name, +e.target.value * DEG) });
      sliders[j.name] = { r, n };
      return el('label', { class: `joint${sel === i ? ' sel' : ''}`, 'data-j': j.name, onclick: () => select(i) },
        el('span', { class: 'jn' }, t(`j.${j.name}`)), r, n, el('i', { class: 'dot' }));
    }));
    const poses = el('div', { class: 'row' },
      el('button', { class: 'btn', onclick: () => manualPose(deg(READY)) }, t('play.pose.ready')),
      el('button', { class: 'btn', onclick: () => manualPose(sim.home) }, t('play.pose.rest')),
      el('button', { class: 'btn ghost', id: 'undo', hidden: true, onclick: undo }, t('play.undo')));

    const recBox = el('details', { class: 'rec' }, el('summary', {}, t('play.rec')),
      el('div', { class: 'row' },
        el('button', { class: 'btn ghost', onclick: () => { rec.add(t('play.rec.wp', { n: rec.waypoints.length + 1 })); build(); } }, t('play.rec.add')),
        el('span', { class: 'muted' }, t('play.rec.n', { n: rec.waypoints.length })),
        el('button', { class: 'btn ghost', disabled: !rec.waypoints.length, onclick: () => startDemo({ id: 'rec', route: rec.toRoute(t('play.rec.name')) }) }, t('play.rec.play')),
        el('button', { class: 'btn ghost', disabled: !rec.waypoints.length, onclick: download }, t('play.rec.save')),
        el('button', { class: 'btn ghost', disabled: !rec.waypoints.length, onclick: () => { rec.clear(); build(); } }, t('play.rec.clear'))));

    ui = el('section', { class: 'play' },
      el('div', { class: 'pip-frame', id: 'pipf' }, el('span', {}, t('play.cam')),
        el('button', { class: 'chip', onclick: () => { big = !big; view.pip.scale = big ? 1.7 : 1; build(); } }, big ? t('play.cam.small') : t('play.cam.big'))),
      el('aside', { class: 'panel' },
        el('h2', {}, t('play.title')), el('p', { class: 'muted' }, t('play.sub')),
        el('h3', {}, t('play.demos')), cards, transport, wps, note,
        el('h3', {}, t('play.joints')), joints, poses, recBox,
        el('p', { class: 'muted small' }, t('play.keys'))));
    document.querySelector('#ui').replaceChildren(ui);
    refresh();
    maybeGuide();
  }

  function refresh() {
    const pp = ui?.querySelector('#pp');
    if (pp) pp.textContent = player.playing ? `❚❚ ${t('play.pause')}` : paused ? `▶ ${t('play.resume')}` : `▶ ${t('play.play')}`;
    const note = ui?.querySelector('#note');
    if (note) {
      note.textContent = paused ? (adjusted.size ? t('play.adjusted', { j: [...adjusted].map((n) => t(`j.${n}`)).join(', ') }) : t('play.paused')) : '';
      note.hidden = !paused;
    }
    const undoB = ui?.querySelector('#undo'); if (undoB) undoB.hidden = !(paused && adjusted.size);
    for (const n of names) ui?.querySelector(`[data-j="${n}"]`)?.classList.toggle('adj', adjusted.has(n));
    const r = view.pipRect(); const f = ui?.querySelector('#pipf');
    if (f) Object.assign(f.style, { left: `${r.left - 2}px`, bottom: `${r.bottom - 2}px`, width: `${r.width + 4}px`, height: `${r.height + 4}px` });
  }

  // ---------- demo control
  function startDemo(d) {
    active = d.route; active_id = d.id; paused = false; adjusted.clear();
    player.load(d.route); fullPlan = player.plan; player.play(); build();
  }
  function togglePlay() {
    if (!active) return;
    if (player.playing) { player.pause(); paused = true; pauseQ = { ...sim.target }; }
    else {
      if (paused && adjusted.size) player.replanFromCurrent();
      else if (player.status?.kind === 'done') player.restart();
      paused = false; adjusted.clear(); player.play();
    }
    refresh();
  }
  function restart() { if (!active) return; paused = false; adjusted.clear(); player.restart(); player.play(); refresh(); }
  function jumpTo(i) {
    if (!active) return;
    // full pose at waypoint i-1 of the original plan, so a partial waypoint i gets the joints set before it
    const full = fullPlan?.segs?.[i - 1]?.to;
    player.seek(i, { from: full ?? { ...sim.target } }); paused = false; adjusted.clear(); player.play(); refresh();
  }
  // a joint changed by the user: pauses a running demo first (the change is kept when it resumes)
  function setJoint(n, q) {
    if (player.playing) { player.pause(); paused = true; pauseQ = { ...sim.target }; }
    sim.setTarget(n, q);
    if (paused) adjusted.add(n);
    refresh();
  }
  function manualPose(q) {
    if (player.playing) { player.pause(); paused = true; pauseQ = { ...sim.target }; }
    sim.setTargets(q); if (paused) names.forEach((n) => adjusted.add(n)); refresh();
  }
  function undo() { if (pauseQ) sim.setTargets(pauseQ); adjusted.clear(); refresh(); }
  function select(i) { sel = i; ui?.querySelectorAll('.joint').forEach((e, k) => e.classList.toggle('sel', k === i)); view.highlightBody(sim.joints[i].body); }
  function download() {
    const blob = new Blob([JSON.stringify(rec.toRoute(t('play.rec.name')), null, 1)], { type: 'application/json' });
    const a = el('a', { href: URL.createObjectURL(blob), download: 'lerobotneck_route.json' }); document.body.append(a); a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  }

  // ---------- first-visit guide
  function maybeGuide() {
    let seen = false; try { seen = localStorage.getItem('lerobotneck.guide') === '1'; } catch { seen = true; }
    if (seen || ui.querySelector('.guide')) return;
    let step = 1;
    const box = el('div', { class: 'guide' });
    const close = () => { box.remove(); try { localStorage.setItem('lerobotneck.guide', '1'); } catch { /* blocked */ } };
    const show = () => {
      box.replaceChildren(el('div', { class: 'guide-card' },
        el('small', {}, `${step} / 3`), el('b', {}, t(`guide.${step}.t`)), el('p', {}, t(`guide.${step}.b`)),
        el('div', { class: 'row' },
          el('button', { class: 'btn ghost', onclick: close }, t('guide.skip')),
          el('button', { class: 'btn primary', onclick: () => { if (step === 3) close(); else { step++; show(); } } }, step === 3 ? t('guide.done') : t('guide.next')))));
    };
    show(); ui.append(box);
  }

  // ---------- picking + keys
  const cv = view.renderer.domElement; const ray = new THREE.Raycaster(); const ndc = new THREE.Vector2(); let down = null;
  const onDown = (e) => { down = [e.clientX, e.clientY]; };
  const onUp = (e) => {
    if (!down) return; const moved = Math.hypot(e.clientX - down[0], e.clientY - down[1]); down = null;
    if (moved > 5) return;
    const r = cv.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, view.camera);
    const hit = ray.intersectObjects(view.geoms.filter((g) => g.visible), false)[0];
    if (!hit) return;
    const k = sim.joints.findIndex((j) => j.body === hit.object.userData.body);
    if (k >= 0) select(k);
  };
  const onKey = (e) => {
    if (e.target.closest?.('input, select, textarea')) return;
    if (e.type === 'keydown') {
      if (e.code === 'Space') { e.preventDefault(); togglePlay(); return; }
      if (e.code === 'BracketLeft') select((sel + names.length - 1) % names.length);
      if (e.code === 'BracketRight') select((sel + 1) % names.length);
      if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(e.code)) e.preventDefault();
      keys.add(e.code);
    } else keys.delete(e.code);
  };

  return {
    enter() {
      sim.speed = 70 * DEG; build(); view.pip.on = true;
      ctx.flyTo(PLAY_VIEW.pos, PLAY_VIEW.target, 1.3);
      player.text = { toStart: t('play.toStart'), done: t('play.done'), dwell: (l) => t('play.dwell', { l: tr(l) }) };
      if (!active) sim.setTargets(deg(READY));
      cv.addEventListener('pointerdown', onDown); cv.addEventListener('pointerup', onUp);
      addEventListener('keydown', onKey); addEventListener('keyup', onKey);
    },
    exit() {
      player.stop(); paused = false; adjusted.clear(); active = null; active_id = null; view.pip.on = false; view.pip.top = null; view.pip.scale = 1; big = false; view.highlightBody(null);
      cv.removeEventListener('pointerdown', onDown); cv.removeEventListener('pointerup', onUp);
      removeEventListener('keydown', onKey); removeEventListener('keyup', onKey); keys.clear();
    },
    update(dt) {
      view.pip.top = innerWidth <= 760 ? 80 : null;   // phones: the panel is a bottom sheet, inset goes to the top
      // keyboard jog (pauses a running demo like the sliders)
      const jog = (n, s) => setJoint(n, sim.target[n] + s * JOINT_SPEED * dt * (keys.has('ShiftLeft') ? 2.5 : 1));
      if (keys.has('ArrowLeft')) jog('neck_j1', -1); if (keys.has('ArrowRight')) jog('neck_j1', 1);
      if (keys.has('ArrowUp')) jog('neck_j6', -1); if (keys.has('ArrowDown')) jog('neck_j6', 1);
      if (keys.has('Comma')) jog(names[sel], -1); if (keys.has('Period')) jog(names[sel], 1);

      const label = player.update(dt);
      if (!ui) return;
      if (player.status?.kind === 'done' && !player.playing && !paused) refresh();
      const bar = ui.querySelector('#bar');
      if (bar && player.plan) {
        const s = player.status, segs = player.plan.segs;
        const n = active?.waypoints.length ?? 1;
        // progress over the whole route, also after a seek / replan (segment index + time within it)
        let f = 0;
        if (s && s.seg >= 0) { const sg = segs.find((x) => x.i === s.seg); f = (s.seg + (sg ? Math.min(1, (player.t - sg.t0) / Math.max(0.01, (sg.t1 - sg.t0))) : 1)) / n; }
        if (s?.kind === 'done') f = 1;
        bar.style.width = `${Math.round(f * 100)}%`;
        if (label != null) ui.querySelector('#now').textContent = tr(label);
        ui.querySelectorAll('.wp').forEach((b) => b.classList.toggle('on', +b.dataset.i === s?.seg));
      }
      for (const n of names) {
        const s = sliders[n]; if (!s) continue;
        const v = sim.target[n] / DEG;
        if (document.activeElement !== s.r) s.r.value = v.toFixed(1);
        if (document.activeElement !== s.n) s.n.value = v.toFixed(0);
      }
      refresh();
    },
    lang() { build(); player.text = { toStart: t('play.toStart'), done: t('play.done'), dwell: (l) => t('play.dwell', { l: tr(l) }) }; },
    get helpers() { return []; },
  };
}
