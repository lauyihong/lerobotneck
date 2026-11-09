// Page 1: hero. The neck loops a short show (wake up, look around, tilt, peek, rest) while the camera slowly orbits.
// Hover the neck to highlight it, click it to fly into the exploded view.
import * as THREE from 'three';
import { RoutePlayer } from '../routes.js';
import { t, tr } from './i18n.js';

export const HOME_VIEW = { pos: [-0.8, 0.6, 0.85], target: [-0.05, 0.44, 0.05] };

export function homePage(ctx) {
  const { sim, view, el } = ctx;
  const player = new RoutePlayer(sim);
  player.loop = true;
  let route = null, ui = null, idleT = 0, hover = false, down = null, active = false;
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  const neckMeshes = () => view.geoms.filter((g) => g.visible && g.userData.bodyName?.startsWith('neck_'));

  fetch(new URL('routes_neck/index.json', ctx.ROOT)).then((r) => r.json())
    .then((idx) => fetch(new URL(`routes_neck/${idx.home}.json`, ctx.ROOT))).then((r) => r.json())
    .then((r) => { route = r; if (active) start(); })
    .catch((e) => console.warn('home route', e));

  function start() {
    sim.setPose(sim.home);
    player.load(route, { fromHome: true }); player.play();
  }

  function build() {
    ui = el('section', { class: 'home' },
      el('div', { class: 'hero' },
        el('p', { class: 'kicker' }, t('home.kicker')),
        el('h1', {}, t('home.title')),
        el('p', { class: 'lead' }, t('home.lead')),
        el('ul', { class: 'stats' },
          el('li', {}, el('b', {}, '6'), t('home.stat.dof')),
          el('li', {}, el('b', {}, '1'), t('home.stat.parts')),
          el('li', {}, el('b', {}, 'D435i'), t('home.stat.cam'))),
        el('div', { class: 'cta' },
          el('a', { class: 'btn primary', href: ctx.href('play'), onclick: (e) => { e.preventDefault(); ctx.go('play'); } }, t('home.cta.play')),
          el('a', { class: 'btn', href: ctx.href('neck'), onclick: (e) => { e.preventDefault(); ctx.go('neck'); } }, t('home.cta.neck'))),
        el('p', { class: 'hint' }, t('home.hint'))),
      el('div', { class: 'status', id: 'home-status' }),
      el('div', { class: 'tip', id: 'home-tip', hidden: true }, t('home.tip')),
      el('footer', {}, t('footer')));
    document.querySelector('#ui').replaceChildren(ui);
  }

  const cv = view.renderer.domElement;
  const onDown = (e) => { down = [e.clientX, e.clientY]; };
  const pick = (e) => {
    const r = cv.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, view.camera);
    return ray.intersectObjects(neckMeshes(), false).length > 0;
  };
  const onMove = (e) => {
    if (!active || e.buttons) return;
    const h = pick(e);
    if (h !== hover) { hover = h; for (const g of neckMeshes()) { g.material.emissive.set(h ? 0xff6a13 : 0); g.material.emissiveIntensity = h ? 0.22 : 0; } }
    cv.style.cursor = h ? 'pointer' : '';
    const tip = document.querySelector('#home-tip');
    if (tip) { tip.hidden = !h; tip.style.left = `${e.clientX + 14}px`; tip.style.top = `${e.clientY + 10}px`; }
  };
  const onUp = (e) => {
    if (!active || !down) return;
    const moved = Math.hypot(e.clientX - down[0], e.clientY - down[1]); down = null;
    if (moved < 5 && pick(e)) ctx.go('neck');
  };
  const onOrbit = () => { idleT = 0; view.orbit.autoRotate = false; };

  return {
    enter(prev) {
      active = true; build();
      player.text = { toStart: t('play.toStart'), done: t('play.done'), dwell: (l) => tr(l) };
      ctx.flyTo(HOME_VIEW.pos, HOME_VIEW.target, prev ? 1.6 : 0.01);
      view.orbit.autoRotateSpeed = 0.55; view.orbit.autoRotate = true;
      cv.addEventListener('pointerdown', onDown); cv.addEventListener('pointermove', onMove); cv.addEventListener('pointerup', onUp);
      view.orbit.addEventListener('start', onOrbit);
      if (route) { if (prev) { player.load(route, { fromHome: true }); player.play(); } else start(); }
    },
    exit() {
      active = false; player.stop(); view.orbit.autoRotate = false; hover = false; cv.style.cursor = '';
      for (const g of view.geoms) { g.material.emissive.set(0); g.material.emissiveIntensity = 0; }
      cv.removeEventListener('pointerdown', onDown); cv.removeEventListener('pointermove', onMove); cv.removeEventListener('pointerup', onUp);
      view.orbit.removeEventListener('start', onOrbit);
    },
    update(dt) {
      player.update(dt);
      idleT += dt;
      if (!view.orbit.autoRotate && idleT > 5 && !ctx.fly) view.orbit.autoRotate = true;
      const st = document.querySelector('#home-status');
      const s = player.status;
      if (st && s) st.textContent = s.seg >= 0 && route ? tr(route.waypoints[s.seg].label) : '';
    },
    lang() { build(); player.text.toStart = t('play.toStart'); },
  };
}
