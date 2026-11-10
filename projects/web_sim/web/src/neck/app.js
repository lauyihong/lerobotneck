// LeRobotNeck showcase: one page app, four routes (/ home, /play/ simulator, /neck/ exploded view, /why/ active perception) on one shared
// WebGL canvas, so switching pages only moves the camera and swaps the overlay UI. Simulation only.
import * as THREE from 'three';
import { Sim, DEG } from '../sim.js';
import { View } from '../render.js';
import { t, getLang, setLang, onLang } from './i18n.js';
import { homePage } from './home.js';
import { playPage } from './play.js';
import { neckPage } from './explode.js';
import { whyPage } from './why.js';

// site root (absolute URL) from the page's data-root ("./" on /, "../" on /play/ and /neck/)
const ROOT = new URL(document.documentElement.dataset.root ?? './', location.href);
const PATHS = { home: '', play: 'play/', neck: 'neck/', why: 'why/' };
export const DOCS = new URL('docs/', ROOT).href;
export const GITHUB = 'https://github.com/lauyihong/lerobotneck';

const $ = (s) => document.querySelector(s);
export const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v; else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (v === true) e.setAttribute(k, ''); else if (v !== false && v != null) e.setAttribute(k, v);
  }
  for (const k of kids.flat()) if (k != null) e.append(k.nodeType ? k : document.createTextNode(k));
  return e;
};

// the poses every page starts from (degrees); REST = folded on the stops (poses.json home)
export const READY = { neck_j1: 0, neck_j2: 90, neck_j3: 150, neck_j4: 0, neck_j5: 0, neck_j6: -25 };
export const deg = (q) => Object.fromEntries(Object.entries(q).map(([n, v]) => [n, v * DEG]));

function pageFromLocation() {
  const rel = decodeURI(location.pathname).slice(decodeURI(ROOT.pathname).length);
  if (rel.startsWith('play')) return 'play';
  if (rel.startsWith('neck')) return 'neck';
  if (rel.startsWith('why')) return 'why';
  return 'home';
}

async function main() {
  document.documentElement.lang = getLang() === 'zh' ? 'zh-CN' : 'en';
  const loadText = $('#loading-text'), loadBar = $('#loading-bar');
  loadText.textContent = t('loading');
  const poster = $('#poster'); poster.src = new URL('img/lerobotneck_poster.png', ROOT).href;
  poster.onerror = () => { poster.hidden = true; };

  let sim;
  try {
    sim = await Sim.create(new URL('model_neck/', ROOT).href, (f) => {
      loadText.textContent = t('loading.pct', { p: Math.round(f * 100) }); loadBar.style.width = `${Math.round(f * 100)}%`;
    });
  } catch (e) {
    loadText.textContent = t('loading.fail', { e: e.message }); console.error(e); return;
  }
  sim.kinematic = true;
  sim.speed = 90 * DEG;

  const view = new View(sim, $('#view'), {
    background: 0xe8eaee, camPos: [-0.95, 0.78, 0.62], target: [-0.08, 0.5, 0], pip: false,
  });
  view.orbit.minDistance = 0.25; view.orbit.maxDistance = 2.5;
  view.orbit.maxPolarAngle = Math.PI * 0.53;   // don't go under the desk
  // the hidden stock SO101 (stock_* bodies) only appears on the exploded-view page
  for (const mesh of view.geoms) if (mesh.userData.bodyName?.startsWith('stock_')) mesh.visible = false;
  // showcase colours: the real neck is all black; graphite prints vs black servos keep the shapes readable
  for (const mesh of view.geoms) {
    const n = mesh.userData.meshName ?? '';
    if (mesh.userData.bodyName?.startsWith('stock_')) continue;
    const hex = /Motor|topbase2/.test(n) ? 0x121315 : n === 'tophead6' ? 0x8d9096 : 0x44474e;
    mesh.material.color.set(hex); mesh.userData.baseColor = mesh.material.color.clone();
    mesh.material.roughness = 0.5;
  }

  // things on the desk for the D435i to look at: the green block and the box of the active-perception demos and a
  // few more (routes_neck/scene.json from scripts/make_neck_routes.py; three.js only, not in the MuJoCo model)
  fetch(new URL('routes_neck/scene.json', ROOT)).then((r) => r.json()).then(({ props }) => {
    for (const p of props) {
      const s = p.size;
      const geo = p.shape === 'sphere' ? new THREE.SphereGeometry(s[0], 24, 16)
        : p.shape === 'cylinder' ? new THREE.CylinderGeometry(s[0], s[0], s[1], 28).rotateX(Math.PI / 2)   // axis along z
          : new THREE.BoxGeometry(...s);
      const b = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: p.color, roughness: 0.55 }));
      b.position.set(...p.pos); b.rotation.z = p.yaw ?? 0; b.castShadow = b.receiveShadow = true; b.name = p.id;
      view.world.add(b);
    }
  }).catch((e) => console.warn('desk scene', e));

  const ctx = {
    sim, view, ROOT, el, t,
    fly: null,   // camera tween {p0,t0,p1,t1,u,T}
    // move the camera (three.js coords) to `pos` looking at `target` in `T` seconds
    flyTo(pos, target, T = 1.4) {
      const t1 = new THREE.Vector3(...target), p1 = new THREE.Vector3(...pos);
      if (innerWidth <= 760) p1.sub(t1).multiplyScalar(1.5).add(t1);   // phones: portrait, step back to fit the robot
      ctx.fly = { p0: view.camera.position.clone(), t0: view.orbit.target.clone(), p1, t1, u: 0, T };
    },
    go, href: (name) => new URL(PATHS[name], ROOT).href,
  };

  const pages = { home: homePage(ctx), play: playPage(ctx), neck: neckPage(ctx), why: whyPage(ctx) };
  let current = null;

  function go(name, { push = true } = {}) {
    if (current === pages[name]) return;
    const prev = current;
    prev?.exit?.(name);
    current = pages[name]; ctx.page = name;
    if (push) history.pushState({ page: name }, '', ctx.href(name));
    document.body.dataset.page = name;
    renderNav();
    current.enter(prev ? Object.keys(pages).find((k) => pages[k] === prev) : null);
    document.title = name === 'home' ? 'LeRobotNeck' : `${t(`nav.${name}`)} · LeRobotNeck`;
  }
  addEventListener('popstate', () => go(pageFromLocation(), { push: false }));

  // ---- top navigation
  function renderNav() {
    const nav = $('#nav'); nav.replaceChildren();
    const link = (name) => el('a', { href: ctx.href(name), class: ctx.page === name ? 'on' : '',
      onclick: (e) => { if (e.metaKey || e.ctrlKey) return; e.preventDefault(); go(name); } }, t(`nav.${name}`));
    nav.append(
      el('a', { class: 'brand', href: ctx.href('home'), onclick: (e) => { e.preventDefault(); go('home'); } },
        el('span', { class: 'logo' }), el('span', { class: 'name' }, 'LeRobotNeck')),
      el('nav', {}, link('home'), link('play'), link('neck'), link('why'),
        el('a', { href: DOCS }, t('nav.docs')), el('a', { href: GITHUB, rel: 'noopener', class: 'gh' }, t('nav.github'))),
      el('button', { class: 'lang', onclick: () => setLang(getLang() === 'en' ? 'zh' : 'en') }, t('lang.other')),
    );
  }
  onLang(() => { renderNav(); current?.lang?.(); document.title = ctx.page === 'home' ? 'LeRobotNeck' : `${t(`nav.${ctx.page}`)} · LeRobotNeck`; });

  // ---- render loop
  let last = performance.now();
  const ease = (u) => (u < 0.5 ? 4 * u * u * u : 1 - (-2 * u + 2) ** 3 / 2);
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    tick(dt);
    requestAnimationFrame(frame);
  }
  function tick(dt) {
    if (ctx.fly) {
      const f = ctx.fly; f.u = Math.min(1, f.u + dt / f.T); const k = ease(f.u);
      view.camera.position.lerpVectors(f.p0, f.p1, k); view.orbit.target.lerpVectors(f.t0, f.t1, k);
      if (f.u >= 1) ctx.fly = null;
    }
    // keep the robot clear of each page's text / panel on wide screens
    const wide = innerWidth > 760;
    view.shiftTarget = wide ? ({ home: 0.2, play: -0.17, neck: 0.14, why: -0.17 }[ctx.page] ?? 0) : 0;
    view.shiftYTarget = wide ? 0 : ({ home: 0.2, play: 0.1, neck: 0.16, why: 0.14 }[ctx.page] ?? 0);   // phones: UI at the bottom
    current?.update?.(dt);
    sim.step(dt);
    view.draw(current?.helpers ?? []);
    current?.afterDraw?.();
  }
  ctx.tick = (dt, n = 1) => { for (let i = 0; i < n; i++) tick(dt); };   // debugging / tests: advance time by hand

  go(pageFromLocation(), { push: false });
  history.replaceState({ page: ctx.page }, '', location.href);
  requestAnimationFrame((n) => { last = n; frame(n); });
  $('#loading').classList.add('done');
  setTimeout(() => $('#loading').remove(), 900);
  window.lerobotneck = ctx;   // for debugging in the console
}

main();
