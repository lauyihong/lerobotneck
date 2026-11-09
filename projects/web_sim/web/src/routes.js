// Fixed routes: a list of waypoints (joint angles in degrees, partial — unspecified joints keep their value),
// played with minimum-jerk interpolation between stops.
//
// JSON format (public/routes/*.json):
// { "name": "...", "description": "...", "speed_dps": 30, "dwell_s": 1.0, "from": "current" | "home",
//   "waypoints": [ { "label": "...", "q": { "neck_j1": -45, ... }, "duration_s": 2.0 (optional), "dwell_s": 0.5 (optional) } ] }
import { DEG } from './sim.js';

const minJerk = (s) => s * s * s * (10 - 15 * s + 6 * s * s);

export function planRoute(sim, route, start) {
  const speed = (route.speed_dps ?? 30) * DEG;
  let cur = { ...start };
  const segs = []; let t = 0;
  for (const [i, wp] of route.waypoints.entries()) {
    const to = { ...cur };
    for (const [n, v] of Object.entries(wp.q || {})) if (n in sim.byName) to[n] = sim.clamp(n, v * DEG);
    const dmax = Math.max(0, ...Object.keys(to).map((n) => Math.abs(to[n] - cur[n])));
    // min-jerk peak speed is 1.875 × the mean speed: size the segment so the peak stays at `speed`
    const T = wp.duration_s ?? Math.max(0.6, 1.875 * dmax / speed);
    segs.push({ i, label: wp.label || `#${i + 1}`, t0: t, t1: t + T, from: cur, to });
    t += T + (wp.dwell_s ?? route.dwell_s ?? 0.8);
    cur = to;
  }
  return { segs, T: t, end: cur };
}

export class RoutePlayer {
  constructor(sim) {
    this.sim = sim; this.plan = null; this.t = 0; this.playing = false; this.loop = false; this.route = null;
    // UI strings (the LeRobotNeck pages pass translated ones); status = {kind: start|move|dwell|done, seg} for richer UIs
    this.text = { toStart: '移动到起点', done: '完成', dwell: (l) => `${l}（停留）` };
    this.status = null;
  }

  load(route, { fromHome = route.from === 'home' } = {}) {
    this.route = route;
    const start = fromHome ? { ...this.sim.home } : { ...this.sim.target };
    this.start = start; this.fromHome = fromHome;
    this.plan = planRoute(this.sim, route, start);
    this.t = 0;
  }

  play() {
    if (!this.plan) return;
    if (this.t >= this.plan.T) this.t = 0;
    if (this.t === 0 && this.fromHome) this.sim.setTargets(this.sim.home);
    this.playing = true;
  }
  pause() { this.playing = false; }
  stop() { this.playing = false; this.t = 0; }

  // returns the label of the active segment (for the UI)
  update(dt) {
    if (!this.playing || !this.plan) return null;
    // wait until the robot has reached the route's start pose before the clock runs
    if (this.t === 0) {
      const far = Object.entries(this.start).some(([n, v]) => Math.abs(this.sim.cmd[n] - v) > 0.5 * DEG);
      if (far) { this.sim.setTargets(this.start); this.status = { kind: 'start', seg: -1 }; return this.text.toStart; }
    }
    this.t += dt;
    if (this.t >= this.plan.T) {
      this.sim.setCommand(this.plan.end);
      if (this.loop) { this.t = 0; this.start = { ...this.plan.end }; this.plan = planRoute(this.sim, this.route, this.start); }
      else { this.playing = false; this.status = { kind: 'done', seg: this.plan.segs.length - 1 }; return this.text.done; }
    }
    let label = '';
    for (const s of this.plan.segs) {
      if (this.t < s.t0) break;
      const u = Math.min(1, (this.t - s.t0) / (s.t1 - s.t0));
      const k = minJerk(u);
      const q = {};
      for (const n of Object.keys(s.to)) q[n] = s.from[n] + k * (s.to[n] - s.from[n]);
      this.sim.setCommand(q);
      label = u < 1 ? s.label : this.text.dwell(s.label);
      this.status = { kind: u < 1 ? 'move' : 'dwell', seg: s.i };
    }
    return label;
  }

  // index of the waypoint the clock is at (moving to it, or dwelling on it)
  segAt(t = this.t) {
    if (!this.plan) return -1;
    let k = 0;
    for (const s of this.plan.segs) if (t >= s.t0) k = s.i;
    return k;
  }

  // Continue the route from wherever the robot is now (the user may have changed joints while paused): replan from
  // sim.target to the current waypoint (or the next one if we were dwelling on it) and everything after it.
  replanFromCurrent() {
    if (!this.plan || !this.route) return;
    const k = this.segAt(); const s = this.plan.segs[k];
    const next = this.t > s.t1 ? k + 1 : k;   // dwelling at k -> go on with k+1
    this.seek(next, { from: { ...this.sim.target } });
  }

  // restart the route at waypoint `k` (moving there from `from`, default the current targets)
  seek(k, { from = { ...this.sim.target } } = {}) {
    if (!this.route) return;
    const wps = this.route.waypoints;
    if (k >= wps.length) { this.t = this.plan.T; this.status = { kind: 'done', seg: wps.length - 1 }; return; }
    // carry the joints set by earlier waypoints (partial waypoints only list what changes)
    const base = { ...from };
    this.offset = k;
    this.plan = planRoute(this.sim, { ...this.route, waypoints: wps.slice(k) }, base);
    for (const sg of this.plan.segs) sg.i += k;
    this.start = base; this.fromHome = false; this.t = 0;
  }

  // full route again from its first waypoint
  restart() { if (this.route) this.load(this.route, { fromHome: this.route.from === 'home' }); }

  progress() { return this.plan ? Math.min(1, this.t / this.plan.T) : 0; }
}

// recorder: snapshot the current joint targets as waypoints, export as a route JSON
export class Recorder {
  constructor(sim) { this.sim = sim; this.waypoints = []; }
  add(label) {
    const q = Object.fromEntries(Object.entries(this.sim.target).map(([n, v]) => [n, Math.round(v / DEG * 10) / 10]));
    this.waypoints.push({ label: label || `点 ${this.waypoints.length + 1}`, q });
  }
  clear() { this.waypoints = []; }
  toRoute(name = '我的路线') {
    return { name, description: '在网页模拟器里录制', speed_dps: 30, dwell_s: 0.8, from: 'current', waypoints: this.waypoints };
  }
}
