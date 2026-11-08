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
  constructor(sim) { this.sim = sim; this.plan = null; this.t = 0; this.playing = false; this.loop = false; this.route = null; }

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
      if (far) { this.sim.setTargets(this.start); return '移动到起点'; }
    }
    this.t += dt;
    if (this.t >= this.plan.T) {
      this.sim.setCommand(this.plan.end);
      if (this.loop) { this.t = 0; this.start = { ...this.plan.end }; this.plan = planRoute(this.sim, this.route, this.start); }
      else { this.playing = false; return '完成'; }
    }
    let label = '';
    for (const s of this.plan.segs) {
      if (this.t < s.t0) break;
      const u = Math.min(1, (this.t - s.t0) / (s.t1 - s.t0));
      const k = minJerk(u);
      const q = {};
      for (const n of Object.keys(s.to)) q[n] = s.from[n] + k * (s.to[n] - s.from[n]);
      this.sim.setCommand(q);
      label = u < 1 ? s.label : `${s.label}（停留）`;
    }
    return label;
  }

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
