// MuJoCo side of the web sim: load the exported model, step physics, slew joint commands, IK.
// Sim only — nothing here (or anywhere in this app) talks to hardware.
import loadMujoco from '@mujoco/mujoco';

const DEG = Math.PI / 180;
const ELBOW_MIN = 25 * DEG;   // the arm is stretched out straight at elbow_flex ≈ 15° (the upper arm is L-shaped)

export const ARMS = {
  left: { joints: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'].map((j) => `left_arm_${j}`), site: 'left_ee', jaw: 'left_arm_fixed_jaw' },
  right: { joints: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'].map((j) => `right_arm_${j}`), site: 'right_ee', jaw: 'right_arm_fixed_jaw' },
};

// robot base frame used in the UI: x forward, y left, z up. The model faces world −x, so base = world rotated 180° about z.
export const worldToBase = (p) => [-p[0], -p[1], p[2]];
export const baseToWorld = (p) => [-p[0], -p[1], p[2]];

export class Sim {
  static async create(baseUrl, onProgress = () => {}) {
    const mj = await loadMujoco();
    const man = await (await fetch(`${baseUrl}manifest.json`)).json();
    mj.FS.mkdirTree('/model/meshes');
    // `packs` (Artifact build only, see scripts/package_artifact.py): files concatenated, gzipped and base64-encoded into
    // one text file, because the Artifact host does not serve .stl
    const packs = man.packs ?? [];
    const total = man.files.length + packs.length;
    let done = 0;
    await Promise.all([
      ...man.files.map(async (f) => {
        const r = await fetch(`${baseUrl}${f}`);
        if (!r.ok) throw new Error(`${f}: HTTP ${r.status}`);
        mj.FS.writeFile(`/model/${f}`, new Uint8Array(await r.arrayBuffer()));
        onProgress(++done / total);
      }),
      ...packs.map(async (p) => {
        const r = await fetch(`${baseUrl}${p.file}`);
        if (!r.ok) throw new Error(`${p.file}: HTTP ${r.status}`);
        const bin = Uint8Array.from(atob((await r.text()).trim()), (c) => c.charCodeAt(0));
        const raw = new Uint8Array(await new Response(new Blob([bin]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());
        let off = 0;
        for (const [name, len] of p.entries) { mj.FS.writeFile(`/model/${name}`, raw.subarray(off, off + len)); off += len; }
        onProgress(++done / total);
      }),
    ]);
    const poses = await (await fetch(`${baseUrl}poses.json`)).json();
    return new Sim(mj, mj.MjModel.from_xml_path(`/model/${man.entry}`), poses);
  }

  constructor(mj, model, poses) {
    this.mj = mj; this.m = model; this.d = new mj.MjData(model); this.scratch = new mj.MjData(model);
    const m = model;
    this.joints = poses.joints.map((j) => {
      const id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT.value, j.name);
      const act = mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR.value, j.name);
      return { ...j, id, act, qadr: m.jnt_qposadr[id], dadr: m.jnt_dofadr[id], body: m.jnt_bodyid[id] };
    });
    this.byName = Object.fromEntries(this.joints.map((j) => [j.name, j]));
    this.home = Object.fromEntries(this.joints.map((j) => [j.name, j.home]));
    this.cmd = { ...this.home };        // what the actuators are told (slewed)
    this.target = { ...this.home };     // where the user wants each joint
    this.speed = 60 * DEG;              // rad/s slew limit for keyboard / slider moves
    this.gravComp = true;
    this.nv = m.nv;
    this.jacp = mj.DoubleBuffer.FromArray(new Array(3 * m.nv).fill(0));
    this.jacr = mj.DoubleBuffer.FromArray(new Array(3 * m.nv).fill(0));
    this.siteId = (n) => mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE.value, n);
    this.bodyId = (n) => mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY.value, n);
    this.reset();
  }

  clamp(name, q) { const j = this.byName[name]; return Math.min(j.upper, Math.max(j.lower, q)); }

  reset(pose = this.home) {
    const { mj, m, d } = this;
    mj.mj_resetData(m, d);
    for (const j of this.joints) { d.qpos[j.qadr] = pose[j.name]; this.cmd[j.name] = this.target[j.name] = pose[j.name]; }
    this.applyCtrl(); mj.mj_forward(m, d);
  }

  setTarget(name, q) { this.target[name] = this.clamp(name, q); }
  setTargets(q) { for (const [n, v] of Object.entries(q)) if (n in this.byName) this.setTarget(n, v); }
  // route player writes commands directly (it does its own smooth interpolation)
  setCommand(q) { for (const [n, v] of Object.entries(q)) if (n in this.byName) this.cmd[n] = this.target[n] = this.clamp(n, v); }

  applyCtrl() { const c = this.d.ctrl; for (const j of this.joints) c[j.act] = this.cmd[j.name]; }

  // advance physics by dt seconds of sim time
  step(dt) {
    const max = this.speed * dt;
    for (const j of this.joints) {
      const e = this.target[j.name] - this.cmd[j.name];
      this.cmd[j.name] += Math.max(-max, Math.min(max, e));
    }
    this.applyCtrl();
    const n = Math.max(1, Math.round(dt / this.m.opt.timestep));
    const bias = this.d.qfrc_bias, app = this.d.qfrc_applied;
    for (let i = 0; i < n; i++) {
      // gravity compensation: the real STS3215 servos hold their position stiffly; the sim's position actuators alone
      // (kp 20) would sag by a few degrees under the arm's weight
      if (this.gravComp) for (let k = 0; k < this.nv; k++) app[k] = bias[k];
      this.mj.mj_step(this.m, this.d);
    }
  }

  q(name) { return this.d.qpos[this.byName[name].qadr]; }
  qAll() { return Object.fromEntries(this.joints.map((j) => [j.name, this.d.qpos[j.qadr]])); }
  sitePos(name, data = this.d) { const i = this.siteId(name); const p = data.site_xpos; return [p[3 * i], p[3 * i + 1], p[3 * i + 2]]; }
  // tool pointing direction: fixed-jaw frame, from the wrist towards the jaw tips (−y of the jaw body, see the MJCF)
  toolDir(arm, data = this.d) { const b = this.bodyId(ARMS[arm].jaw); const R = data.xmat; return [-R[9 * b + 1], -R[9 * b + 4], -R[9 * b + 7]]; }

  // FK of a joint vector on the scratch data (no effect on the simulation)
  fk(q) {
    const { mj, m, scratch: s } = this;
    for (const j of this.joints) s.qpos[j.qadr] = q[j.name] ?? this.target[j.name];
    mj.mj_kinematics(m, s); mj.mj_comPos(m, s);
    return s;
  }

  /**
   * Damped-least-squares IK for one arm. Moves pan/lift/elbow/wrist_flex so the `*_ee` site reaches `goal` (world, m)
   * and, if `pitch` is given, the tool's elevation angle (rad, + = pointing up) matches it. Wrist roll and the gripper
   * are left alone. Starts from the current targets; returns {q, err, pos} (q already clamped to the limits).
   */
  ik(arm, goal, pitch = null, { iters = 40, lambda = 0.02, wPitch = 0.1, maxChange = 0.25 } = {}) {
    const names = ARMS[arm].joints.slice(0, 4);
    const js = names.map((n) => this.byName[n]);
    const q = { ...this.target };
    // IK-only bounds: the joint limits, at most `maxChange` rad away from the start (no jumps within one call), and the
    // elbow kept bent (elbow_flex ≥ ELBOW_MIN) so the arm never flips through the stretched-out singularity
    const lo = {}, hi = {};
    for (const n of names) {
      const j = this.byName[n];
      lo[n] = Math.max(j.lower, q[n] - maxChange, n.endsWith('elbow_flex') ? ELBOW_MIN : -Infinity);
      hi[n] = Math.min(j.upper, q[n] + maxChange);
      if (lo[n] > hi[n]) lo[n] = hi[n] = Math.min(Math.max(q[n], j.lower), j.upper);
    }
    const site = this.siteId(ARMS[arm].site);
    for (let it = 0; it < iters; it++) {
      const s = this.fk(q);
      const p = this.sitePos(ARMS[arm].site, s);
      const e = [goal[0] - p[0], goal[1] - p[1], goal[2] - p[2]];
      this.mj.mj_jacSite(this.m, s, this.jacp, this.jacr, site);
      const Jp = this.jacp.GetView(), Jr = this.jacr.GetView(), nv = this.nv;
      const J = [0, 1, 2].map((r) => js.map((j) => Jp[r * nv + j.dadr]));
      if (pitch != null) {
        const t = this.toolDir(arm, s);
        const cur = Math.asin(Math.max(-1, Math.min(1, t[2])));
        const cz = Math.max(0.2, Math.cos(cur));
        // d(elevation)/dq_k = (ω_k × t)_z / cos(elevation), ω_k = rotational Jacobian column of joint k
        J.push(js.map((j) => { const wx = Jr[j.dadr], wy = Jr[nv + j.dadr]; return wPitch * (wx * t[1] - wy * t[0]) / cz; }));
        e.push(wPitch * (pitch - cur));
      }
      if (Math.hypot(...e) < 1e-4) break;
      const dq = dls(J, e, lambda);
      for (let k = 0; k < js.length; k++) { const n = names[k]; q[n] = Math.min(hi[n], Math.max(lo[n], q[n] + Math.max(-0.2, Math.min(0.2, dq[k])))); }
    }
    const out = Object.fromEntries(names.map((n) => [n, q[n]]));
    const pos = this.sitePos(ARMS[arm].site, this.fk(q));
    const t = this.toolDir(arm, this.scratch);
    return { q: out, pos, pitch: Math.asin(Math.max(-1, Math.min(1, t[2]))), err: Math.hypot(goal[0] - pos[0], goal[1] - pos[1], goal[2] - pos[2]) };
  }

  // end effector of the TARGET pose (what the IK and the gizmo work on), world frame; pitch = tool elevation (rad)
  targetEE(arm) {
    const s = this.fk(this.target); const t = this.toolDir(arm, s);
    return { pos: this.sitePos(ARMS[arm].site, s), pitch: Math.asin(Math.max(-1, Math.min(1, t[2]))) };
  }
}

// dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e, J is m×n (small), solved with Gaussian elimination
function dls(J, e, lambda) {
  const m = J.length, n = J[0].length;
  const A = Array.from({ length: m }, (_, i) => Array.from({ length: m }, (_, k) => {
    let s = 0; for (let c = 0; c < n; c++) s += J[i][c] * J[k][c];
    return s + (i === k ? lambda * lambda : 0);
  }));
  const y = solve(A, e.slice());
  return Array.from({ length: n }, (_, c) => { let s = 0; for (let i = 0; i < m; i++) s += J[i][c] * y[i]; return s; });
}

function solve(A, b) {
  const n = b.length;
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]]; [b[c], b[p]] = [b[p], b[c]];
    for (let r = c + 1; r < n; r++) { const f = A[r][c] / A[c][c]; for (let k = c; k < n; k++) A[r][k] -= f * A[c][k]; b[r] -= f * b[c]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = b[r]; for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k]; x[r] = s / A[r][r]; }
  return x;
}

export { DEG };
