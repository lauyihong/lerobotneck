"""READ-ONLY reader of all 18 joint encoders (COM3: left arm 1-6 + neck 7-12, COM4: right arm 1-6).

    .venv/Scripts/python.exe projects/sim_mirror/scripts/real_reader.py            # one read, prints ticks + deg
    .venv/Scripts/python.exe projects/sim_mirror/scripts/real_reader.py --json out.json

Only READ instructions are sent (Present_Position reg 56, Torque_Enable reg 40) - no register is ever written, so this
cannot move the robot. Per-ID reads (broadcast ping with 12 motors on COM3 loses replies). Close every other script
that uses COM3/COM4 first ("Access is denied" = port busy).
"""
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "tools"))
import calib  # noqa: E402
from sts import Bus  # noqa: E402

REG_TORQUE, REG_POS = 40, 56


class RealReader:
    def __init__(self, ports=("COM3", "COM4"), retries=3):
        self.retries = retries
        self.buses = {p: Bus(p) for p in ports}
        self.ids = {n: (c["port"], c["id"]) for n, c in calib.CAL.items() if c["port"] in ports}
        self.misses = 0

    def _read(self, port, sid, addr, n):
        for _ in range(self.retries):
            d = self.buses[port].read(sid, addr, n)
            if d is not None and len(d) == n:
                return d
        self.misses += 1
        return None

    def ticks(self) -> dict:
        out = {}
        for name, (p, sid) in self.ids.items():
            d = self._read(p, sid, REG_POS, 2)
            out[name] = None if d is None else d[0] | d[1] << 8
        return out

    def torque_state(self) -> dict:
        out = {}
        for name, (p, sid) in self.ids.items():
            d = self._read(p, sid, REG_TORQUE, 1)
            out[name] = None if d is None else d[0]
        return out

    def assert_torque_off(self):
        t = self.torque_state()
        on = {n: v for n, v in t.items() if v != 0}
        if on:
            raise RuntimeError(f"torque not off / unreadable on {on} - this tool only reads; check what enabled it")
        return t

    def close(self):
        for b in self.buses.values():
            b.s.close()


def main():
    r = RealReader()
    try:
        r.assert_torque_off()
        t0 = time.perf_counter(); tk = r.ticks(); dt = time.perf_counter() - t0
        r.assert_torque_off()
    finally:
        r.close()
    q = calib.ticks_to_q(tk)
    for n in calib.ORDER:
        print(f"  {n:24s} tick {tk[n]!s:>5}  ->  {math.degrees(q[n]) if n in q else float('nan'):+8.1f} deg")
    print(f"read 18 joints in {dt * 1e3:.0f} ms, missed reads {r.misses}, torque off on all before and after")
    if "--json" in sys.argv:
        p = Path(sys.argv[sys.argv.index("--json") + 1])
        p.write_text(json.dumps({"date": time.strftime("%Y-%m-%d %H:%M:%S"), "ticks": tk,
                                 "q_deg": {n: round(math.degrees(v), 2) for n, v in q.items()}}, indent=2))
        print("saved", p)


if __name__ == "__main__":
    main()
