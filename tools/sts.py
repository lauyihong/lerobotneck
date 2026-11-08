"""Minimal Feetech STS3215 bus driver (SCS protocol) for XLeRobot debugging."""
import serial, time

class Bus:
    def __init__(self, port, baud=1_000_000):
        self.s = serial.Serial(port, baud, timeout=0.05)

    def _tx(self, sid, instr, params=b""):
        body = bytes([sid, len(params) + 2, instr]) + bytes(params)
        chk = (~sum(body)) & 0xFF
        self.s.reset_input_buffer()
        self.s.write(b"\xff\xff" + body + bytes([chk]))

    def _rx(self):
        h = self.s.read(4)
        if len(h) < 4 or h[:2] != b"\xff\xff":
            return None
        rest = self.s.read(h[3])
        if len(rest) < h[3]:
            return None
        return {"id": h[2], "err": rest[0], "data": rest[1:-1]}

    def ping(self, sid):
        self._tx(sid, 1)
        return self._rx() is not None

    def read(self, sid, addr, n):
        self._tx(sid, 2, [addr, n])
        r = self._rx()
        return None if r is None else r["data"]

    def write(self, sid, addr, data):
        self._tx(sid, 3, [addr] + list(data))
        return self._rx()

    def r16(self, sid, addr):
        d = self.read(sid, addr, 2)
        if d is None: return None
        v = d[0] | d[1] << 8
        return -(v & 0x7FFF) if v & 0x8000 else v

    def w16(self, sid, addr, v):
        if v < 0: v = (-v) | 0x8000
        return self.write(sid, addr, [v & 0xFF, v >> 8 & 0xFF])

    def status(self, sid):
        d = self.read(sid, 56, 15)  # pos,speed,load,volt,temp,...
        if d is None: return None
        u16 = lambda i: d[i] | d[i+1] << 8
        return {"pos": u16(0), "load": u16(4) & 0x3FF, "volt": d[6] / 10, "temp": d[7], "moving": d[10]}

    def torque(self, sid, on):
        return self.write(sid, 40, [1 if on else 0])

    def goto(self, sid, pos, acc=20, speed=600):
        self.write(sid, 41, [acc])
        return self.write(sid, 42, [pos & 0xFF, pos >> 8, 0, 0, speed & 0xFF, speed >> 8])
