"""Change a Feetech STS3215 ID on a bus: python set_motor_id.py COM4 4 6
Refuses unless the old ID answers cleanly (no duplicate on the bus) and the new ID is free."""
import sys
from sts import Bus

port, old, new = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
b = Bus(port)
ok = sum(b.ping(old) for _ in range(30))
if ok < 30:
    sys.exit(f"ID {old} answered {ok}/30 pings -> missing or duplicated, not touching it")
if any(b.ping(new) for _ in range(5)):
    sys.exit(f"ID {new} already in use on {port}")
b.torque(old, 0)
b.write(old, 55, [0])          # unlock EEPROM
b.write(old, 5, [new])         # ID register
b.write(new, 55, [1])          # lock EEPROM
ok = sum(b.ping(new) for _ in range(30))
print(f"{port}: ID {old} -> {new}, new ID ping {ok}/30, old ID still answers: {b.ping(old)}")
print("bus now:", [i for i in range(0, 254) if b.ping(i)])
