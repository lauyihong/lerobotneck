"""List the servo IDs (1-20) on each bus, with status, position limits and homing offset. Read-only.

    python tools/scan_motors.py                  # COM3 COM4 (reference PC)
    python tools/scan_motors.py /dev/ttyACM0     # any ports
"""
import sys

from sts import Bus

for port in sys.argv[1:] or ["COM3", "COM4"]:
    b=Bus(port); ids=[i for i in range(1,21) if b.ping(i)]
    print(port, ids)
    for i in ids:
        lim=(b.r16(i,9),b.r16(i,11)); off=b.r16(i,31)
        print("  id",i,b.status(i),"limits",lim,"offset",off)
