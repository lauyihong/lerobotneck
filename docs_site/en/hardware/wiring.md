# Wiring and motor IDs

Two Feetech serial buses at 1 Mbps, one USB adapter each.

| bus | servos | IDs |
|---|---|---|
| bus 1 (`port_left`) | left arm + neck | 1–6 = left arm (shoulder_pan … gripper), **7–12 = neck J1–J6** |
| bus 2 (`port_right`) | right arm | 1–6 = right arm |

The plugin's defaults are `COM3` / `COM4`, as on the reference Windows PC. On Linux these are usually
`/dev/ttyACM0` / `/dev/ttyACM1`; pass them as `--robot.port_left=… --robot.port_right=…`.

## Setting IDs

New STS3215 servos all have ID 1. Connect **one servo at a time** and give it its ID:

```bash
python tools/scan_motors.py COM3            # list the IDs on a bus (Linux: /dev/ttyACM0)
python tools/set_motor_id.py COM3 1 7       # port, old ID, new ID
```

`set_motor_id.py` refuses to act if the old ID is missing or duplicated, or if the new ID is already in use. Label
each servo as you go, for example `L1–L6`, `N7–N12` and `R1–R6`.

## Things that bite

- **Only one program can open a serial port at a time.** "Access is denied" or "port busy" means another script or
  viewer still has it open.
- **Broadcast ping loses replies with 12 servos on one bus.** The tools ping each ID individually.
- **Wrist camera indices** can change after re-plugging. `projects/sim_mirror/scripts/probe_cams.py` grabs one frame
  from every OpenCV index so you can tell them apart. On Windows the D435i also appears as an OpenCV camera. Don't
  open it there while the RealSense SDK is streaming.
- Some servo batches ship with different EEPROM defaults (phase, max temperature, max voltage). If one servo reports
  voltage errors on every packet, compare its `Max_Voltage_Limit` with the others.
