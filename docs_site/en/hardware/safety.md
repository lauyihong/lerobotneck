# Safety rules

These rules come from bringing up the reference robot. Most of them are cheap to follow and expensive to learn.

## Before torque
- **At power-up, `Goal_Position`, `Goal_Speed` and `Acceleration` read 0.** Enabling torque then means "go to tick 0
  at full speed". Write goal = present position, a speed cap and an acceleration first, read them back, and only
  then enable torque.
- **Writing `Goal_Position` switches torque on by itself**, even when goal = present. Write the speed cap,
  acceleration and torque limit first, and the goal last.
- After enabling torque, hold still for about a second and check that no joint drifts. If one does, release.

## While moving
- **J2 and J3 (neck and arms) rest on mechanical stops** in the folded pose, beyond their calibrated limits. Never
  command them further into the stop. Return to the clamped `target_ticks` of the `home` pose instead.
- Keep `max_relative_target` set (plugin option) when you run a new policy or script.
- Use slow speeds for large motions (the reference scripts use 8–15 °/s).
- Freeze on tracking errors or overload. If a joint still moves after the freeze, release torque.

## At the end
- **Always release torque.** `robot.disconnect()` does it. The neck demo ramps the torque limit down over 3 s so
  that J2 and J3 settle gently onto their stops.

## Known open issues on the reference robot
- One neck servo (J6) drives to a wrong target when torque is enabled, so it is kept limp. The cause is not known
  yet.
- One neck servo (J4) reports `Present_Position` + 4096 after reassembly. Readers use the value mod 4096, and the
  joint is kept limp until it is understood.
- The arms have not been driven yet. See the encoder wrap warning in [](calibration.md).
