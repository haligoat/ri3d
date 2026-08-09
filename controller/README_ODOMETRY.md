# Odometry

Pose tracking (x, y, heading) for the mecanum robot.

```
controller/
  controller.ino              robot code; ODOM_CALIBRATION_MODE lives here
  src/odometry/
    OdomConstants.h           <-- the only file you edit
    Odometry.h / .cpp         the Kalman filter
    OdomCalibration.h / .cpp  the guided calibration menu
```

The folder must be named `src/` — Arduino only compiles extra sources from the
sketch root or a folder with that exact name.

## How it works

EchoLib has no encoder support (`Motor` is open-loop MCPWM) and the BMI270 has
no magnetometer, so wheel odometry and absolute heading are both off the table.
What's left:

- **Gyro Z** → heading. Good; drifts a few °/min.
- **Accelerometer** → sees *changes* in motion. Alone, double-integrates into garbage.
- **Commanded `(x, y, turn)`** → predicts velocity while the wheels grip; lies
  during slip, stall, or collision.

The last two fail in opposite situations, so the filter predicts with the
accelerometer, corrects with the command model, and snaps velocity to zero when
the robot is provably parked (a zero-velocity update, ZUPT). Heading is not a
filter state — it comes straight from the gyro, which keeps the filter linear.

## Limits — read before trusting a number

| Scenario | Error |
| --- | --- |
| Parked 10 s | 1 mm |
| 4 m straight / strafe / turn-then-drive | 0.2–0.6% |
| Combined command >100% (clamping) | 0.03% |
| Jammed on a wall at full throttle | **unreliable: 0.1 m or 3.8 m** |
| Sustained 25% wheel slip | **31%** |

**Slip is not fixable here.** An accelerometer sees only *changes* in velocity,
so a robot steadily doing 75% of commanded speed has no acceleration signature
at all. Loosening model trust 10× moved it 31%→26%. Don't tune it — drive within
traction.

**Jam detection is a coin flip.** Across 12 noise seeds it recovered on 3 and ran
away on 9. Don't rely on the pose surviving a collision.

**Position is dead reckoning** — error only accumulates. Good for "where am I
relative to where auto started", not a whole match. Re-zero with `setPose()`
against a known reference when you can.

### Is the filter worth it?

Against a 15-line estimator that just integrates the commanded velocity using
the gyro for heading:

| Scenario | Simple | Kalman |
| --- | --- | --- |
| straight / strafe / turn / clamping | 0.0% | 0.1–0.2% |
| stop-go | 0.0% | 1.1% |
| 25% slip | 33.3% | 31.8% |
| wall jam | 91.8% | 79.0% |

They tie everywhere that matters. (Simple scores 0.0% partly because the
simulation's truth model *is* the command model — on a real robot both are
limited by how well `ODOM_MAX_FORWARD_SPEED` matches reality.) The filter's only
real edge is collision handling, and that edge is unreliable. **If you want less
to maintain, the simple approach gives up almost nothing.**

## Usage

Already wired into `controller.ino`. One rule: **every `driver.drive()` must be
matched by `odom.setCommand()` with the same arguments** — the motion model is
built from those commands, and a stale one makes the filter predict motion that
isn't happening.

```cpp
IMU imu;
MecanumOdometry odom(imu);

void setup() {
  imu.begin();          // still + level: calibrates the gyro
  imu.calibrateAccel(); // still + level
  odom.begin();         // latches current heading as theta = 0
}

void loop() {
  driver.drive(x, y, turn);
  odom.setCommand(x, y, turn);   // keep these together
  odom.update();                 // self-limits to 100 Hz
}
```

```cpp
odom.getX(); odom.getY();   // meters, field frame
odom.getThetaDeg();         // CCW positive, continuous (not wrapped)
odom.getSpeed();            // m/s
odom.isStationary();        // ZUPT engaged
odom.getSlipEstimate();     // m/s of model-vs-accelerometer disagreement
odom.setPose(x, y, thetaRad);
```

**Frames.** Body: `+forward` is the nose, `+right` its right side — matching
`MecanumDrive::drive(x, y, turn)` where `y` = forward, `x` = right. Field: fixed
at `begin()`; `theta = 0` means the nose points along field `+X`, and theta
increases counter-clockwise, so field `+Y` is robot-left.

## IMU mounting

Defaults assume the IMU's **+Y points forward, +X points right**, mounted
**flat** (Z vertical). Flat matters — the gravity compensation assumes it, and
no tuning fixes a sideways board.

If the board is rotated in yaw, only the axis mapping changes:

| Mount | FORWARD_AXIS | FORWARD_SIGN | RIGHT_AXIS | RIGHT_SIGN |
| --- | --- | --- | --- | --- |
| 0° (default) | 1 (Y) | +1 | 0 (X) | +1 |
| 90° CCW (from above) | 0 (X) | +1 | 1 (Y) | −1 |
| 90° CW | 0 (X) | −1 | 1 (Y) | +1 |
| 180° | 1 (Y) | −1 | 0 (X) | −1 |

Rotating the board 90° CCW swings its +Y from forward to *left*, and its +X from
right to *forward* — so forward becomes X+, and robot-right becomes −Y.

`ODOM_YAW_SIGN` does **not** change: gyro Z is still vertical, and rotating a
sensor about the axis it measures doesn't reverse that rotation's sense. Nothing
else changes either — speeds, deadband and filter tuning are drivetrain
properties. You also don't need to cancel the constant heading offset;
`odom.begin()` latches whatever it reads at boot as θ = 0.

Prefer calibration test 1 over this table — it measures the same four values
empirically, and this is easy to get wrong by reasoning.

## Calibration — do this before trusting any number

Values in `OdomConstants.h` are **placeholders, not measurements**. Set
`ODOM_CALIBRATION_MODE` to `1` in `controller.ino`, flash, open Serial at
115200. Each `>>>` line it prints is a drop-in replacement for a line in
`OdomConstants.h`.

> The robot **drives itself** in tests 2–4. Clear floor, hand near the switch.

**1 — Axis mapping.** Sets the five mounting constants. Do this first;
everything depends on it. A wrong sign makes position run backwards, which looks
like a broken filter but is a mounting question.

It asks you to **lift** the front, then the right side. Lift, don't tilt down: an
accelerometer at rest reads the *up-vector*, not gravity's direction (level, it
reads +9.8 on Z). Lifting an edge makes the axis pointing out of it read
positive; tilting down inverts every sign.

Results apply in RAM immediately so tests 2–5 work in the same session, but do
**not** survive a reboot — paste them in and reflash.

**2 / 3 — Top speed.** Drives at 100% for 1.5 s; you measure the distance. Sets
`ODOM_MAX_FORWARD_SPEED`, `ODOM_MAX_STRAFE_SPEED`, estimates `ODOM_MOTOR_TAU`.
Expect strafe at 0.5–0.8× forward. Cross-check with:

```cpp
odomWheelSpeedFromRpm(outputRpm);   // 67 mm wheels: 1 rev = 0.2105 m
```

Pass geared output RPM, derate ~0.75 for load. Disagreement over ~25% means a
gear ratio, backwards motor, or sagging battery — not a bad tape measure.

**4 — Deadband.** Ramps until the robot moves. Sets `ODOM_DEADBAND_PCT`.

**5 — Drift check.** Parked 30 s; a tuned filter stays under a few cm. If it says
ZUPT never engaged, raise `ODOM_ZUPT_ACCEL_THRESH` / `ODOM_ZUPT_YAW_RATE_THRESH`.

**6 — Live pose.** Push it by hand and watch the estimate follow.

## Tuning

Only after calibration, one change at a time. All in `OdomConstants.h`.

| Symptom | Knob |
| --- | --- |
| Drifts while parked | ZUPT isn't engaging — raise `ODOM_ZUPT_ACCEL_THRESH`, `ODOM_ZUPT_YAW_RATE_THRESH` |
| Estimate lags real motion | lower `ODOM_MOTOR_TAU` |
| Over/under-shoots distance | fix `ODOM_MAX_FORWARD_SPEED` (a scale error, not a filter problem) |
| Overshoots after hitting things | lower `ODOM_SLIP_SCALE` |
| Jitters on rough floor | raise `ODOM_ACCEL_NOISE` |
| Creeps during long pushes | lower `ODOM_SLIP_MEMORY` |

## Driver station

`wifi_control.py` shows live pose while you drive; the robot sends
`ODOM,x,y,thetaDeg,vx,vy,stationary` every 100 ms.

It re-sends the held command at 20 Hz. `WiFiServerBridge` drops the connection
and hard-stops the motors after 2 s of silence, so the old
send-once-on-keypress behaviour stalled the robot whenever you held a key
longer than that.
