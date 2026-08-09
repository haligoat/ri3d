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

**All tunable values live in `OdomConstants.h`.** Calibration prints lines in
exactly that file's format; replace the matching line and reflash. You should
not need to open `Odometry.cpp` to tune anything.

## What it can and cannot do

EchoLib has **no encoder support** — `Motor` is open-loop MCPWM, so there is no
wheel feedback anywhere in the library. The BMI270 is a 6-axis IMU with **no
magnetometer**, so there is no absolute heading either. Classic mecanum wheel
odometry is therefore off the table.

What we have instead:

| Source | Good at | Fails when |
| --- | --- | --- |
| Gyro Z | heading, excellent short-term | drifts slowly; no absolute north |
| Accelerometer | detecting *changes* in motion | double-integrates into garbage alone |
| Commanded `(x, y, turn)` | predicting velocity while wheels grip | slip, stall, collision, low battery |

The accelerometer and the command model fail in opposite situations, which is
what the Kalman filter exploits: **predict with the accelerometer, correct with
the command model**, and snap velocity to zero whenever the robot is provably
parked (a zero-velocity update, or ZUPT).

Heading is *not* a filter state — it comes straight from the gyro as a known
rotation. That keeps the filter linear (an LTV Kalman filter, not an EKF), which
means no Jacobians to get wrong.

### Measured accuracy (simulation)

| Scenario | Error |
| --- | --- |
| Parked 10 s | 1 mm |
| 4 m straight | 0.6% |
| 2.3 m strafe | 0.2% |
| Turn 90° then drive 3.3 m | 0.3% |
| Combined command >100% (clamping) | 0.03% |
| Jammed against a wall at full throttle, 3 s | 5 cm |
| **Sustained 25% wheel slip** | **31%** |

That last row is not a tuning failure — it is a hard limit. An accelerometer
measures only *changes* in velocity, so a robot steadily doing 75% of its
commanded speed produces **no acceleration signature at all**. Loosening the
model trust by 10× only moved it from 31% to 26%. Don't burn time tuning it;
drive within traction instead.

Two related limits worth knowing:

- **Heading drifts.** A few degrees per minute after gyro bias is learned.
  Position error grows with it. Re-zero against a known field reference with
  `setPose()` when you get the chance.
- **Position is dead reckoning.** Error accumulates monotonically. It is good
  for "where am I relative to where auto started", not for a whole match.

## Setup

Already wired into `controller.ino`. The important part is that **every**
`driver.drive(x, y, turn)` is matched by `odom.setCommand(x, y, turn)` — the
motion model is built from those commands, and a stale one makes the filter
predict motion the robot isn't making.

```cpp
IMU imu;
MecanumOdometry odom(imu);

void setup() {
  imu.begin();          // must be still + level: calibrates the gyro
  imu.calibrateAccel(); // must be still + level
  odom.begin();         // latches current heading as theta = 0
}

void loop() {
  driver.drive(x, y, turn);
  odom.setCommand(x, y, turn);   // keep these two together
  odom.update();                 // self-limits to 100 Hz
}
```

Reading it:

```cpp
odom.getX();            // meters, field frame
odom.getY();
odom.getThetaDeg();     // degrees, CCW positive, continuous (not wrapped)
odom.getSpeed();        // m/s
odom.isStationary();    // ZUPT engaged
odom.getSlipEstimate(); // m/s of model-vs-accelerometer disagreement
odom.setPose(x, y, thetaRad);  // re-zero from a known reference
```

### Frames

- **Body:** `+forward` is the robot's nose, `+right` its right side — matching
  `MecanumDrive::drive(x, y, turn)` where `y` = forward and `x` = right.
- **Field:** fixed at `begin()`. `theta = 0` means the nose points along field
  `+X`; theta increases counter-clockwise, so field `+Y` is to the robot's left.

## Calibration — do this before trusting any number

The values in `OdomConstants.h` are **placeholders, not measurements**.
Set `ODOM_CALIBRATION_MODE` to `1` in `controller.ino`, flash,
and open Serial at 115200 for a guided menu. Each `>>>` line it prints is a
drop-in replacement for a line in `OdomConstants.h`.

> The robot **drives itself** in tests 2–4. Clear floor, hand near the switch.

**1 — IMU axis mapping.** Sets `ODOM_FORWARD_AXIS` / `ODOM_FORWARD_SIGN` /
`ODOM_RIGHT_AXIS` / `ODOM_RIGHT_SIGN` / `ODOM_YAW_SIGN`. Do this first; every other test depends on it. A wrong
sign here makes the estimate run backwards, which looks like a broken filter but
is really a mounting question. It also warns if the IMU isn't mounted flat —
the gravity compensation assumes its Z axis is vertical.

It asks you to **lift** the front, then the right side. Lift, don't tilt down:
an accelerometer at rest reads the *up-vector*, not gravity's direction (level,
it reads +9.8 on Z — that's why `calibrateAccel` subtracts g from Z and nothing
from X/Y). Lifting an edge makes the axis pointing out of it read positive;
tilting down inverts every sign.

Results apply in RAM immediately, so tests 2–5 work in the same session. They
do **not** survive a reboot — paste the `>>>` lines into `OdomConstants.h` and
reflash.

**2 / 3 — Top speed.** Drives at 100% for 1.5 s; you measure the distance.
Sets `ODOM_MAX_FORWARD_SPEED`, `ODOM_MAX_STRAFE_SPEED`, and estimates
`ODOM_MOTOR_TAU`.

Cross-check against the motor spec with the helper in `OdomConstants.h`:

```cpp
odomWheelSpeedFromRpm(outputRpm);   // 67 mm wheels: 1 rev = 0.2105 m
```

Pass geared output RPM (free speed ÷ gear ratio) and derate ~0.75 for load.
If measured and computed disagree by more than ~25%, suspect a gear ratio, a
backwards motor, or a sagging battery before trusting the tape measure.

Expect strafe to come out at roughly 0.5–0.8× forward — mecanum rollers waste a
lot of force sideways.

**4 — Deadband.** Ramps the command until the robot actually moves. Sets
`ODOM_DEADBAND_PCT`.

**5 — Drift check.** Leaves the robot parked for 30 s. A tuned filter stays
under a few centimetres. If it reports that ZUPT never engaged, raise
`ODOM_ZUPT_ACCEL_THRESH` / `ODOM_ZUPT_YAW_RATE_THRESH` — a vibrating chassis can
sit above the defaults.

**6 — Live pose stream.** Push the robot by hand and watch the estimate follow.

## Tuning

Only after calibration, and change one thing at a time. Every knob below is a
constant in `OdomConstants.h`.

| Symptom | Knob |
| --- | --- |
| Drifts while parked | ZUPT isn't engaging — raise `ODOM_ZUPT_ACCEL_THRESH`, `ODOM_ZUPT_YAW_RATE_THRESH` |
| Estimate lags real motion | lower `ODOM_MOTOR_TAU` |
| Consistently over/under-shoots distance | fix `ODOM_MAX_FORWARD_SPEED` (a scale error, not a filter problem) |
| Overshoots after hitting things | lower `ODOM_SLIP_SCALE` (quicker to distrust the wheels) |
| Jitters on a rough floor | raise `ODOM_ACCEL_NOISE` |
| Position creeps during long pushes | lower `ODOM_SLIP_MEMORY` |

`ODOM_ZUPT_VEL_THRESH` is what lets ZUPT fire while the driver is still commanding
full throttle — that's the wall-jam case. Raising it too far will make a
slow-moving robot falsely believe it is parked.

## Driver station

`wifi_control.py` shows live pose on one line while you drive. The robot sends
`ODOM,x,y,thetaDeg,vx,vy,stationary` every 100 ms.

It also now **re-sends the held command at 20 Hz**. `WiFiServerBridge` drops the
connection and hard-stops the motors after 2 s of silence, so the previous
send-once-on-keypress behaviour stalled the robot whenever you held a key for
more than two seconds. The odometry model needs the current command anyway.
