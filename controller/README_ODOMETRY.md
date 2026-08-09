# Odometry

Pose tracking (x, y, heading) for the mecanum robot.

```
controller/
  controller.ino              robot code; ODOM_CALIBRATION_MODE lives here
  src/odometry/
    OdomConstants.h           <-- the only file you edit
    Odometry.h / .cpp         the estimator
    OdomCalibration.h / .cpp  the guided calibration menu
```

The folder must be named `src/` — Arduino only compiles extra sources from the
sketch root or a folder with that exact name.

## How it works

EchoLib has no encoder support (`Motor` is open-loop MCPWM) and the BMI270 has
no magnetometer, so wheel odometry and absolute heading are both off the table.
What's left is two things:

- **Heading** comes from the IMU's gyro, read every cycle at 100 Hz. This is the
  one genuinely good measurement in the system.
- **Position** comes from integrating the velocity we *asked* for — the
  commanded `(x, y, turn)` run through the mecanum kinematics and a first-order
  motor lag.

That's the whole estimator, about 170 lines.

> **The IMU is essential — half of it.** The BMI270 is two sensors in one chip.
> The **gyroscope** is the backbone of this estimator. The **accelerometer** is
> not used at all (outside a one-time mounting check in calibration). So "we
> dropped the accelerometer" is not "we dropped the IMU".
>
> Heading matters more than any other single number here, because an error in it
> rotates every metre you drive afterwards. It's also the only part of the
> estimate that *observes* the robot rather than assuming the motors did what
> they were told — if a wheel slips through a turn, heading still tracks.

A Kalman filter fusing the accelerometer was tried first and dropped; it's in
git history if you want it back. The reason is structural rather than a tuning
failure: an accelerometer only measures *changes* in velocity, while the errors
that dominate here — a mis-measured top speed, steady wheel slip — are
near-constant offsets that produce no acceleration signature at all. Adding
encoders would change that calculus completely.

## Limits — read before trusting a number

**Position is dead reckoning.** Error only accumulates; there is no absolute
reference to correct against. It answers "where am I relative to where I
started", over tens of seconds — not a whole match. Re-zero with `setPose()`
against a known reference (a wall, a field landmark) whenever you get the
chance.

**Wheel slip is invisible.** If the wheels spin without gripping, the estimate
keeps happily integrating the speed you asked for. Same if the robot is jammed
against a wall or another robot at full throttle, or is being pushed while
"parked" — the pose will drift badly and nothing will flag it.

**Heading drifts** a few degrees per minute, and position error grows with it.

**Everything scales with `ODOM_MAX_FORWARD_SPEED`.** If that's off by 20%, every
distance is off by 20%. Calibration matters more than anything else here.

## Usage

Already wired into `controller.ino`. One rule: **every `driver.drive()` must be
matched by `odom.setCommand()` with the same arguments.** The commands *are* the
position estimate, so a stale one means an invented position.

```cpp
IMU imu;
MecanumOdometry odom(imu);

void setup() {
  imu.begin();    // keep the robot still: this calibrates the gyro
  odom.begin();   // latches current heading as theta = 0
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
odom.isStationary();        // commanding no motion (not a sensor reading)
odom.setPose(x, y, thetaRad);
```

**Frames.** Body: `+forward` is the nose, `+right` its right side — matching
`MecanumDrive::drive(x, y, turn)` where `y` = forward, `x` = right. Field: fixed
at `begin()`; `theta = 0` means the nose points along field `+X`, and theta
increases counter-clockwise, so field `+Y` is robot-left.

## IMU mounting

Mount the IMU **flat**, so its Z axis is vertical and gyro Z is the yaw axis.
Nothing downstream can compensate for a board mounted on its side.

Rotation *in yaw* needs no correction at all — `odom.begin()` latches whatever
heading it reads at boot as θ = 0, which absorbs any fixed mounting angle. Since
the accelerometer is never read, the board's X/Y orientation doesn't matter
either. `ODOM_YAW_SIGN` is the only mounting-related constant, and calibration
test 1 measures it.

## Calibration — do this before trusting any number

The values in `OdomConstants.h` are **placeholders, not measurements**. Set
`ODOM_CALIBRATION_MODE` to `1` in `controller.ino`, flash, open Serial at
115200. Each `>>>` line it prints is a drop-in replacement for a line in
`OdomConstants.h`.

> The robot **drives itself** in tests 2–4. Clear floor, hand near the switch.

**1 — Heading.** Confirms the IMU is flat and right side up, then has you rotate
the robot 90° counter-clockwise to determine `ODOM_YAW_SIGN`. Applies in RAM
immediately, but doesn't survive a reboot — paste it in and reflash.

**2 / 3 — Top speed.** Drives at 100% for 1.5 s; you measure the powered
distance. Sets `ODOM_MAX_FORWARD_SPEED` and `ODOM_MAX_STRAFE_SPEED`. Expect
strafe at 0.5–0.8× forward. Cross-check with:

```cpp
odomWheelSpeedFromRpm(outputRpm);   // 67 mm wheels: 1 rev = 0.2105 m
```

Pass geared output RPM and derate ~0.75 for load. Disagreement over ~25% means a
gear ratio, backwards motor, or sagging battery — not a bad tape measure.

**4 — Deadband.** Ramps the command until the robot moves. Sets
`ODOM_DEADBAND_PCT`.

**5 — Live pose.** Push the robot by hand and watch the estimate follow. Note
that pushing it is exactly the case the estimator can't see, so this only
confirms heading and signs, not distances.

## Autonomous

`AutoDrive` gives you blocking movement primitives. **Meters, degrees, percent**
— the same units the odometry reports, so there's only one unit system to keep
straight.

```cpp
AutoDrive auto_(driver, odom);

auto_.setAbortCheck(autoAbort);   // do this before anything moves -- see below

odom.reset();                     // this pose is now the origin
auto_.driveDistance(1.0, 40);     // 1.0 m forward at 40%, holding heading
auto_.turnToAngle(90, 35);        // face 90 deg
auto_.driveDistance(-0.5);        // half a metre backwards
auto_.strafeDistance(0.3);        // 0.3 m to the right
auto_.turnBy(45);                 // relative turn
```

Every call returns `true` only if it reached the target. **Check it** — a `false`
means timeout, abort, or a runaway, and the pose is not where you asked:

```cpp
if (!auto_.driveDistance(1.0)) return;   // don't run the rest from a bad pose
```

Angles are **absolute field headings**, where 0 is whichever way the robot faced
at `odom.begin()`. Absolute rather than relative so errors don't compound: three
`turnToAngle(90)` calls all leave you at 90°, whereas three `turnBy(90)` calls
accumulate whatever each one got wrong.

### The abort hook is not optional

These calls block, so `loop()` is not running while a move executes — nothing
services the network or your kill switch unless you provide it:

```cpp
static bool autoAbort() {
  server.processIncoming();
  if (!server.getStatus()) return true;              // link dropped
  String d = server.readData();
  if (d.length() == 1 && d.equals("x")) return true; // kill switch
  return false;
}
```

Without it, a robot that starts a 3-second move ignores your stop command for
all three seconds.

### If the robot turns the wrong way

`AUTO_TURN_SIGN` says which sign of `turn` rotates counter-clockwise. The
default is derived from EchoLib's wheel mixing, but that assumes your motors are
wired and numbered the way the constructor claims — so it's a guess.

You don't have to reason it out. A turn heading away from its target is detected
and aborted, and the serial output tells you to flip the constant.

### What to expect

**Turns are the accurate half** — closed-loop on the gyro. **Distances are dead
reckoning**, only as good as `ODOM_MAX_FORWARD_SPEED` and only true while the
wheels grip. So:

- Keep drive segments short and re-square against a wall between them,
  using `odom.setPose()` to re-zero.
- Drive gently. Slip is invisible to the estimator and worst under hard
  acceleration; half speed usually finishes closer *and* sooner because you
  don't overshoot.
- A blocked robot will report a move "complete" having gone nowhere. There is
  no sensor that can tell you otherwise.

## Tuning

There are only four things to get right, all in `OdomConstants.h`.

| Symptom | Fix |
| --- | --- |
| Turns the wrong way | flip `ODOM_YAW_SIGN` |
| Every distance off by the same ratio | fix `ODOM_MAX_FORWARD_SPEED` |
| Strafe distances off, forward fine | fix `ODOM_MAX_STRAFE_SPEED` |
| Estimate lags real motion, overshoots on stops | lower `ODOM_MOTOR_TAU` |
| Creeps while sitting at a small command | raise `ODOM_DEADBAND_PCT` |

For autos:

| Symptom | Fix |
| --- | --- |
| Turns spin away from the target | flip `AUTO_TURN_SIGN` |
| Wanders off heading while driving | raise `AUTO_HEADING_KP` |
| Oscillates around a heading | raise `AUTO_HEADING_KD`, or lower `AUTO_HEADING_KP` |
| Overshoots the distance | lower `AUTO_DISTANCE_KP`, or drive slower |
| Stalls just short and sits buzzing | raise `AUTO_MIN_DRIVE_PCT` / `AUTO_MIN_TURN_PCT` |
| Declares done while still coasting | raise `AUTO_SETTLE_MS` |

## Driver station

`wifi_control.py` shows live pose while you drive; the robot sends
`ODOM,x,y,thetaDeg,vx,vy,stationary` every 100 ms.

It re-sends the held command at 20 Hz. `WiFiServerBridge` drops the connection
and hard-stops the motors after 2 s of silence, so the old
send-once-on-keypress behaviour stalled the robot whenever you held a key
longer than that.
