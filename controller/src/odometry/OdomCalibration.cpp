#include "OdomCalibration.h"

static void waitForEnter(const __FlashStringHelper* prompt) {
    Serial.println();
    Serial.println(prompt);
    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();
}

// ---------------------------------------------------------------------------
//  Test 1 -- IMU axis mapping and signs
// ---------------------------------------------------------------------------
static void testHeading(IMU& imu, MecanumOdometry& odom) {
    Serial.println(F("\n=== TEST 1: heading ==="));
    Serial.println(F("Put the robot flat on the floor and keep it still."));
    waitForEnter(F("Press ENTER when still..."));

    // The IMU must be flat for gyro Z to be the yaw axis. Nothing downstream
    // can compensate for a board mounted on its side.
    float az = 0;
    for (int i = 0; i < 100; i++) { az += imu.getAccelZms(); delay(5); }
    az /= 100;
    Serial.printf("Z at rest: %+.2f m/s^2\n", az);
    if (fabsf(az) < 8.0f)
        Serial.println(F("!! Z is not ~9.8 -- the IMU is NOT mounted flat. Remount it level."));
    else if (az < 0)
        Serial.println(F("!! Z is negative -- the IMU is upside down."));
    else
        Serial.println(F("OK: flat and right side up."));

    Serial.println(F("\nNow rotate the robot COUNTER-CLOCKWISE (to its left) about 90 deg."));
    float y0 = imu.getGyroZdeg();
    waitForEnter(F("Press ENTER after rotating..."));
    float d = imu.getGyroZdeg() - y0;
    if (d >  180) d -= 360;
    if (d < -180) d += 360;

    int ySign = (d > 0) ? 1 : -1;
    Serial.printf("Gyro Z changed by %+.1f deg\n", d);
    Serial.printf("\n>>> constexpr int8_t ODOM_YAW_SIGN = %+d;\n", ySign);
    if (fabsf(d) < 30.0f)
        Serial.println(F("!! Barely moved -- rotate a clear 90 deg and redo."));

    odom.config().yawSign = (int8_t)ySign;
    odom.begin();
    Serial.println(F("\nApplied for this session. Paste the >>> line into"
                     "\nOdomConstants.h and reflash to make it permanent."));
}

// ---------------------------------------------------------------------------
//  Test 2/3 -- top speed, and a first-order estimate of the motor time constant
// ---------------------------------------------------------------------------
static void testSpeed(MecanumDrive& drive, bool strafe) {
    Serial.printf("\n=== TEST %d: max %s speed ===\n", strafe ? 3 : 2,
                  strafe ? "STRAFE" : "FORWARD");
    Serial.println(F("Put the robot on the floor with 4+ meters of clear space."));
    Serial.println(F("Mark the starting position of a fixed point on the chassis."));
    Serial.println(F("It will drive at 100% for 1.5 s, then coast to a stop."));
    waitForEnter(F("Press ENTER to run (or reposition first)..."));

    Serial.println(F("3...")); delay(1000);
    Serial.println(F("2...")); delay(1000);
    Serial.println(F("1...")); delay(1000);

    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
        if (strafe) drive.drive(100, 0, 0); else drive.drive(0, 100, 0);
        delay(5);
    }
    drive.drive(0, 0, 0);

    Serial.println(F("\nDone. Measure the distance travelled WHILE POWERED"));
    Serial.println(F("(ignore the coast -- mark where it started slowing)."));
    Serial.println(F("Enter the powered distance in METERS, then ENTER:"));

    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    float dist = Serial.readStringUntil('\n').toFloat();
    if (dist <= 0.0f) { Serial.println(F("Bad input, skipping.")); return; }

    // The robot spent roughly one time constant getting up to speed, so a plain
    // distance/time underestimates the top speed. Invert the first-order
    // response instead: d = v * (T - tau * (1 - e^(-T/tau))).
    const float T = 1.5f;
    float tau = ODOM_MOTOR_TAU;
    float vMax = dist / (T - tau * (1.0f - expf(-T / tau)));

    Serial.printf("\nAssuming ODOM_MOTOR_TAU = %.2f s\n", tau);
    if (strafe) Serial.printf(">>> constexpr float ODOM_MAX_STRAFE_SPEED  = %.2ff;\n", vMax);
    else        Serial.printf(">>> constexpr float ODOM_MAX_FORWARD_SPEED = %.2ff;\n", vMax);
    Serial.println(F("\nIf the robot visibly takes longer than ~0.25 s to get up to"
                     "\nspeed, raise ODOM_MOTOR_TAU and run this again."));
}

// ---------------------------------------------------------------------------
//  Test 4 -- command deadband
// ---------------------------------------------------------------------------
static void testDeadband(MecanumDrive& drive) {
    Serial.println(F("\n=== TEST 4: deadband ==="));
    Serial.println(F("Ramping forward command until the robot actually moves."));
    Serial.println(F("Watch it -- note the printed value at first movement."));
    waitForEnter(F("Press ENTER to start ramping..."));

    for (int pct = 2; pct <= 60; pct += 2) {
        drive.drive(0, pct, 0);
        Serial.printf("  command = %d%%\n", pct);
        delay(700);
    }
    drive.drive(0, 0, 0);
    Serial.println(F("\nEnter the percent at which it FIRST moved, then ENTER:"));
    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    String s = Serial.readStringUntil('\n');
    float db = s.toFloat();
    if (db > 0) Serial.printf("\n>>> constexpr float ODOM_DEADBAND_PCT = %.1ff;\n", db);
}

// ---------------------------------------------------------------------------
//  Test 6 -- deadband, measured instead of eyeballed
// ---------------------------------------------------------------------------
//  Test 4 asks a human to spot the moment the robot creeps forward, which is
//  the least repeatable part of the whole calibration -- observers routinely
//  disagree by several percent, and that error feeds into every estimate.
//
//  This measures it from two different sensors:
//
//    TURN  -- ramps a spin and watches the GYRO. The trustworthy one: the gyro
//             measures rotation directly, so "is it moving" has an unambiguous
//             answer well clear of the noise floor.
//    DRIVE -- ramps forward and watches the ACCELEROMETER for the launch
//             transient. Noisier, and only observable while speeding up, which
//             is why every step starts from rest.
//
//  The accelerometer is otherwise unused by the odometry, for reasons set out
//  in Odometry.h. Detecting a threshold crossing is a different job from
//  integrating for position, so using it here does not contradict that.

// Sampled at rest to set detection thresholds, so a noisy IMU or a vibrating
// chassis raises the bar instead of producing false positives.
struct NoiseFloor {
    float gyroRate;   // deg/s
    float accel;      // m/s^2, horizontal magnitude
};

static float horizontalAccel(IMU& imu) {
    float ax = imu.getAccelXms();
    float ay = imu.getAccelYms();
    return sqrtf(ax * ax + ay * ay);
}

// Shortest signed gyro delta, handling the 0..360 wrap.
static float yawDelta(float from, float to) {
    float d = to - from;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

static NoiseFloor measureNoiseFloor(IMU& imu) {
    Serial.println(F("Sampling sensor noise at rest (2 s) -- do not touch the robot."));

    float maxRate = 0.0f, maxAcc = 0.0f, baseAcc = horizontalAccel(imu);
    float prevYaw = imu.getGyroZdeg();
    uint32_t prevMs = millis(), t0 = millis();

    while (millis() - t0 < 2000) {
        delay(20);
        uint32_t now = millis();
        float dt = (now - prevMs) / 1000.0f;
        if (dt <= 0.0f) continue;
        float yaw = imu.getGyroZdeg();
        float rate = fabsf(yawDelta(prevYaw, yaw)) / dt;
        prevYaw = yaw; prevMs = now;

        if (rate > maxRate) maxRate = rate;
        float acc = fabsf(horizontalAccel(imu) - baseAcc);
        if (acc > maxAcc) maxAcc = acc;
    }

    NoiseFloor n;
    // 3x the observed peak, with a floor so a suspiciously quiet sample cannot
    // produce a threshold that trips on nothing.
    n.gyroRate = fmaxf(maxRate * 3.0f, 6.0f);
    n.accel    = fmaxf(maxAcc  * 3.0f, 0.30f);
    Serial.printf("  noise: gyro %.1f deg/s, accel %.2f m/s^2 -> thresholds %.1f, %.2f\n",
                  maxRate, maxAcc, n.gyroRate, n.accel);
    return n;
}

// Ramps one axis and returns the first percent that produced motion, or -1.
static float rampUntilMotion(IMU& imu, MecanumDrive& drive,
                             const NoiseFloor& noise, bool turnAxis) {
    for (int pct = 2; pct <= 50; pct += 1) {
        float baseAcc = horizontalAccel(imu);
        uint32_t t0 = millis();
        float peakRate = 0.0f, peakAcc = 0.0f;
        float prevYaw = imu.getGyroZdeg();
        uint32_t prevMs = t0;

        while (millis() - t0 < 450) {
            if (turnAxis) drive.drive(0, 0, pct);
            else          drive.drive(0, pct, 0);
            delay(15);

            uint32_t now = millis();
            float dt = (now - prevMs) / 1000.0f;
            if (dt > 0.0f) {
                float yaw = imu.getGyroZdeg();
                float rate = fabsf(yawDelta(prevYaw, yaw)) / dt;
                if (rate > peakRate) peakRate = rate;
                prevYaw = yaw; prevMs = now;
            }
            float acc = fabsf(horizontalAccel(imu) - baseAcc);
            if (acc > peakAcc) peakAcc = acc;
        }

        drive.drive(0, 0, 0);
        delay(400);   // settle, so the next step starts from a standstill

        bool moved = turnAxis ? (peakRate > noise.gyroRate)
                              : (peakAcc  > noise.accel);

        Serial.printf("  %2d%%  gyro %5.1f deg/s  accel %4.2f m/s^2  %s\n",
                      pct, peakRate, peakAcc, moved ? "<-- MOVED" : "");

        if (moved) return (float)pct;
    }
    return -1.0f;
}

static float medianOf3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

static void testDeadbandAuto(IMU& imu, MecanumDrive& drive) {
    Serial.println(F("\n=== TEST 6: deadband (automatic) ==="));
    Serial.println(F("Robot on the FLOOR, wheels down, ~1 m clear all round."));
    Serial.println(F("It will creep forward and spin in place, repeatedly."));
    waitForEnter(F("Press ENTER to begin..."));

    NoiseFloor noise = measureNoiseFloor(imu);

    // Three passes per axis: static friction is not perfectly repeatable, and
    // one pass can land a percent or two off depending on where the wheels
    // happen to be sitting. The median rejects a single bad run.
    float turnRuns[3], driveRuns[3];

    Serial.println(F("\n-- TURN ramp (gyro) --"));
    for (int i = 0; i < 3; i++) {
        Serial.printf("pass %d:\n", i + 1);
        turnRuns[i] = rampUntilMotion(imu, drive, noise, true);
        if (turnRuns[i] < 0) { Serial.println(F("!! no motion by 50% -- check power and wiring.")); return; }
    }

    Serial.println(F("\n-- DRIVE ramp (accelerometer) --"));
    for (int i = 0; i < 3; i++) {
        Serial.printf("pass %d:\n", i + 1);
        driveRuns[i] = rampUntilMotion(imu, drive, noise, false);
        if (driveRuns[i] < 0) { Serial.println(F("!! no motion by 50% -- check power and wiring.")); return; }
    }

    float turnDb  = medianOf3(turnRuns[0],  turnRuns[1],  turnRuns[2]);
    float driveDb = medianOf3(driveRuns[0], driveRuns[1], driveRuns[2]);

    Serial.println(F("\n--- RESULTS ---"));
    Serial.printf("turn  passes: %.0f %.0f %.0f  -> median %.0f%%\n",
                  turnRuns[0], turnRuns[1], turnRuns[2], turnDb);
    Serial.printf("drive passes: %.0f %.0f %.0f  -> median %.0f%%\n",
                  driveRuns[0], driveRuns[1], driveRuns[2], driveDb);

    // ODOM_DEADBAND_PCT is about the drive axis, so that is what gets printed.
    // The turn figure is the better measurement though, so disagreement is
    // flagged rather than silently averaging two different quantities.
    Serial.printf("\n>>> constexpr float ODOM_DEADBAND_PCT = %.1ff;\n", driveDb);

    if (fabsf(turnDb - driveDb) > 4.0f) {
        Serial.println(F("\n!! The two measurements disagree by more than 4%."));
        Serial.println(F("!! The gyro (turn) figure is the more reliable of the two."));
        Serial.println(F("!! A much higher DRIVE number usually means the accelerometer"));
        Serial.println(F("!! missed a gentle launch -- consider using the turn value."));
        Serial.printf("!! turn-based alternative: %.1ff\n", turnDb);
    }

    Serial.println(F("\nPaste the >>> line into OdomConstants.h and reflash."));
}

// ---------------------------------------------------------------------------
//  Test 7 -- motor time constant (tau), measured from a step response
// ---------------------------------------------------------------------------
//  ODOM_MOTOR_TAU is the time to reach ~63% of a commanded step change. Test 2
//  currently ASSUMES this value in order to back out top speed, so a wrong tau
//  quietly biases the speed calibration too -- worth measuring rather than
//  guessing.
//
//  Measured on the TURN axis because the gyro reports rotation RATE directly.
//  That is exactly the quantity a first-order step response is defined over.
//  The drive axis has no such sensor: with no encoders, forward velocity would
//  have to be integrated from the accelerometer, and integrating that noise
//  over seconds is precisely what Odometry.h rejected for position.
//
//  CAVEAT, and it is a real one: this measures the ROTATIONAL time constant.
//  Rotational and translational inertia are not the same, so the drivetrain's
//  forward tau can differ. It is the same motors, gearing and mass, so it is a
//  well-founded estimate -- but treat it as a good starting value, then adjust
//  if the pose estimate visibly leads or lags real motion.
//
//  Both directions are measured. Rise and decay should be similar for a
//  first-order system; if they are not, the model is a poor fit for this
//  drivetrain and that is worth knowing.
#define TAU_TEST_PCT   60     // well clear of the deadband, short of top speed
#define TAU_RUN_MS     2500   // long enough to reach a steady rate
#define TAU_COAST_MS   2500
#define TAU_SAMPLE_MS  20

struct TauResult {
    float rise;    // seconds to 63.2% of steady-state rate
    float decay;   // seconds to fall to 36.8% after power is cut
    float steady;  // deg/s reached
    bool  ok;
};

static TauResult measureTauOnce(IMU& imu, MecanumDrive& drive) {
    TauResult r = {0, 0, 0, false};

    // --- spin up, recording the rate curve ---
    const int N = TAU_RUN_MS / TAU_SAMPLE_MS;
    static float rate[TAU_RUN_MS / TAU_SAMPLE_MS];
    float prevYaw = imu.getGyroZdeg();
    uint32_t prevMs = millis();
    float smooth = 0.0f;

    for (int i = 0; i < N; i++) {
        drive.drive(0, 0, TAU_TEST_PCT);
        delay(TAU_SAMPLE_MS);
        uint32_t now = millis();
        float dt = (now - prevMs) / 1000.0f;
        float yaw = imu.getGyroZdeg();
        float inst = (dt > 0.0f) ? fabsf(yawDelta(prevYaw, yaw)) / dt : 0.0f;
        prevYaw = yaw; prevMs = now;
        // Light smoothing: raw gyro differences are noisy enough that a single
        // spike could satisfy the 63% crossing far too early.
        smooth = (i == 0) ? inst : (0.7f * smooth + 0.3f * inst);
        rate[i] = smooth;
    }

    // Steady state = mean of the last 600 ms, by which point a sane drivetrain
    // has settled.
    int tailStart = N - (600 / TAU_SAMPLE_MS);
    if (tailStart < 1) tailStart = 1;
    float ss = 0.0f;
    for (int i = tailStart; i < N; i++) ss += rate[i];
    ss /= (N - tailStart);
    r.steady = ss;

    if (ss < 20.0f) {
        drive.drive(0, 0, 0);
        Serial.println(F("  !! barely rotating -- raise TAU_TEST_PCT or check power."));
        return r;
    }

    for (int i = 0; i < N; i++) {
        if (rate[i] >= 0.632f * ss) { r.rise = (i * TAU_SAMPLE_MS) / 1000.0f; break; }
    }

    // --- cut power and record the decay ---
    drive.drive(0, 0, 0);
    prevYaw = imu.getGyroZdeg();
    prevMs = millis();
    uint32_t t0 = millis();
    smooth = ss;
    r.decay = TAU_COAST_MS / 1000.0f;   // if it never falls, report the ceiling

    while (millis() - t0 < TAU_COAST_MS) {
        delay(TAU_SAMPLE_MS);
        uint32_t now = millis();
        float dt = (now - prevMs) / 1000.0f;
        float yaw = imu.getGyroZdeg();
        float inst = (dt > 0.0f) ? fabsf(yawDelta(prevYaw, yaw)) / dt : 0.0f;
        prevYaw = yaw; prevMs = now;
        smooth = 0.7f * smooth + 0.3f * inst;
        if (smooth <= 0.368f * ss) { r.decay = (now - t0) / 1000.0f; break; }
    }

    r.ok = true;
    return r;
}

static void testMotorTau(IMU& imu, MecanumDrive& drive) {
    Serial.println(F("\n=== TEST 7: motor time constant (tau) ==="));
    Serial.println(F("Robot on the FLOOR with ~1 m clear all round."));
    Serial.println(F("It will spin up in place, then coast, three times."));
    Serial.println(F("Do NOT hold or touch it -- contact changes the result."));
    waitForEnter(F("Press ENTER to begin..."));

    float rises[3], decays[3];

    for (int i = 0; i < 3; i++) {
        Serial.printf("\npass %d: spinning up at %d%%...\n", i + 1, TAU_TEST_PCT);
        TauResult r = measureTauOnce(imu, drive);
        if (!r.ok) { Serial.println(F("Aborting.")); drive.drive(0, 0, 0); return; }
        rises[i]  = r.rise;
        decays[i] = r.decay;
        Serial.printf("  steady %.0f deg/s | rise %.2f s | decay %.2f s\n",
                      r.steady, r.rise, r.decay);
        delay(1200);   // let it come fully to rest before the next pass
    }

    drive.drive(0, 0, 0);

    float riseTau  = medianOf3(rises[0],  rises[1],  rises[2]);
    float decayTau = medianOf3(decays[0], decays[1], decays[2]);

    Serial.println(F("\n--- RESULTS ---"));
    Serial.printf("rise  passes: %.2f %.2f %.2f -> median %.2f s\n",
                  rises[0], rises[1], rises[2], riseTau);
    Serial.printf("decay passes: %.2f %.2f %.2f -> median %.2f s\n",
                  decays[0], decays[1], decays[2], decayTau);

    // The constant is defined on the rising edge, so that is what is emitted.
    Serial.printf("\n>>> constexpr float ODOM_MOTOR_TAU = %.2ff;\n", riseTau);

    if (fabsf(riseTau - decayTau) > 0.5f * riseTau) {
        Serial.println(F("\n!! Rise and decay differ by more than 50%."));
        Serial.println(F("!! A first-order lag is a rough fit for this drivetrain."));
        Serial.println(F("!! Usually means friction dominates the coast-down"));
        Serial.println(F("!! (decay much faster than rise). The rise figure is the"));
        Serial.println(F("!! one the model wants; expect some error either way."));
    }

    Serial.println(F("\nNOTE: measured while ROTATING. Forward tau may differ."));
    Serial.println(F("Re-run TEST 2 afterwards -- it uses tau to compute top speed."));
    Serial.println(F("\nPaste the >>> line into OdomConstants.h and reflash."));
}

// ---------------------------------------------------------------------------
void runOdometryCalibration(IMU& imu, MecanumDrive& drive, MecanumOdometry& odom) {
    Serial.println(F("\n\n########################################"));
    Serial.println(F("#   ODOMETRY CALIBRATION MODE          #"));
    Serial.println(F("#   THE ROBOT WILL DRIVE ITSELF.       #"));
    Serial.println(F("########################################"));

    for (;;) {
        Serial.println(F("\n--- MENU ---"));
        Serial.println(F("  1  Heading: mounting check + yaw sign"));
        Serial.println(F("  2  Max forward speed"));
        Serial.println(F("  3  Max strafe speed"));
        Serial.println(F("  4  Deadband (manual, you watch it)"));
        Serial.println(F("  5  Live pose stream"));
        Serial.println(F("  6  Deadband (automatic, sensor-measured)"));
        Serial.println(F("  7  Motor time constant (tau)"));
        Serial.print(F("Choose: "));

        while (Serial.available()) Serial.read();
        while (!Serial.available()) delay(10);
        int choice = Serial.read() - '0';
        while (Serial.available()) Serial.read();
        Serial.println(choice);

        switch (choice) {
            case 1: testHeading(imu, odom); break;
            case 2: testSpeed(drive, false); break;
            case 3: testSpeed(drive, true); break;
            case 4: testDeadband(drive); break;
            case 5: {
                Serial.println(F("Streaming pose. Drive it by hand. Send any key to stop."));
                odom.reset();
                while (Serial.available()) Serial.read();
                uint32_t lastPrint = 0;
                while (!Serial.available()) {
                    odom.setCommand(0, 0, 0);
                    odom.update();
                    if (millis() - lastPrint > 200) {
                        lastPrint = millis();
                        Serial.println(odom.telemetry());
                    }
                }
                while (Serial.available()) Serial.read();
                break;
            }
            case 6: testDeadbandAuto(imu, drive); break;
            case 7: testMotorTau(imu, drive); break;
            default: Serial.println(F("Unknown option.")); break;
        }
        drive.drive(0, 0, 0);
    }
}
