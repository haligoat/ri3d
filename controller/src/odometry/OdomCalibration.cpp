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
            default: Serial.println(F("Unknown option.")); break;
        }
        drive.drive(0, 0, 0);
    }
}
