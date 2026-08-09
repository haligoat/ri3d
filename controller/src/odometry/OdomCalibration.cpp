#include "OdomCalibration.h"

static void waitForEnter(const char* prompt) {
    Serial.println();
    Serial.println(prompt);
    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();
}

// ---------------------------------------------------------------------------
//  Test 1 -- IMU axis mapping and signs
// ---------------------------------------------------------------------------
// Find which accelerometer axis swings hardest while the robot is held tilted,
// and with what sign.
//
// An accelerometer at rest reads the UP-vector, not gravity's direction: level,
// it reports +9.8 on Z (which is why EchoLib's calibrateAccel subtracts g from
// the Z reading and nothing from X/Y). So LIFTING an edge makes the axis
// pointing out of that edge read POSITIVE. Tilting it down would invert every
// sign here, which silently makes the whole odometry estimate run backwards.
static void findAxis(IMU& imu, const char* instruction, int& axis, int& sign) {
    Serial.println();
    Serial.println(instruction);
    Serial.println(F("Hold it there. Watching for 4 seconds..."));
    delay(1500);

    float peak = 0;
    axis = -1;
    sign = 1;
    uint32_t t0 = millis();
    while (millis() - t0 < 4000) {
        float v[3] = { imu.getAccelXms(), imu.getAccelYms(), imu.getAccelZms() };
        for (int i = 0; i < 2; i++) {           // only X and Y can be horizontal
            if (fabsf(v[i]) > fabsf(peak)) { peak = v[i]; axis = i; sign = (v[i] > 0) ? 1 : -1; }
        }
        delay(10);
    }
    Serial.printf("Largest swing: axis %c, value %+.2f m/s^2\n", 'X' + axis, peak);
    if (fabsf(peak) < 3.0f)
        Serial.println(F("!! Weak signal -- tilt it further (aim for 30-45 deg) and redo."));
}

static void testAxes(IMU& imu, MecanumOdometry& odom) {
    Serial.println(F("\n=== TEST 1: IMU axis mapping ==="));
    Serial.println(F("Keep the robot ON THE FLOOR and STILL first."));
    waitForEnter(F("Press ENTER when still..."));

    float ax = 0, ay = 0, az = 0;
    for (int i = 0; i < 100; i++) {
        ax += imu.getAccelXms(); ay += imu.getAccelYms(); az += imu.getAccelZms();
        delay(5);
    }
    ax /= 100; ay /= 100; az /= 100;
    Serial.printf("At rest: X=%.2f  Y=%.2f  Z=%.2f m/s^2\n", ax, ay, az);
    if (fabsf(az) < 8.0f) {
        Serial.println(F("!! Z is not reading ~9.8. The IMU is NOT mounted flat."));
        Serial.println(F("!! Remount it level -- the gravity compensation assumes Z is up."));
    } else if (az < 0) {
        Serial.println(F("!! Z reads NEGATIVE gravity -- the IMU is upside down."));
    } else {
        Serial.println(F("OK: IMU is flat and right side up."));
    }

    int fAxis, fSign, rAxis, rSign;
    findAxis(imu, F("LIFT THE FRONT of the robot (nose UP, rear on the floor)."),
             fAxis, fSign);
    Serial.printf("\n>>> constexpr uint8_t ODOM_FORWARD_AXIS = %d;\n"
                  ">>> constexpr int8_t  ODOM_FORWARD_SIGN = %+d;\n", fAxis, fSign);

    findAxis(imu, F("LIFT THE RIGHT SIDE of the robot (left side on the floor)."),
             rAxis, rSign);
    Serial.printf("\n>>> constexpr uint8_t ODOM_RIGHT_AXIS   = %d;\n"
                  ">>> constexpr int8_t  ODOM_RIGHT_SIGN   = %+d;\n", rAxis, rSign);

    if (fAxis == rAxis)
        Serial.println(F("!! Forward and right resolved to the SAME axis. One tilt was\n"
                         "!! wrong or too small -- redo test 1."));

    Serial.println(F("\nNow rotate the robot COUNTER-CLOCKWISE (left) about 90 deg."));
    float y0 = imu.getGyroZdeg();
    waitForEnter(F("Press ENTER after rotating..."));
    float y1 = imu.getGyroZdeg();
    float d = y1 - y0;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    int ySign = (d > 0) ? 1 : -1;
    Serial.printf("Gyro Z changed by %+.1f deg\n", d);
    Serial.printf("\n>>> constexpr int8_t  ODOM_YAW_SIGN     = %+d;\n", ySign);
    if (fabsf(d) < 30.0f)
        Serial.println(F("!! Barely moved -- rotate a clear 90 deg and redo."));

    // Apply in RAM so tests 2-5 use the right axes immediately. Tests 2/3 read
    // acceleration through these to estimate motorTau, so without this you
    // would have to paste the results in and reflash before they meant
    // anything. Paste them in anyway -- this does not survive a reboot.
    OdomConfig& c = odom.config();
    c.forwardAxis = (uint8_t)fAxis; c.forwardSign = (int8_t)fSign;
    c.rightAxis   = (uint8_t)rAxis; c.rightSign   = (int8_t)rSign;
    c.yawSign     = (int8_t)ySign;
    odom.begin();   // re-latch heading with the corrected sign

    Serial.println(F("\nApplied for this session. Paste the >>> lines into"
                     "\nOdomConstants.h (replacing the matching lines) and reflash"
                     "\nto make them permanent."));
}

// ---------------------------------------------------------------------------
//  Test 2/3 -- top speed, and a first-order estimate of the motor time constant
// ---------------------------------------------------------------------------
static void testSpeed(IMU& imu, MecanumDrive& drive, MecanumOdometry& odom, bool strafe) {
    Serial.printf("\n=== TEST %d: max %s speed ===\n", strafe ? 3 : 2,
                  strafe ? "STRAFE" : "FORWARD");
    Serial.println(F("Put the robot on the floor with 4+ meters of clear space."));
    Serial.println(F("Mark the starting position of a fixed point on the chassis."));
    Serial.println(F("It will drive at 100% for 1.5 s, then coast to a stop."));
    waitForEnter(F("Press ENTER to run (or reposition first)..."));

    Serial.println(F("3...")); delay(1000);
    Serial.println(F("2...")); delay(1000);
    Serial.println(F("1...")); delay(1000);

    const OdomConfig& c = odom.config();
    float peakAccel = 0.0f;

    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
        if (strafe) drive.drive(100, 0, 0); else drive.drive(0, 100, 0);

        float v[3] = { imu.getAccelXms(), imu.getAccelYms(), imu.getAccelZms() };
        uint8_t axis = strafe ? c.rightAxis : c.forwardAxis;
        int8_t  sgn  = strafe ? c.rightSign : c.forwardSign;
        float a = v[axis] * (float)sgn;
        if (a > peakAccel) peakAccel = a;
        delay(5);
    }
    drive.drive(0, 0, 0);

    Serial.println(F("\nDone. Measure the distance the robot travelled WHILE POWERED"));
    Serial.println(F("(ignore the coast at the end -- mark where it started slowing)."));
    Serial.println(F("Enter the powered distance in METERS, then ENTER:"));

    while (Serial.available()) Serial.read();
    while (!Serial.available()) delay(10);
    String s = Serial.readStringUntil('\n');
    float dist = s.toFloat();

    if (dist <= 0.0f) { Serial.println(F("Bad input, skipping.")); return; }

    // It spent roughly one time-constant getting up to speed, so the powered
    // distance is a bit less than v_max * t. Solving the first-order response
    // exactly: d = v*(T - tau*(1 - e^(-T/tau))). Using the peak acceleration
    // for tau makes this self-consistent enough for a first pass.
    float T = 1.5f;
    float vGuess = dist / T;
    float tau = (peakAccel > 0.5f) ? (vGuess / peakAccel) : 0.25f;
    if (tau > 1.0f) tau = 1.0f;
    float vMax = dist / (T - tau * (1.0f - expf(-T / tau)));

    Serial.printf("\nPeak measured accel: %.2f m/s^2\n", peakAccel);
    if (strafe) Serial.printf("\n>>> constexpr float ODOM_MAX_STRAFE_SPEED  = %.2ff;\n", vMax);
    else        Serial.printf("\n>>> constexpr float ODOM_MAX_FORWARD_SPEED = %.2ff;\n", vMax);
    Serial.printf(">>> constexpr float ODOM_MOTOR_TAU = %.2ff;   // forward test only\n", tau);
}

// ---------------------------------------------------------------------------
//  Test 4 -- command deadband
// ---------------------------------------------------------------------------
static void testDeadband(IMU& imu, MecanumDrive& drive) {
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
//  Test 5 -- drift check with the robot parked
// ---------------------------------------------------------------------------
static void testDrift(MecanumOdometry& odom) {
    Serial.println(F("\n=== TEST 5: stationary drift ==="));
    Serial.println(F("Leave the robot COMPLETELY STILL for 30 seconds."));
    Serial.println(F("A well-tuned filter should stay under a few cm."));
    waitForEnter(F("Press ENTER to start..."));

    odom.reset();
    uint32_t t0 = millis();
    uint32_t lastPrint = 0;
    while (millis() - t0 < 30000) {
        odom.setCommand(0, 0, 0);
        odom.update();
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            MecanumOdometry::Pose p = odom.getPose();
            Serial.printf("t=%2lus  x=%+.3f y=%+.3f  th=%+.1f  drift=%.3f m  %s\n",
                          (millis() - t0) / 1000, p.x, p.y, odom.getThetaDeg(),
                          sqrtf(p.x * p.x + p.y * p.y),
                          odom.isStationary() ? "[ZUPT]" : "");
        }
    }
    MecanumOdometry::Pose p = odom.getPose();
    Serial.printf("\nFinal drift: %.3f m over 30 s, heading %.2f deg\n",
                  sqrtf(p.x * p.x + p.y * p.y), odom.getThetaDeg());
    Serial.printf("Accel bias: fwd=%.3f right=%.3f  Gyro bias: %.4f rad/s\n",
                  odom.getBiasForward(), odom.getBiasRight(), odom.getGyroBias());
    if (!odom.isStationary())
        Serial.println(F("!! ZUPT never engaged -- raise zuptAccelThresh / zuptYawRateThresh."));
}

// ---------------------------------------------------------------------------
void runOdometryCalibration(IMU& imu, MecanumDrive& drive, MecanumOdometry& odom) {
    Serial.println(F("\n\n########################################"));
    Serial.println(F("#   ODOMETRY CALIBRATION MODE          #"));
    Serial.println(F("#   THE ROBOT WILL DRIVE ITSELF.       #"));
    Serial.println(F("########################################"));

    for (;;) {
        Serial.println(F("\n--- MENU ---"));
        Serial.println(F("  1  IMU axis mapping + signs"));
        Serial.println(F("  2  Max forward speed + motor tau"));
        Serial.println(F("  3  Max strafe speed"));
        Serial.println(F("  4  Deadband"));
        Serial.println(F("  5  Stationary drift check"));
        Serial.println(F("  6  Live pose stream"));
        Serial.print(F("Choose: "));

        while (Serial.available()) Serial.read();
        while (!Serial.available()) delay(10);
        int choice = Serial.read() - '0';
        while (Serial.available()) Serial.read();
        Serial.println(choice);

        switch (choice) {
            case 1: testAxes(imu, odom); break;
            case 2: testSpeed(imu, drive, odom, false); break;
            case 3: testSpeed(imu, drive, odom, true); break;
            case 4: testDeadband(imu, drive); break;
            case 5: testDrift(odom); break;
            case 6: {
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
            default: Serial.println(F("Unknown option.")); break;
        }
        drive.drive(0, 0, 0);
    }
}
