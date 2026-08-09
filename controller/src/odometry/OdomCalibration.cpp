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
        Serial.println(F("  4  Deadband"));
        Serial.println(F("  5  Live pose stream"));
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
            default: Serial.println(F("Unknown option.")); break;
        }
        drive.drive(0, 0, 0);
    }
}
