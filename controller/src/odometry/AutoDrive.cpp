#include "AutoDrive.h"

float wrapDegrees180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

// Push a magnitude clear of the drivetrain deadband. Below it the robot stalls
// short of the target and sits buzzing until the timeout.
static float floorMagnitude(float value, float minMag, float maxMag) {
    float mag = fabsf(value);
    if (mag < 0.01f) return 0.0f;          // genuinely zero: leave it alone
    if (mag < minMag) mag = minMag;
    if (mag > maxMag) mag = maxMag;
    return (value < 0.0f) ? -mag : mag;
}

AutoDrive::AutoDrive(MecanumDrive& drive_, MecanumOdometry& odom_)
    : drive(drive_), odom(odom_), abortFn(nullptr),
      distTol(AUTO_DISTANCE_TOLERANCE), angTol(AUTO_ANGLE_TOLERANCE),
      timeoutMs(0), lastHeadingError(0.0f) {}

// Every motion goes through here so the odometry can never miss a command --
// the commanded velocity IS the position estimate.
void AutoDrive::applyDrive(int x, int y, int turn) {
    drive.drive(x, y, turn);
    odom.setCommand(x, y, turn);
}

void AutoDrive::stop() {
    applyDrive(0, 0, 0);
    odom.update();
}

bool AutoDrive::aborted(const char* what) {
    if (abortFn && abortFn()) {
        stop();
        Serial.print(F("AUTO: aborted during "));
        Serial.println(what);
        return true;
    }
    return false;
}

// P+D on heading error, returned as a turn command in percent.
float AutoDrive::headingCorrection(float targetDeg, float dt) {
    float err = wrapDegrees180(targetDeg - odom.getThetaDeg());
    float rate = (dt > 1e-4f) ? (err - lastHeadingError) / dt : 0.0f;
    lastHeadingError = err;

    float out = AUTO_HEADING_KP * err + AUTO_HEADING_KD * rate;
    return (float)AUTO_TURN_SIGN * out;
}

// ---------------------------------------------------------------------------
//  Straight-line and strafe moves
// ---------------------------------------------------------------------------
bool AutoDrive::driveDistance(float meters, int speedPct) {
    return driveAxis(meters, speedPct, false);
}

bool AutoDrive::strafeDistance(float meters, int speedPct) {
    return driveAxis(meters, speedPct, true);
}

bool AutoDrive::driveAxis(float meters, int speedPct, bool strafe) {
    odom.update();

    const float startX = odom.getX();
    const float startY = odom.getY();
    const float holdHeading = odom.getThetaDeg();

    // Unit vector along the axis we intend to travel, in field coordinates.
    // Progress is PROJECTED onto it, so sideways drift doesn't get counted as
    // distance travelled -- straight-line error and along-track error stay
    // separate quantities.
    float th = odom.getThetaRad();
    float ux, uy;
    if (strafe) { ux =  sinf(th); uy = -cosf(th); }   // +right
    else        { ux =  cosf(th); uy =  sinf(th); }   // +forward

    float maxSpeed = (float)abs(speedPct);
    if (maxSpeed > 100.0f) maxSpeed = 100.0f;

    // Derive a timeout from the distance if none was set: the time this should
    // take at the requested speed, tripled, plus a second of slack.
    uint32_t limit = timeoutMs;
    if (limit == 0) {
        float vRef = (strafe ? ODOM_MAX_STRAFE_SPEED : ODOM_MAX_FORWARD_SPEED)
                   * (maxSpeed / 100.0f);
        float expected = (vRef > 0.05f) ? (fabsf(meters) / vRef) : 5.0f;
        limit = (uint32_t)(expected * 3000.0f) + 1000;
    }

    lastHeadingError = wrapDegrees180(holdHeading - odom.getThetaDeg());
    uint32_t start = millis();
    uint32_t settledSince = 0;
    uint32_t lastUs = micros();

    for (;;) {
        if (aborted(strafe ? "strafeDistance" : "driveDistance")) return false;
        if (millis() - start > limit) {
            stop();
            Serial.print(F("AUTO: timeout after "));
            Serial.print(limit);
            Serial.println(F(" ms -- pose is NOT at the target"));
            return false;
        }

        odom.update();

        uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;

        float travelled = (odom.getX() - startX) * ux + (odom.getY() - startY) * uy;
        float remaining = meters - travelled;

        if (fabsf(remaining) <= distTol) {
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= AUTO_SETTLE_MS) {
                stop();
                return true;
            }
        } else {
            settledSince = 0;
        }

        // Proportional on remaining distance: full speed far out, easing into
        // the target rather than slamming to a halt and coasting past it.
        float cmd = floorMagnitude(AUTO_DISTANCE_KP * remaining,
                                   AUTO_MIN_DRIVE_PCT, maxSpeed);

        float corr = headingCorrection(holdHeading, dt);
        if (corr >  AUTO_MAX_HEADING_CORRECTION) corr =  AUTO_MAX_HEADING_CORRECTION;
        if (corr < -AUTO_MAX_HEADING_CORRECTION) corr = -AUTO_MAX_HEADING_CORRECTION;

        if (strafe) applyDrive((int)cmd, 0, (int)corr);
        else        applyDrive(0, (int)cmd, (int)corr);

        delay(5);
    }
}

// ---------------------------------------------------------------------------
//  Turns -- closed-loop on the gyro, the accurate half of this class
// ---------------------------------------------------------------------------
bool AutoDrive::turnBy(float deltaDeg, int speedPct) {
    odom.update();
    return turnToAngle(odom.getThetaDeg() + deltaDeg, speedPct);
}

bool AutoDrive::turnToAngle(float headingDeg, int speedPct) {
    odom.update();

    float maxSpeed = (float)abs(speedPct);
    if (maxSpeed > 100.0f) maxSpeed = 100.0f;

    float firstError = wrapDegrees180(headingDeg - odom.getThetaDeg());
    lastHeadingError = firstError;

    uint32_t limit = timeoutMs ? timeoutMs : 6000;
    uint32_t start = millis();
    uint32_t settledSince = 0;
    uint32_t lastUs = micros();

    for (;;) {
        if (aborted("turnToAngle")) return false;
        if (millis() - start > limit) {
            stop();
            Serial.println(F("AUTO: turn timed out -- heading is NOT at target"));
            return false;
        }

        odom.update();

        uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;

        float err = wrapDegrees180(headingDeg - odom.getThetaDeg());

        // Runaway guard. If AUTO_TURN_SIGN is backwards the robot spins away
        // from the target forever, which on a real field means a robot loose at
        // full throttle. Catch it instead of making the user reason it out.
        if (fabsf(err) > fabsf(firstError) + AUTO_RUNAWAY_DEG) {
            stop();
            Serial.println(F("AUTO: turn is running AWAY from the target."));
            Serial.println(F("AUTO: flip AUTO_TURN_SIGN in OdomConstants.h."));
            return false;
        }

        if (fabsf(err) <= angTol) {
            if (settledSince == 0) settledSince = millis();
            if (millis() - settledSince >= AUTO_SETTLE_MS) {
                stop();
                return true;
            }
            applyDrive(0, 0, 0);   // coast inside the tolerance band while settling
            delay(5);
            continue;
        }
        settledSince = 0;

        float corr = headingCorrection(headingDeg, dt);
        applyDrive(0, 0, (int)floorMagnitude(corr, AUTO_MIN_TURN_PCT, maxSpeed));

        delay(5);
    }
}
