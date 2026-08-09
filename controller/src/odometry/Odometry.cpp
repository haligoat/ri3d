#include "Odometry.h"

static const float DEG2RAD = 0.0174532925f;

OdomConfig OdomConfig::defaults() {
    // Every value lives in OdomConstants.h -- that is the file you edit after
    // running calibration. Nothing robot-specific belongs here.
    OdomConfig c;
    c.yawSign         = ODOM_YAW_SIGN;
    c.maxForwardSpeed = ODOM_MAX_FORWARD_SPEED;
    c.maxStrafeSpeed  = ODOM_MAX_STRAFE_SPEED;
    c.motorTau        = ODOM_MOTOR_TAU;
    c.deadbandPct     = ODOM_DEADBAND_PCT;
    return c;
}

MecanumOdometry::MecanumOdometry(IMU& imu_, const OdomConfig& cfg_)
    : imu(imu_), cfg(cfg_), updateIntervalMs(10) {
    pose.x = pose.y = pose.theta = 0.0f;
    headingRaw = headingOffset = lastRawDeg = yawRate = 0.0f;
    cmdX = cmdY = cmdTurn = 0;
    modelVFwd = modelVRight = 0.0f;
    lastUpdateUs = lastStepMs = 0;
    started = false;
}

void MecanumOdometry::begin() {
    reset();
    // Latch the current heading as zero. imu.begin() has already calibrated the
    // gyro, so this just removes the arbitrary power-on angle -- which is also
    // why a fixed IMU mounting rotation needs no correction anywhere.
    lastRawDeg    = imu.getGyroZdeg();
    headingRaw    = 0.0f;
    headingOffset = 0.0f;

    lastUpdateUs = micros();
    lastStepMs   = millis();
    started      = true;
}

void MecanumOdometry::reset() {
    pose.x = pose.y = pose.theta = 0.0f;
    headingOffset = headingRaw;
    modelVFwd = modelVRight = 0.0f;
}

void MecanumOdometry::setPose(float x, float y, float thetaRad) {
    pose.x = x;
    pose.y = y;
    pose.theta = thetaRad;
    headingOffset = headingRaw - thetaRad;
}

void MecanumOdometry::setCommand(int x, int y, int turn) {
    cmdX = x;
    cmdY = y;
    cmdTurn = turn;
}

// ---------------------------------------------------------------------------
//  Heading -- the one genuinely good measurement we have
// ---------------------------------------------------------------------------
void MecanumOdometry::updateHeading() {
    // getGyroZdeg() wraps to 0..360, so recover a continuous angle by
    // accumulating the shortest-path delta each step. At 100 Hz this only
    // breaks past 18000 deg/s, so there is enormous margin.
    float rawDeg = imu.getGyroZdeg();
    float d = rawDeg - lastRawDeg;
    lastRawDeg = rawDeg;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;

    headingRaw += d * DEG2RAD * (float)cfg.yawSign;
    pose.theta = headingRaw - headingOffset;
}

// ---------------------------------------------------------------------------
//  Commanded-velocity model
// ---------------------------------------------------------------------------
void MecanumOdometry::updateModelVelocity(float dt) {
    // Replicate MecanumDrive::drive() exactly, including the fact that it does
    // NOT normalise: each wheel command is clamped to +/-100 by
    // MotorControllers::percentToCycle(). Above a combined 100 that clamping
    // silently distorts the motion, so the model has to clamp too or it
    // over-predicts speed on exactly the aggressive commands that matter.
    float FL = (float)(cmdY + cmdX + cmdTurn);
    float FR = (float)(cmdY - cmdX - cmdTurn);
    float BL = (float)(cmdY - cmdX + cmdTurn);
    float BR = (float)(cmdY + cmdX - cmdTurn);

    FL = constrain(FL, -100.0f, 100.0f);
    FR = constrain(FR, -100.0f, 100.0f);
    BL = constrain(BL, -100.0f, 100.0f);
    BR = constrain(BR, -100.0f, 100.0f);

    // Forward kinematics -- invert the mixing above.
    float effFwd   = (FL + FR + BL + BR) * 0.25f;
    float effRight = (FL - FR - BL + BR) * 0.25f;

    if (fabsf(effFwd)   < cfg.deadbandPct) effFwd   = 0.0f;
    if (fabsf(effRight) < cfg.deadbandPct) effRight = 0.0f;

    float targetFwd   = (effFwd   / 100.0f) * cfg.maxForwardSpeed;
    float targetRight = (effRight / 100.0f) * cfg.maxStrafeSpeed;

    // First-order lag: the chassis cannot step to a new speed instantly.
    float alpha = (cfg.motorTau > 1e-3f) ? (dt / cfg.motorTau) : 1.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    modelVFwd   += (targetFwd   - modelVFwd)   * alpha;
    modelVRight += (targetRight - modelVRight) * alpha;
}

// ---------------------------------------------------------------------------
void MecanumOdometry::update() {
    if (!started) return;

    uint32_t nowMs = millis();
    if ((nowMs - lastStepMs) < updateIntervalMs) return;
    lastStepMs = nowMs;

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUpdateUs) * 1e-6f;   // unsigned math handles wrap
    lastUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.2f) return;         // first call, or a stall

    float prevTheta = pose.theta;
    updateHeading();
    yawRate = (pose.theta - prevTheta) / dt;

    updateModelVelocity(dt);

    // Body -> field. Field +Y is LEFT, so the right component enters negated.
    float c = cosf(pose.theta);
    float s = sinf(pose.theta);
    pose.x += (c * modelVFwd + s * modelVRight) * dt;
    pose.y += (s * modelVFwd - c * modelVRight) * dt;
}

MecanumOdometry::Twist MecanumOdometry::getVelocity() const {
    float c = cosf(pose.theta);
    float s = sinf(pose.theta);
    Twist t;
    t.vx = c * modelVFwd + s * modelVRight;
    t.vy = s * modelVFwd - c * modelVRight;
    t.omega = yawRate;
    return t;
}

float MecanumOdometry::getSpeed() const {
    return sqrtf(modelVFwd * modelVFwd + modelVRight * modelVRight);
}

bool MecanumOdometry::isStationary() const {
    return cmdX == 0 && cmdY == 0 && cmdTurn == 0
        && fabsf(modelVFwd) < 0.02f && fabsf(modelVRight) < 0.02f;
}

String MecanumOdometry::telemetry() const {
    Twist v = getVelocity();
    String s;
    s.reserve(72);
    s += String(pose.x, 3);        s += ',';
    s += String(pose.y, 3);        s += ',';
    s += String(getThetaDeg(), 1); s += ',';
    s += String(v.vx, 2);          s += ',';
    s += String(v.vy, 2);          s += ',';
    s += (isStationary() ? '1' : '0');
    return s;
}
