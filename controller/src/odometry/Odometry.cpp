#include "Odometry.h"

static const float G_MS2   = 9.80665f;
static const float DEG2RAD = 0.0174532925f;

OdomConfig OdomConfig::defaults() {
    // Every value lives in OdomConstants.h -- that is the file you edit after
    // running calibration. Nothing robot-specific should be hard-coded here.
    OdomConfig c;

    c.forwardAxis = ODOM_FORWARD_AXIS;
    c.forwardSign = ODOM_FORWARD_SIGN;
    c.rightAxis   = ODOM_RIGHT_AXIS;
    c.rightSign   = ODOM_RIGHT_SIGN;
    c.yawSign     = ODOM_YAW_SIGN;

    c.maxForwardSpeed = ODOM_MAX_FORWARD_SPEED;
    c.maxStrafeSpeed  = ODOM_MAX_STRAFE_SPEED;
    c.motorTau        = ODOM_MOTOR_TAU;
    c.deadbandPct     = ODOM_DEADBAND_PCT;

    c.accelNoise        = ODOM_ACCEL_NOISE;
    c.accelBiasWalk     = ODOM_ACCEL_BIAS_WALK;
    c.modelVelNoise     = ODOM_MODEL_VEL_NOISE;
    c.modelVelNoiseTurn = ODOM_MODEL_VEL_NOISE_TURN;
    c.zuptNoise         = ODOM_ZUPT_NOISE;

    c.slipScale  = ODOM_SLIP_SCALE;
    c.slipMemory = ODOM_SLIP_MEMORY;

    c.zuptAccelThresh   = ODOM_ZUPT_ACCEL_THRESH;
    c.zuptYawRateThresh = ODOM_ZUPT_YAW_RATE_THRESH;
    c.zuptVelThresh     = ODOM_ZUPT_VEL_THRESH;
    c.zuptHoldMs        = ODOM_ZUPT_HOLD_MS;
    c.zuptFilterTau     = ODOM_ZUPT_FILTER_TAU;

    return c;
}

MecanumOdometry::MecanumOdometry(IMU& imu_, const OdomConfig& cfg_)
    : imu(imu_), cfg(cfg_), updateIntervalMs(10) {
    for (int i = 0; i < 6; i++) {
        state[i] = 0.0f;
        for (int j = 0; j < 6; j++) P[i][j] = 0.0f;
    }
    pose.x = pose.y = pose.theta = 0.0f;
    headingRaw = headingOffset = gyroBiasZ = gyroBiasCorr = 0.0f;
    lastRawDeg = 0.0f;
    yawRate = 0.0f;
    headingInit = false;
    cmdX = cmdY = cmdTurn = 0;
    modelVFwd = modelVRight = 0.0f;
    freeVx = freeVy = 0.0f;
    lastAfx = lastAfy = 0.0f;
    accelMagLp = yawRateLp = 0.0f;
    stationary = false;
    stillSince = 0;
    lastUpdateUs = 0;
    lastStepMs = 0;
    started = false;
}

void MecanumOdometry::begin() {
    reset();
    // Latch the current heading as zero. imu.begin() should already have run
    // its gyro calibration, so this only removes the arbitrary power-on angle.
    headingInit = false;
    lastRawDeg = imu.getGyroZdeg();
    headingRaw = 0.0f;
    headingOffset = 0.0f;
    gyroBiasCorr = 0.0f;
    headingInit = true;

    lastUpdateUs = micros();
    lastStepMs = millis();
    stillSince = millis();
    started = true;
}

void MecanumOdometry::reset() {
    for (int i = 0; i < 4; i++) state[i] = 0.0f;   // keep bias estimates
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) P[i][j] = 0.0f;

    // Start confident in position (it is zero by definition) and velocity,
    // but keep whatever uncertainty we had about the biases.
    P[0][0] = P[1][1] = 1e-4f;
    P[2][2] = P[3][3] = 1e-3f;
    P[4][4] = P[5][5] = 0.5f;

    pose.x = pose.y = 0.0f;
    pose.theta = 0.0f;
    headingOffset = headingRaw - gyroBiasCorr;
    modelVFwd = modelVRight = 0.0f;
    freeVx = freeVy = 0.0f;
}

void MecanumOdometry::setPose(float x, float y, float thetaRad) {
    state[0] = x;
    state[1] = y;
    pose.x = x;
    pose.y = y;
    pose.theta = thetaRad;
    headingOffset = (headingRaw - gyroBiasCorr) - thetaRad;
}

void MecanumOdometry::setCommand(int x, int y, int turn) {
    cmdX = x;
    cmdY = y;
    cmdTurn = turn;
}

// ---------------------------------------------------------------------------
//  Heading
// ---------------------------------------------------------------------------
void MecanumOdometry::updateHeading(float dt) {
    // getGyroZdeg() wraps to 0..360, so recover a continuous angle by
    // accumulating the shortest-path delta each step. This is why the filter
    // has to run faster than the robot can turn 180 degrees between samples --
    // at 100 Hz that is 18000 deg/s, so there is enormous margin.
    float rawDeg = imu.getGyroZdeg();
    float d = rawDeg - lastRawDeg;
    lastRawDeg = rawDeg;
    if (d > 180.0f)  d -= 360.0f;
    if (d < -180.0f) d += 360.0f;

    float dRad = d * DEG2RAD * (float)cfg.yawSign;
    headingRaw += dRad;

    // Integrate the learned gyro bias out of the heading.
    gyroBiasCorr += gyroBiasZ * dt;

    float prevTheta = pose.theta;
    pose.theta = headingRaw - gyroBiasCorr - headingOffset;
    yawRate = (dt > 1e-6f) ? (pose.theta - prevTheta) / dt : 0.0f;
}

// ---------------------------------------------------------------------------
//  Body-frame acceleration, gravity removed
// ---------------------------------------------------------------------------
void MecanumOdometry::readBodyAccel(float& aFwd, float& aRight) {
    float roll, pitch, yaw;
    imu.getOrientation(roll, pitch, yaw);   // one coherent burst read

    float a[3] = { imu.getAccelXms(), imu.getAccelYms(), imu.getAccelZms() };

    // Gravity expressed in the body frame, using the library's own roll/pitch
    // convention (accel_roll = atan2(ay, ...), accel_pitch = atan2(-ax, ...)),
    // so level reads (0, 0, +g).
    float r = roll * DEG2RAD;
    float p = pitch * DEG2RAD;
    float gb[3] = { -G_MS2 * sinf(p),
                     G_MS2 * sinf(r),
                     G_MS2 * cosf(r) * cosf(p) };

    // Removing gravity via the complementary-filtered tilt matters more than
    // it looks: when the chassis pitches back under hard acceleration, gravity
    // leaks straight into the forward axis and reads as phantom thrust.
    float lin[3] = { a[0] - gb[0], a[1] - gb[1], a[2] - gb[2] };

    aFwd   = lin[cfg.forwardAxis] * (float)cfg.forwardSign;
    aRight = lin[cfg.rightAxis]   * (float)cfg.rightSign;
}

// ---------------------------------------------------------------------------
//  Commanded-velocity model
// ---------------------------------------------------------------------------
void MecanumOdometry::updateModelVelocity(float dt) {
    // Replicate MecanumDrive::drive() exactly, including the fact that it does
    // NOT normalise: each wheel command is clamped to +/-100 by
    // MotorControllers::percentToCycle(). At combined commands above 100 that
    // clamping silently distorts the motion, so the model has to clamp too or
    // it will over-predict speed in exactly the situations that matter.
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
//  Stationary detection
// ---------------------------------------------------------------------------
bool MecanumOdometry::detectStationary(float aFwd, float aRight, float dt) {
    // Test the BIAS-CORRECTED acceleration, not the raw reading. A real
    // accelerometer bias is easily 0.3 m/s^2, which on its own sits close to
    // the threshold -- judging the raw signal makes ZUPT flicker on exactly
    // the robots whose bias most needs correcting.
    float cFwd   = aFwd   - state[4];
    float cRight = aRight - state[5];
    float aMag = sqrtf(cFwd * cFwd + cRight * cRight);

    // Low-pass before comparing. This is not cosmetic: chassis vibration puts
    // individual samples well above the threshold even when parked, and since
    // any one sample over the line restarts the zuptHoldMs timer, an
    // instantaneous test can keep ZUPT from EVER latching. Measured on the
    // wall-jam case, the unfiltered version engaged on only 4 of 12 noise
    // seeds; filtered, it is deterministic. The noise is zero-mean, so
    // averaging removes it while leaving real motion intact.
    float alpha = dt / cfg.zuptFilterTau;
    if (alpha > 1.0f) alpha = 1.0f;
    accelMagLp += (aMag - accelMagLp) * alpha;
    yawRateLp  += (fabsf(yawRate) - yawRateLp) * alpha;

    bool quiet = accelMagLp < cfg.zuptAccelThresh
              && yawRateLp  < cfg.zuptYawRateThresh;

    // "Not accelerating and not turning" is also true of a robot cruising at a
    // constant speed, so it cannot decide this on its own. The accelerometer-
    // only velocity is what separates the two: it reads ~0 for a robot that is
    // genuinely parked and ~cruise speed for one that is moving.
    //
    // Deliberately NOT requiring a zero command. A robot jammed against a wall
    // or another robot at full throttle is stationary, and that is precisely
    // the case where the commanded-velocity model runs away -- gating ZUPT on
    // the command would blind us to it.
    bool velocityZero = sqrtf(freeVx * freeVx + freeVy * freeVy) < cfg.zuptVelThresh;

    // The command-is-zero path stays as an alternative because freeV is only
    // trustworthy once the accel bias has converged, and bias convergence
    // itself depends on ZUPT. Without this, a cold boot deadlocks.
    bool commandZero = (cmdX == 0 && cmdY == 0 && cmdTurn == 0);

    quiet = quiet && (commandZero || velocityZero);

    uint32_t now = millis();
    if (!quiet) {
        stillSince = now;
        return false;
    }
    return (now - stillSince) >= cfg.zuptHoldMs;
}

// ---------------------------------------------------------------------------
//  Kalman predict
// ---------------------------------------------------------------------------
void MecanumOdometry::predict(float dt, float aFwd, float aRight) {
    float c = cosf(pose.theta);
    float s = sinf(pose.theta);

    // Body (forward, right) -> field (X, Y). Field +Y is LEFT, so the right
    // component enters negated: left = -right.
    //   fieldX = c*fwd + s*right
    //   fieldY = s*fwd - c*right
    float R00 =  c, R01 =  s;
    float R10 =  s, R11 = -c;

    // Bias-corrected acceleration in the field frame.
    float bf = state[4], br = state[5];
    float ax = R00 * (aFwd - bf) + R01 * (aRight - br);
    float ay = R10 * (aFwd - bf) + R11 * (aRight - br);
    lastAfx = ax;
    lastAfy = ay;

    // ---- State propagation (constant acceleration over the step) ----
    state[0] += state[2] * dt + 0.5f * ax * dt * dt;
    state[1] += state[3] * dt + 0.5f * ay * dt * dt;
    state[2] += ax * dt;
    state[3] += ay * dt;
    // biases are a random walk: unchanged in the mean

    // ---- Covariance: P = F P F^T + Q ----
    // F is identity plus the position/velocity coupling and the bias
    // sensitivity. d(pos)/d(bias) = -0.5*R*dt^2, d(vel)/d(bias) = -R*dt,
    // because a positive bias means we over-estimated acceleration.
    float h = 0.5f * dt * dt;
    float F[6][6] = {
        { 1, 0, dt, 0,  -h * R00,  -h * R01 },
        { 0, 1, 0, dt,  -h * R10,  -h * R11 },
        { 0, 0, 1, 0,  -dt * R00, -dt * R01 },
        { 0, 0, 0, 1,  -dt * R10, -dt * R11 },
        { 0, 0, 0, 0,     1,          0     },
        { 0, 0, 0, 0,     0,          1     }
    };

    float FP[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++) sum += F[i][k] * P[k][j];
            FP[i][j] = sum;
        }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++) sum += FP[i][k] * F[j][k];
            P[i][j] = sum;
        }

    // Process noise: standard constant-acceleration discretisation driven by
    // accelerometer noise, plus a random walk on the bias states.
    float sa2 = cfg.accelNoise * cfg.accelNoise;
    float qpp = sa2 * dt * dt * dt * dt * 0.25f;
    float qpv = sa2 * dt * dt * dt * 0.5f;
    float qvv = sa2 * dt * dt;
    float qbb = cfg.accelBiasWalk * cfg.accelBiasWalk * dt;

    P[0][0] += qpp;  P[1][1] += qpp;
    P[2][2] += qvv;  P[3][3] += qvv;
    P[0][2] += qpv;  P[2][0] += qpv;
    P[1][3] += qpv;  P[3][1] += qpv;
    P[4][4] += qbb;  P[5][5] += qbb;
}

// ---------------------------------------------------------------------------
//  Kalman correct -- measures field-frame velocity [vx, vy] (states 2 and 3)
// ---------------------------------------------------------------------------
void MecanumOdometry::correctVelocity(float zvx, float zvy, float r) {
    // Innovation
    float y0 = zvx - state[2];
    float y1 = zvy - state[3];

    // S = H P H^T + R, with H selecting states 2 and 3.
    float S00 = P[2][2] + r, S01 = P[2][3];
    float S10 = P[3][2],     S11 = P[3][3] + r;

    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-12f) return;   // singular; skip rather than blow up
    float invDet = 1.0f / det;
    float Si00 =  S11 * invDet, Si01 = -S01 * invDet;
    float Si10 = -S10 * invDet, Si11 =  S00 * invDet;

    // K = P H^T S^-1  (P H^T is just columns 2 and 3 of P)
    float K[6][2];
    for (int i = 0; i < 6; i++) {
        float p0 = P[i][2], p1 = P[i][3];
        K[i][0] = p0 * Si00 + p1 * Si10;
        K[i][1] = p0 * Si01 + p1 * Si11;
    }

    for (int i = 0; i < 6; i++) state[i] += K[i][0] * y0 + K[i][1] * y1;

    // Joseph form: P = A P A^T + K R K^T, where A = I - K H. Slower than the
    // short form but it stays symmetric positive-definite under float32, which
    // the short form does not over a long match.
    float A[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) A[i][j] = (i == j) ? 1.0f : 0.0f;
        A[i][2] -= K[i][0];
        A[i][3] -= K[i][1];
    }

    float AP[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++) sum += A[i][k] * P[k][j];
            AP[i][j] = sum;
        }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++) sum += AP[i][k] * A[j][k];
            P[i][j] = sum + (K[i][0] * K[j][0] + K[i][1] * K[j][1]) * r;
        }
}

// ---------------------------------------------------------------------------
//  Main step
// ---------------------------------------------------------------------------
void MecanumOdometry::update() {
    if (!started) return;

    uint32_t nowMs = millis();
    if ((nowMs - lastStepMs) < updateIntervalMs) return;
    lastStepMs = nowMs;

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUpdateUs) * 1e-6f;   // unsigned math handles wrap
    lastUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.2f) return;         // stall or first call

    updateHeading(dt);

    float aFwd, aRight;
    readBodyAccel(aFwd, aRight);

    updateModelVelocity(dt);

    predict(dt, aFwd, aRight);

    stationary = detectStationary(aFwd, aRight, dt);

    // Model velocity in the field frame. Needed here (not just in the update
    // branch) because the slip measure below feeds ZUPT detection too.
    float ct = cosf(pose.theta);
    float st = sinf(pose.theta);
    float zvx = ct * modelVFwd + st * modelVRight;
    float zvy = st * modelVFwd - ct * modelVRight;

    // Advance the accelerometer-only velocity. The whole point is that the
    // model never touches it: the filter's own velocity gets pulled toward the
    // model every step, so once the model is wrong the innovation collapses to
    // zero and the error becomes invisible. This channel stays independent, so
    // a jam or a slip shows up as a persistent disagreement instead of a
    // single transient spike.
    freeVx += lastAfx * dt;
    freeVy += lastAfy * dt;

    float dvx = freeVx - zvx;
    float dvy = freeVy - zvy;
    float slip2 = (dvx * dvx + dvy * dvy) / (cfg.slipScale * cfg.slipScale);

    // Bleed it back toward the filter so its own integration drift cannot grow
    // without bound during a long stretch of honest driving -- but suppress
    // that bleed exactly when the two disagree. An ungated leak pulls freeV
    // back toward the filter's model-dragged velocity and erases the slip
    // signal within a second or two, which is the whole thing we are trying to
    // measure.
    float leak = (dt / cfg.slipMemory) / (1.0f + slip2);
    if (leak > 1.0f) leak = 1.0f;
    freeVx += (state[2] - freeVx) * leak;
    freeVy += (state[3] - freeVy) * leak;

    if (stationary) {
        // ZUPT. This is what makes the accelerometer bias observable: holding
        // velocity at zero while the filter keeps integrating a biased accel
        // forces the mismatch into the bias states, where it belongs.
        correctVelocity(0.0f, 0.0f, cfg.zuptNoise * cfg.zuptNoise);

        // With the robot provably still, any residual yaw rate is gyro bias.
        gyroBiasZ += (yawRate - gyroBiasZ) * 0.02f;

        freeVx = freeVy = 0.0f;
    } else {
        // Correct with the commanded-velocity model. Rotation makes the
        // mecanum model less trustworthy (the wheels scrub), so inflate the
        // noise with |omega|, and de-weight it further in proportion to how far
        // it has drifted from what the accelerometer independently believes.
        float sigma = cfg.modelVelNoise + cfg.modelVelNoiseTurn * fabsf(yawRate);
        correctVelocity(zvx, zvy, sigma * sigma * (1.0f + slip2));
    }

    pose.x = state[0];
    pose.y = state[1];
}

MecanumOdometry::Twist MecanumOdometry::getVelocity() const {
    Twist t;
    t.vx = state[2];
    t.vy = state[3];
    t.omega = yawRate;
    return t;
}

float MecanumOdometry::getSpeed() const {
    return sqrtf(state[2] * state[2] + state[3] * state[3]);
}

String MecanumOdometry::telemetry() const {
    String s;
    s.reserve(72);
    s += String(state[0], 3); s += ',';
    s += String(state[1], 3); s += ',';
    s += String(getThetaDeg(), 1); s += ',';
    s += String(state[2], 2); s += ',';
    s += String(state[3], 2); s += ',';
    s += (stationary ? '1' : '0');
    return s;
}

float MecanumOdometry::getSlipEstimate() const {
    float c = cosf(pose.theta);
    float s = sinf(pose.theta);
    float zvx = c * modelVFwd + s * modelVRight;
    float zvy = s * modelVFwd - c * modelVRight;
    return sqrtf((freeVx - zvx) * (freeVx - zvx) + (freeVy - zvy) * (freeVy - zvy));
}
