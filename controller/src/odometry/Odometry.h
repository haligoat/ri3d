#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <EchoLib.h>
#include "OdomConstants.h"

// ============================================================================
//  MecanumOdometry -- pose tracking for an EchoLib mecanum robot with NO
//  wheel encoders.
//
//  WHY THIS IS SHAPED THE WAY IT IS
//  --------------------------------
//  EchoLib's Motor class is open-loop MCPWM: there is no encoder feedback
//  anywhere in the library, so classic mecanum wheel odometry (integrate
//  measured wheel rotations through the inverse kinematics) is not available.
//  The BMI270 also has no magnetometer, so there is no absolute heading.
//
//  What we DO have:
//    1. A very good yaw *rate* (gyro Z), which integrates into a heading that
//       drifts slowly -- good to a few degrees per minute once bias-corrected.
//    2. Body-frame acceleration, which is unbiased over short horizons but
//       double-integrates into garbage over long ones.
//    3. The commanded (x, y, turn) we hand to MecanumDrive, which -- run
//       through the drive's own kinematics and a first-order motor model --
//       predicts body velocity well while the wheels are gripping, and lies
//       during slip, stall, or a collision.
//
//  (2) and (3) fail in opposite regimes, which is exactly the situation a
//  Kalman filter is for. We PREDICT with the accelerometer and CORRECT with
//  the commanded-velocity model, plus a zero-velocity update (ZUPT) whenever
//  the robot is provably parked. The ZUPT is what makes accelerometer bias
//  observable and is the single biggest accuracy win in the whole file.
//
//  STATE (6): [ px, py, vx, vy, bx, by ]
//    px, py  position in the FIELD frame, meters
//    vx, vy  velocity in the FIELD frame, m/s
//    bx, by  accelerometer bias in the BODY frame, m/s^2
//
//  Heading theta is NOT a filter state. It comes straight from the gyro and
//  enters as a known time-varying rotation. That keeps the filter linear
//  (an LTV Kalman filter, not an EKF) -- less code, no Jacobian mistakes, and
//  it loses nothing because the gyro heading is far better than anything the
//  accelerometer could tell us about yaw anyway.
//
//  FRAMES
//    Body:  +forward is the robot's front, +right is its right side. This
//           matches MecanumDrive::drive(x, y, turn) where y = forward and
//           x = right.
//    Field: fixed at begin()/reset(). theta = 0 means the robot's nose points
//           along field +X. theta increases counter-clockwise (right-handed,
//           viewed from above), so +Y is field-left.
//
//  YOU MUST CALIBRATE THIS. See README_ODOMETRY.md -- the constants below are
//  placeholders, not measurements. Set ODOM_CALIBRATION_MODE to 1 in
//  controller.ino to get the guided procedure.
// ============================================================================

// Tunables. You do not edit these here -- the values live in OdomConstants.h,
// which is also where each one is explained and where calibration output goes.
struct OdomConfig {
    // IMU mounting. Axis: 0 = X, 1 = Y, 2 = Z.
    uint8_t forwardAxis;
    int8_t  forwardSign;
    uint8_t rightAxis;
    int8_t  rightSign;
    int8_t  yawSign;

    // Drivetrain model.
    float maxForwardSpeed;   // m/s at 100% command
    float maxStrafeSpeed;    // m/s at 100% command
    float motorTau;          // seconds to ~63% of a step change
    float deadbandPct;       // command percent below which nothing moves

    // Filter tuning.
    float accelNoise;        // m/s^2, 1-sigma
    float accelBiasWalk;     // m/s^2 per sqrt(s)
    float modelVelNoise;     // m/s
    float modelVelNoiseTurn; // extra m/s per rad/s of rotation
    float zuptNoise;         // m/s

    // Slip / collision rejection.
    float slipScale;         // m/s of disagreement that doubles model variance
    float slipMemory;        // seconds before the free velocity is bled back

    // Stationary (ZUPT) detection.
    float zuptAccelThresh;   // m/s^2
    float zuptYawRateThresh; // rad/s
    float zuptVelThresh;     // m/s, accelerometer-only speed
    uint32_t zuptHoldMs;
    float zuptFilterTau;     // s, low-pass on the stationarity signals

    static OdomConfig defaults();
};

class MecanumOdometry {
public:
    struct Pose {
        float x;      // meters, field frame
        float y;      // meters, field frame
        float theta;  // radians, continuous (NOT wrapped), CCW positive
    };

    struct Twist {
        float vx;     // m/s, field frame
        float vy;     // m/s, field frame
        float omega;  // rad/s
    };

    explicit MecanumOdometry(IMU& imu, const OdomConfig& cfg = OdomConfig::defaults());

    // Call once in setup(), AFTER imu.begin() and while the robot is still.
    // Latches the current heading as theta = 0 and zeroes the filter.
    void begin();

    // Zero pose and velocity, keep the learned biases. Cheap, safe mid-match.
    void reset();

    // Mirror of MecanumDrive::drive(). Call with the SAME arguments, every
    // time you call drive() -- including the zeros on a safety stop, or the
    // model will happily keep predicting motion the robot isn't making.
    void setCommand(int x, int y, int turn);

    // Run one predict/correct cycle. Call this every loop() iteration; it
    // self-limits to updateIntervalMs and is a no-op in between.
    void update();

    Pose  getPose()     const { return pose; }
    Twist getVelocity() const;

    float getX()        const { return state[0]; }
    float getY()        const { return state[1]; }
    float getThetaRad() const { return pose.theta; }
    float getThetaDeg() const { return pose.theta * 57.2957795f; }
    float getSpeed()    const;

    // True when ZUPT is actively holding the robot at zero velocity.
    bool  isStationary() const { return stationary; }

    // How far the commanded-velocity model currently disagrees with the
    // accelerometer, m/s. Large and sustained means slipping, jammed, or
    // being pushed. Handy to surface on the driver station.
    float getSlipEstimate() const;

    // Estimated accelerometer bias, body frame, m/s^2. Useful sanity check:
    // it should settle to a small constant, not wander.
    float getBiasForward() const { return state[4]; }
    float getBiasRight()   const { return state[5]; }

    // Estimated gyro Z bias, rad/s, learned during stationary periods.
    float getGyroBias() const { return gyroBiasZ; }

    // Force the pose to a known value -- e.g. from a field landmark, a wall
    // alignment, or the start of an auto routine. Heading is in radians.
    void setPose(float x, float y, float thetaRad);

    // How often the filter steps, milliseconds. Default 10 (100 Hz).
    void setUpdateInterval(uint16_t ms) { updateIntervalMs = ms; }

    OdomConfig& config() { return cfg; }

    // "x,y,thetaDeg,vx,vy,stationary" -- ready to hand to WiFiServerBridge.
    String telemetry() const;

private:
    void predict(float dt, float aFwd, float aRight);
    void correctVelocity(float zvx, float zvy, float r);
    void updateHeading(float dt);
    void readBodyAccel(float& aFwd, float& aRight);
    void updateModelVelocity(float dt);
    bool detectStationary(float aFwd, float aRight, float dt);

    IMU& imu;
    OdomConfig cfg;

    // Kalman filter
    float state[6];
    float P[6][6];

    Pose pose;

    // Heading bookkeeping
    float headingRaw;        // unwrapped gyro heading, radians
    float headingOffset;     // latched at begin()
    float gyroBiasZ;         // rad/s
    float gyroBiasCorr;      // accumulated integral of gyroBiasZ, radians
    float lastRawDeg;        // for unwrapping the 0..360 gyro angle
    float yawRate;           // rad/s, from finite difference
    bool  headingInit;

    // Commanded-velocity model
    int   cmdX, cmdY, cmdTurn;
    float modelVFwd, modelVRight;  // body frame, m/s

    // Accelerometer-only velocity, field frame. Deliberately NOT corrected by
    // the model, so it stays an independent opinion we can cross-check.
    float freeVx, freeVy;
    float lastAfx, lastAfy;        // last field-frame acceleration used

    // Stationary detection
    float    accelMagLp, yawRateLp;   // filtered stationarity signals
    bool     stationary;
    uint32_t stillSince;

    uint32_t lastUpdateUs;
    uint16_t updateIntervalMs;
    uint32_t lastStepMs;
    bool     started;
};

#endif // ODOMETRY_H
