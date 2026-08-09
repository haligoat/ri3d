#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <EchoLib.h>
#include "OdomConstants.h"

// ============================================================================
//  MecanumOdometry -- pose tracking with no wheel encoders.
//
//  Heading comes from the gyro. Position comes from integrating the velocity
//  we ASKED the drivetrain for, run through the mecanum kinematics and a
//  first-order motor lag. That's the whole thing.
//
//  WHY SO SIMPLE
//  -------------
//  A Kalman filter fusing the accelerometer was tried first and dropped (it is
//  preserved in git history if you want it back). The reason is structural, not
//  a tuning failure: an accelerometer measures only CHANGES in velocity, while
//  the errors that actually dominate here -- a mis-measured top speed, steady
//  wheel slip -- are near-constant offsets that produce no acceleration
//  signature at all. Fusing it in cost complexity and bought almost nothing.
//
//  So the accelerometer is not read at all. Add encoders and that calculus
//  changes completely; the filter is worth revisiting at that point.
//
//  WHAT THIS IS GOOD FOR
//    "Where am I relative to where the match started", over tens of seconds.
//  WHAT IT IS NOT
//    Reliable through wheel slip, collisions, or being pushed. Error only
//    accumulates -- there is no absolute reference to correct against.
//
//  FRAMES
//    Body:  +forward is the nose, +right its right side -- matching
//           MecanumDrive::drive(x, y, turn) where y = forward, x = right.
//    Field: fixed at begin(). theta = 0 means the nose points along field +X;
//           theta increases counter-clockwise, so field +Y is robot-left.
// ============================================================================

struct OdomConfig {
    // See OdomConstants.h -- that is where these live and are explained.
    int8_t yawSign;
    float  maxForwardSpeed;
    float  maxStrafeSpeed;
    float  motorTau;
    float  deadbandPct;

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

    // Call once in setup(), after imu.begin(). Latches the current heading as
    // theta = 0, so a fixed IMU mounting rotation cancels itself out.
    void begin();

    // Zero the pose. Safe mid-match.
    void reset();

    // Mirror of MecanumDrive::drive(). Call with the SAME arguments every time
    // you call drive(), including the zeros on a safety stop -- this IS the
    // position estimate, so a stale command means an invented position.
    void setCommand(int x, int y, int turn);

    // Run one step. Call every loop(); self-limits to updateIntervalMs.
    void update();

    Pose  getPose()     const { return pose; }
    Twist getVelocity() const;

    float getX()        const { return pose.x; }
    float getY()        const { return pose.y; }
    float getThetaRad() const { return pose.theta; }
    float getThetaDeg() const { return pose.theta * 57.2957795f; }
    float getSpeed()    const;

    // True when we are commanding no motion and the modelled velocity has
    // decayed to ~zero. This is a statement about the COMMAND, not a sensor
    // reading -- a robot being pushed while parked still reports true.
    bool  isStationary() const;

    // Force the pose to a known value -- a field landmark, a wall alignment,
    // the start of an auto routine. The only way to remove accumulated error.
    void setPose(float x, float y, float thetaRad);

    void setUpdateInterval(uint16_t ms) { updateIntervalMs = ms; }
    OdomConfig& config() { return cfg; }

    // "x,y,thetaDeg,vx,vy,stationary" -- ready for WiFiServerBridge.
    String telemetry() const;

private:
    void  updateHeading();
    void  updateModelVelocity(float dt);

    IMU& imu;
    OdomConfig cfg;

    Pose pose;

    // Heading bookkeeping
    float headingRaw;      // unwrapped gyro heading, radians
    float headingOffset;   // latched at begin()
    float lastRawDeg;      // for unwrapping the 0..360 gyro angle
    float yawRate;         // rad/s

    // Commanded-velocity model, body frame
    int   cmdX, cmdY, cmdTurn;
    float modelVFwd, modelVRight;

    uint32_t lastUpdateUs;
    uint32_t lastStepMs;
    uint16_t updateIntervalMs;
    bool     started;
};

#endif // ODOMETRY_H
