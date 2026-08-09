#ifndef AUTODRIVE_H
#define AUTODRIVE_H

#include <Arduino.h>
#include <EchoLib.h>
#include "Odometry.h"
#include "OdomConstants.h"

// ============================================================================
//  AutoDrive -- blocking movement primitives for autonomous routines.
//
//    auto_.driveDistance(1.5);    // 1.5 m forward, holding heading
//    auto_.turnToAngle(90);       // face 90 deg (absolute field heading)
//    auto_.driveDistance(-0.5);   // half a metre backwards
//
//  UNITS: meters, degrees, percent. Same as the odometry -- getX()/getY() are
//  meters and getThetaDeg() is degrees, so there is one unit system, not two.
//
//  Angles are ABSOLUTE field headings, where 0 is whatever direction the robot
//  faced at odom.begin(). Absolute rather than relative so errors don't
//  accumulate down a sequence: three turnToAngle(90) calls leave you at 90,
//  whereas three turnBy(90) calls compound whatever each one got wrong.
//
//  ACCURACY. Turns are closed-loop on the gyro and are the accurate half of
//  this. Distances are dead reckoning from the commanded velocity, so they are
//  only as good as ODOM_MAX_FORWARD_SPEED and only true while the wheels grip.
//  Keep segments short, drive gently, and re-zero with odom.setPose() against a
//  wall when you can. Slip is invisible here -- if the robot is blocked, a move
//  will happily "complete" having gone nowhere.
//
//  SAFETY. These block until the move finishes. While one runs, your loop() is
//  not running, so nothing is servicing the WiFi link or your kill switch
//  unless you give it an abort hook -- see setAbortCheck(). Do that before
//  running anything that moves.
// ============================================================================

// Called repeatedly during a blocking move. Return true to stop immediately.
typedef bool (*AutoAbortFn)();

class AutoDrive {
public:
    AutoDrive(MecanumDrive& drive, MecanumOdometry& odom);

    // Strongly recommended. Service the network and check your kill switch in
    // here; returning true stops the current move and reports failure.
    void setAbortCheck(AutoAbortFn fn) { abortFn = fn; }

    // All return true if the target was reached, false on timeout or abort.
    // A false return means the pose is NOT where you asked -- check it.

    // Drive along the current heading. Negative goes backwards.
    bool driveDistance(float meters, int speedPct = 50);

    // Strafe sideways. Positive is to the robot's right.
    bool strafeDistance(float meters, int speedPct = 50);

    // Rotate to an absolute field heading.
    bool turnToAngle(float headingDeg, int speedPct = 40);

    // Rotate by a relative amount. Prefer turnToAngle where you can.
    bool turnBy(float deltaDeg, int speedPct = 40);

    // Cut the motors and tell the odometry we did.
    void stop();

    void setDistanceTolerance(float meters) { distTol = meters; }
    void setAngleTolerance(float degrees)   { angTol  = degrees; }

    // 0 = derive a timeout from the distance and speed requested (default).
    void setTimeout(uint32_t ms) { timeoutMs = ms; }

private:
    bool driveAxis(float meters, int speedPct, bool strafe);
    bool waitSettled(uint32_t& settledSince, bool within);
    void applyDrive(int x, int y, int turn);
    bool aborted(const char* what);
    float headingCorrection(float targetDeg, float dt);

    MecanumDrive& drive;
    MecanumOdometry& odom;
    AutoAbortFn abortFn;

    float distTol;
    float angTol;
    uint32_t timeoutMs;
    float lastHeadingError;
};

// Shortest signed difference between two headings, in degrees (-180, 180].
// Exposed because it is the single easiest thing to get wrong in an auto.
float wrapDegrees180(float deg);

#endif // AUTODRIVE_H
