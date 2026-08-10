#ifndef ODOM_CONSTANTS_H
#define ODOM_CONSTANTS_H

#include <stdint.h>

// ============================================================================
//  ODOMETRY CONSTANTS -- the only file you should need to edit.
//
//  Run the calibration (set ODOM_CALIBRATION_MODE to 1 in controller.ino) and
//  it prints lines in exactly this format. Replace the matching line below,
//  then reflash. See README_ODOMETRY.md.
//
//  These are properties of YOUR robot. The shipped values are placeholders
//  chosen to be plausible, not measurements -- treat any pose reading as
//  meaningless until you have run the calibration.
// ============================================================================


// ---------------------------------------------------------------------------
//  1. HEADING                 [calibration test 1]
// ---------------------------------------------------------------------------
//  +1 if the gyro's Z angle INCREASES when the robot turns counter-clockwise
//  (to its left). -1 otherwise.
//
//  Assumes the IMU is mounted FLAT, so its Z axis is the yaw axis. A flat board
//  rotated in yaw needs no correction at all -- odom.begin() latches whatever
//  heading it reads at boot as theta = 0, absorbing any fixed mounting angle.
constexpr int8_t ODOM_YAW_SIGN = -1;


// ---------------------------------------------------------------------------
//  2. DRIVETRAIN              [calibration tests 2, 3, 4]
// ---------------------------------------------------------------------------
//  This section IS the position estimate -- get it wrong and every distance is
//  wrong by the same ratio. It matters more than anything else here.

//  Wheel diameter, meters. Measured: 67 mm mecanum wheels.
constexpr float ODOM_WHEEL_DIAMETER_M = 0.067f;

//  Speed at 100% command, meters/second.
//
//  Cross-check with odomWheelSpeedFromRpm() below before believing a tape
//  measure. 2.0 m/s on 67 mm wheels implies ~570 RPM at the output shaft, which
//  is fast for a geared chassis -- most builds land nearer 0.8-1.5 m/s. If
//  measured and computed disagree by more than ~25%, suspect a gear ratio, a
//  backwards motor, or a sagging battery.
//
//  Strafing is always slower: mecanum rollers waste much of the force
//  sideways. Expect roughly 0.5-0.8x of forward.
constexpr float ODOM_MAX_FORWARD_SPEED = 1.28f;
constexpr float ODOM_MAX_STRAFE_SPEED  = 0.98f;

//  Time to reach ~63% of a commanded step change, seconds. Mass and gearing
//  dominate this. Lower it if the estimate lags real motion.
constexpr float ODOM_MOTOR_TAU = 0.25f;

//  Command percent below which the robot does not actually move -- static
//  friction plus motor-driver deadzone.
constexpr float ODOM_DEADBAND_PCT = 18.0f;


// ---------------------------------------------------------------------------
//  3. AUTONOMOUS              (AutoDrive.h)
// ---------------------------------------------------------------------------
//  Which sign of the `turn` argument to MecanumDrive::drive() rotates the robot
//  COUNTER-CLOCKWISE, i.e. in the direction that increases getThetaDeg().
//
//  From EchoLib's mixing (FL = y+x+turn, FR = y-x-turn) a positive turn drives
//  the left wheels forward and the right wheels back, which is a CLOCKWISE
//  spin -- hence -1. But that assumes your motors are wired and numbered the
//  way the constructor says, so treat it as a starting guess.
//
//  You do NOT have to get this right by reasoning: a turn that runs away is
//  detected and aborted, and the serial output tells you to flip this.
constexpr int8_t AUTO_TURN_SIGN = -1;

//  Heading-hold gains. Percent of drive command per degree of error, and per
//  degree/second of error rate. Raise P if it wanders off heading; raise D if
//  it oscillates around the target.
constexpr float AUTO_HEADING_KP = 1.2f;
constexpr float AUTO_HEADING_KD = 0.08f;

//  Distance controller: percent of drive command per meter of remaining
//  distance. This is what decelerates the robot into the target.
constexpr float AUTO_DISTANCE_KP = 220.0f;

//  How close counts as arrived.
constexpr float AUTO_DISTANCE_TOLERANCE = 0.04f;   // meters
constexpr float AUTO_ANGLE_TOLERANCE    = 3.0f;    // degrees

//  Minimum command actually sent while a move is still running. Below the
//  drivetrain deadband the robot stalls short of the target and sits there
//  buzzing until the timeout, so the controller floors its output here.
constexpr float AUTO_MIN_DRIVE_PCT = 18.0f;
constexpr float AUTO_MIN_TURN_PCT  = 16.0f;

//  Ceiling on the heading correction mixed into a straight drive, percent.
//  Keeps a heading fight from overwhelming forward motion.
constexpr float AUTO_MAX_HEADING_CORRECTION = 25.0f;

//  A move must hold its tolerance this long before it counts as finished --
//  stops a fast approach from declaring victory as it flies past.
constexpr uint32_t AUTO_SETTLE_MS = 150;

//  Safety: if a turn's error grows by this many degrees instead of shrinking,
//  AUTO_TURN_SIGN is backwards. The move aborts rather than spinning forever.
constexpr float AUTO_RUNAWAY_DEG = 25.0f;


// ---------------------------------------------------------------------------
//  Helper: wheel surface speed from output-shaft RPM.
//  Pass geared output RPM (free speed / gear ratio) and derate ~0.75 for load;
//  a motor under a robot never reaches its free speed.
// ---------------------------------------------------------------------------
static inline float odomWheelSpeedFromRpm(float outputRpm,
                                          float wheelDiameterM = ODOM_WHEEL_DIAMETER_M) {
    return (outputRpm / 60.0f) * 3.14159265f * wheelDiameterM;
}

#endif // ODOM_CONSTANTS_H
