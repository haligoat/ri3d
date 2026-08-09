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
constexpr int8_t ODOM_YAW_SIGN = +1;


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
constexpr float ODOM_MAX_FORWARD_SPEED = 2.0f;
constexpr float ODOM_MAX_STRAFE_SPEED  = 1.4f;

//  Time to reach ~63% of a commanded step change, seconds. Mass and gearing
//  dominate this. Lower it if the estimate lags real motion.
constexpr float ODOM_MOTOR_TAU = 0.25f;

//  Command percent below which the robot does not actually move -- static
//  friction plus motor-driver deadzone.
constexpr float ODOM_DEADBAND_PCT = 8.0f;


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
