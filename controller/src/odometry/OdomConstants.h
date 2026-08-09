#ifndef ODOM_CONSTANTS_H
#define ODOM_CONSTANTS_H

#include <stdint.h>

// ============================================================================
//  ODOMETRY CONSTANTS -- this is the only file you should need to edit.
//
//  Run the calibration (set ODOM_CALIBRATION_MODE to 1 in controller.ino) and
//  it prints lines in exactly this format. Replace the matching line below with
//  what it printed, then reflash. See README_ODOMETRY.md.
//
//  Everything here is a property of YOUR robot. The values shipped are
//  placeholders chosen to be plausible, not measurements -- treat any pose
//  reading as meaningless until you have run at least sections 1 and 2.
// ============================================================================


// ---------------------------------------------------------------------------
//  1. IMU MOUNTING            [calibration test 1]
// ---------------------------------------------------------------------------
//  Which accelerometer axis points out the FRONT of the robot, and out its
//  RIGHT side.  Axis: 0 = X, 1 = Y, 2 = Z.  Sign: +1 or -1.
//
//  Assumes the IMU is mounted FLAT (Z axis vertical), which is what makes gyro
//  Z the yaw axis. Mounted on its side, the gravity compensation is wrong and
//  no amount of tuning will save it -- remount it flat.
//
//  Getting a sign wrong here makes the estimated position run BACKWARDS. It
//  looks like a broken filter; it is really a mounting question.

constexpr uint8_t ODOM_FORWARD_AXIS = 1;
constexpr int8_t  ODOM_FORWARD_SIGN = +1;
constexpr uint8_t ODOM_RIGHT_AXIS   = 0;
constexpr int8_t  ODOM_RIGHT_SIGN   = +1;

//  +1 if the gyro's Z angle INCREASES when the robot turns counter-clockwise.
constexpr int8_t  ODOM_YAW_SIGN     = +1;


// ---------------------------------------------------------------------------
//  2. DRIVETRAIN              [calibration tests 2, 3, 4]
// ---------------------------------------------------------------------------
//  Wheel diameter, meters. Measured: 67 mm mecanum wheels.
constexpr float ODOM_WHEEL_DIAMETER_M = 0.067f;

//  Speed at 100% command, meters/second.
//
//  Cross-check against the motor spec with odomWheelSpeedFromRpm() below before
//  believing a tape measure. 2.0 m/s on 67 mm wheels implies ~570 RPM at the
//  output shaft, which is fast for a geared chassis -- most builds land nearer
//  0.8-1.5 m/s. If measured and computed disagree by more than ~25%, suspect a
//  gear ratio, a backwards motor, or a sagging battery.
//
//  Strafing is always slower than driving: mecanum rollers waste much of the
//  force sideways. Expect roughly 0.5-0.8x of forward.
constexpr float ODOM_MAX_FORWARD_SPEED = 2.0f;
constexpr float ODOM_MAX_STRAFE_SPEED  = 1.4f;

//  Time to reach ~63% of a commanded step change, seconds. Mass and gearing
//  dominate this.
constexpr float ODOM_MOTOR_TAU = 0.25f;

//  Command percent below which the robot does not actually move -- static
//  friction plus motor-driver deadzone.
constexpr float ODOM_DEADBAND_PCT = 8.0f;


// ---------------------------------------------------------------------------
//  3. FILTER TUNING
// ---------------------------------------------------------------------------
//  Only touch these after sections 1 and 2 are measured, and change one at a
//  time. README_ODOMETRY.md has a symptom-to-knob table.

//  Accelerometer white noise, m/s^2, 1-sigma. Deliberately far above the
//  BMI270 datasheet figure: chassis vibration, not sensor noise, is what
//  actually corrupts this signal. Raise it if the pose jitters on rough floor.
constexpr float ODOM_ACCEL_NOISE = 0.6f;

//  Accelerometer bias random-walk, m/s^2 per sqrt(second).
constexpr float ODOM_ACCEL_BIAS_WALK = 0.02f;

//  How much to trust the commanded-velocity model, m/s. Lower = trust the
//  wheels more.
constexpr float ODOM_MODEL_VEL_NOISE = 0.35f;

//  Extra model distrust per rad/s of rotation -- mecanum wheels scrub while
//  turning, so the model is worse there.
constexpr float ODOM_MODEL_VEL_NOISE_TURN = 0.25f;

//  How much to trust a detected zero-velocity, m/s. Small on purpose: when we
//  are confident the robot is parked, that is the best measurement available.
constexpr float ODOM_ZUPT_NOISE = 0.01f;


// ---------------------------------------------------------------------------
//  4. SLIP / COLLISION REJECTION
// ---------------------------------------------------------------------------
//  Disagreement between the model and the accelerometer (m/s) at which the
//  model's variance doubles. Lower = quicker to distrust the wheels.
constexpr float ODOM_SLIP_SCALE = 0.6f;

//  Seconds before the accelerometer-only velocity is bled back toward the
//  filter, bounding its own drift. Lower if position creeps during long pushes.
constexpr float ODOM_SLIP_MEMORY = 4.0f;


// ---------------------------------------------------------------------------
//  5. STATIONARY (ZUPT) DETECTION
// ---------------------------------------------------------------------------
//  Zero-velocity updates are what make accelerometer bias observable, and are
//  the single biggest accuracy win in the system. If calibration test 5 reports
//  that ZUPT never engaged, raise the first two -- a vibrating chassis can sit
//  above these defaults, and then nothing else works properly.

constexpr float    ODOM_ZUPT_ACCEL_THRESH    = 0.45f;  // m/s^2
constexpr float    ODOM_ZUPT_YAW_RATE_THRESH = 0.06f;  // rad/s

//  Max accelerometer-only speed still counted as "parked", m/s. This is what
//  lets ZUPT fire while the driver is holding full throttle -- the wall-jam
//  case. Too high and a slow-moving robot falsely believes it is parked.
constexpr float    ODOM_ZUPT_VEL_THRESH      = 0.18f;  // m/s

//  How long the above must hold before we trust it, milliseconds.
constexpr uint32_t ODOM_ZUPT_HOLD_MS         = 120;


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
