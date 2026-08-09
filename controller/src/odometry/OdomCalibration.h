#ifndef ODOM_CALIBRATION_H
#define ODOM_CALIBRATION_H

#include <EchoLib.h>
#include "Odometry.h"

// Guided, serial-menu calibration for MecanumOdometry. Blocks forever -- it is
// meant to be run instead of the normal robot code, not alongside it.
// Enable by setting ODOM_CALIBRATION_MODE to 1 in controller.ino.
//
// THE ROBOT DRIVES ITSELF during tests 2-4. Put it on the floor with clear
// space ahead and keep a hand near the power switch.
void runOdometryCalibration(IMU& imu, MecanumDrive& drive, MecanumOdometry& odom);

#endif // ODOM_CALIBRATION_H
