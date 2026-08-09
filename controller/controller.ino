#include <EchoLib.h>
#include <WiFi.h>
#include "src/odometry/Odometry.h"
#include "src/odometry/OdomCalibration.h"
#include "src/odometry/AutoDrive.h"

// Set to 1 to boot into the guided odometry calibration menu instead of the
// normal robot code. See README_ODOMETRY.md.
#define ODOM_CALIBRATION_MODE 0

// How often to push a pose packet back to the driver station, milliseconds.
#define TELEMETRY_INTERVAL_MS 100

WiFiServerBridge server("Zippy", "robot2024", 8888);
MotorControllers motors;
MecanumDrive driver(motors, 1, 3, 4, 2); // fl, fr, bl, br
IMU imu;
MecanumOdometry odom(imu);
AutoDrive auto_(driver, odom);

static unsigned long lastTelemetry = 0;

// AutoDrive's moves block, so loop() is not running while one executes. Without
// this hook nothing would service the network or honour the kill switch for the
// whole duration of a move. Returning true stops the move immediately.
static bool autoAbort() {
  server.processIncoming();
  if (!server.getStatus()) return true;              // link dropped
  String d = server.readData();
  if (d.length() == 1 && d.equals("x")) return true; // kill switch
  return false;
}

// Example routine. Every call is checked: if a move fails (timeout, abort, or a
// runaway turn) the rest of the sequence is skipped rather than run from a pose
// we know is wrong.
static void runAuto() {
  Serial.println("AUTO: start");
  odom.reset();                       // this pose is now the origin

  if (!auto_.driveDistance(1.0, 40)) return;
  if (!auto_.turnToAngle(90, 35))     return;
  if (!auto_.driveDistance(0.5, 40))  return;
  if (!auto_.turnToAngle(0, 35))      return;

  Serial.println("AUTO: done");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BOOT: starting IMU ===");

  // Must be still for this -- begin() calibrates the gyro. The accelerometer
  // is never read by the odometry, so it needs no calibration.
  imu.begin();

  odom.begin();
  Serial.println("=== IMU + odometry ready ===");

#if ODOM_CALIBRATION_MODE
  driver.setBrake();
  runOdometryCalibration(imu, driver, odom);  // never returns
#endif

  Serial.println("=== BOOT: starting WiFi access point ===");

  server.enableDebug(true);
  server.setTimeoutConnectionDependent(true);
  server.begin();

  auto_.setAbortCheck(autoAbort);

  driver.setBrake();
  Serial.println("=== BOOT COMPLETE ===");

  Serial.print("WiFi mode: ");
  Serial.println(WiFi.getMode());
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP SSID: ");
  Serial.println(WiFi.softAPSSID());
  Serial.print("Station count: ");
  Serial.println(WiFi.softAPgetStationNum());
}

void loop() {
  server.processIncoming();

  if (!server.getStatus()) {
    driver.drive(0, 0, 0);  // safety stop if client disconnects
    odom.setCommand(0, 0, 0);
    odom.update();          // keep tracking pose so the estimate survives a dropout
    return;
  }

  String data = server.readData();

  if (data.equals("auto")) {
    runAuto();
    driver.drive(0, 0, 0);
    odom.setCommand(0, 0, 0);
    return;
  }

  if (data.length() == 1 && data.equals("x")) {
    driver.drive(0, 0, 0);  // stop the wheels before aborting
    raise(9);
  }

  if (data.length() > 0) {
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      int x = data.substring(0, firstComma).toInt();
      int y = data.substring(firstComma + 1, secondComma).toInt();
      int turn = data.substring(secondComma + 1).toInt();

      Serial.print("Driving x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.print(y);
      Serial.print(" turn=");
      Serial.println(turn);

      driver.drive(x, y, turn);
      // The filter's motion model is built from the commands we send, so this
      // has to track every drive() call or the model silently goes stale.
      odom.setCommand(x, y, turn);
    }
  }

  // Runs every loop; the filter self-limits to its own update interval.
  odom.update();

  if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = millis();
    server.sendData("ODOM," + odom.telemetry());
  }
}
