#include <EchoLib.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "src/odometry/Odometry.h"
#include "src/odometry/OdomCalibration.h"
#include "src/odometry/AutoDrive.h"

// ==== FILL THESE IN with your home WiFi credentials before uploading ====
const char* WIFI_SSID     = "laroda-home";
const char* WIFI_PASSWORD = "alex2009";
// ==========================================================================

#define UDP_PORT 8888

// Advertised to the router, so the robot shows up by name in the DHCP lease
// table instead of as an anonymous address you have to hunt for.
#define WIFI_HOSTNAME "echo-robot"

// One join attempt is given this long before we tear down and retry.
#define WIFI_JOIN_TIMEOUT_MS 15000

// While disconnected, how often loop() retries the join. The retry is
// non-blocking -- the robot stays responsive on serial the whole time.
#define WIFI_RETRY_INTERVAL_MS 5000

// Set to 1 to boot into the guided odometry calibration menu instead of the
// normal robot code. See README_ODOMETRY.md.
#define ODOM_CALIBRATION_MODE 0

// How often to push a pose packet back to the driver station, milliseconds.
#define TELEMETRY_INTERVAL_MS 100

// A client (the Pi) is considered connected if we've heard from it within
// this many ms. Mirrors WiFiServerBridge's own default timeout.
#define CLIENT_TIMEOUT_MS 2000

MotorControllers motors;
MecanumDrive driver(motors, 1, 3, 4, 2); // fl, fr, bl, br
IMU imu;
MecanumOdometry odom(imu);
AutoDrive auto_(driver, odom);

WiFiUDP udp;
IPAddress lastClientIP;
uint16_t lastClientPort = 0;
unsigned long lastPacketTime = 0;
static unsigned long lastTelemetry = 0;

bool clientConnected() {
  return lastClientPort != 0 && (millis() - lastPacketTime) < CLIENT_TIMEOUT_MS;
}

void sendToClient(const String& msg) {
  if (!clientConnected()) return;
  udp.beginPacket(lastClientIP, lastClientPort);
  udp.print(msg);
  udp.endPacket();
}

// Motor 5 is not part of the drivetrain (MecanumDrive owns 1-4), so it is
// driven straight from the controller's right bumper: full speed while held,
// off the moment it is released.
#define AUX_MOTOR_ID      MOTOR_5_ID
#define AUX_MOTOR_PERCENT 100

static bool auxMotorOn = false;

// Cheap to call every packet -- only touches the hardware on an actual change.
static void setAuxMotor(bool on) {
  if (on == auxMotorOn) return;
  auxMotorOn = on;
  motors.set(AUX_MOTOR_ID, on ? AUX_MOTOR_PERCENT : 0);
  Serial.print("AUX motor ");
  Serial.println(on ? "ON" : "OFF");
}

// AutoDrive's moves block, so loop() is not running while one executes.
// Without this hook nothing would service the network or honour the kill
// switch for the whole duration of a move. Returning true stops the move
// immediately.
static bool autoAbort() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    lastClientIP = udp.remoteIP();
    lastClientPort = udp.remotePort();
    lastPacketTime = millis();
    char buf[64];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len > 0 ? len : 0] = '\0';
    if (String(buf).equals("x")) return true; // kill switch
  }
  if (!clientConnected()) return true; // link dropped
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

// Blocks until joined or WIFI_JOIN_TIMEOUT_MS elapses. Returns whether we got
// on. Safe to call repeatedly -- each attempt starts from a clean radio state,
// which matters because a half-finished association will otherwise sit there
// reporting WL_DISCONNECTED forever.
static bool joinWiFi() {
  WiFi.disconnect(true);   // drop any stale association and clear the old config
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);   // must precede begin() to be sent in the DHCP request
  // Modem sleep MUST stay on. The Bluepad32 core brings up BTstack at boot, and
  // WiFi and Bluetooth share one radio -- ESP-IDF aborts on purpose if you try
  // to run both with modem sleep disabled. Turning it off costs some control
  // latency, but it boot-loops the robot.
  WiFi.setSleep(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_JOIN_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.print("Connected as ");
  Serial.print(WIFI_HOSTNAME);
  Serial.print(" -- IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  return true;
}

// Called every loop(). If the link has dropped, retries the join on a timer
// rather than wedging the robot. Cheap when already connected.
static void serviceWiFi() {
  static unsigned long lastAttempt = 0;

  if (WiFi.status() == WL_CONNECTED) return;

  if (millis() - lastAttempt < WIFI_RETRY_INTERVAL_MS) return;
  lastAttempt = millis();

  // The link is down, so the driver station cannot be reached -- stop the
  // robot before spending time on the retry.
  driver.setBrake();
  setAuxMotor(false);
  lastClientPort = 0;

  Serial.println("!! WiFi link down -- retrying join");
  if (joinWiFi()) {
    udp.begin(UDP_PORT);   // the socket does not survive a reassociation
  }
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

  Serial.print("=== BOOT: joining WiFi (");
  Serial.print(WIFI_SSID);
  Serial.println(") ===");

  if (!joinWiFi()) {
    // Not fatal. loop() keeps retrying, so a router that is still coming up
    // (or a robot powered on out of range) recovers on its own instead of
    // needing a reboot.
    Serial.println("!! Failed to join WiFi. Check WIFI_SSID/WIFI_PASSWORD, and");
    Serial.println("!! confirm the network has a 2.4GHz band -- the ESP32 cannot see 5GHz.");
    Serial.println("!! Will keep retrying in the background.");
  }

  udp.begin(UDP_PORT);
  auto_.setAbortCheck(autoAbort);

  driver.setBrake();
  Serial.println("=== BOOT COMPLETE ===");
  Serial.print("Send 'x,y,turn' UDP packets to the IP above, port ");
  Serial.println(UDP_PORT);
}

void loop() {
  serviceWiFi();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    lastClientIP = udp.remoteIP();
    lastClientPort = udp.remotePort();
    lastPacketTime = millis();

    char buf[64];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len > 0 ? len : 0] = '\0';
    String data(buf);

    if (data.equals("auto")) {
      runAuto();
      driver.drive(0, 0, 0);
      odom.setCommand(0, 0, 0);
      return;
    }

    if (data.length() == 1 && data.equals("x")) {
      driver.drive(0, 0, 0);
      setAuxMotor(false);   // the kill switch has to kill everything, not just the drivetrain
      odom.setCommand(0, 0, 0);
      odom.update();
      if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
        lastTelemetry = millis();
        sendToClient("ODOM," + odom.telemetry());
      }
      return;
    }

    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    // Optional 4th field: aux motor flag. Older clients that send only
    // "x,y,turn" still parse correctly and simply leave motor 5 off.
    int thirdComma = data.indexOf(',', secondComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      int x = data.substring(0, firstComma).toInt();
      int y = data.substring(firstComma + 1, secondComma).toInt();
      int turn = (thirdComma > secondComma)
                   ? data.substring(secondComma + 1, thirdComma).toInt()
                   : data.substring(secondComma + 1).toInt();

      setAuxMotor(thirdComma > secondComma &&
                  data.substring(thirdComma + 1).toInt() != 0);

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

  if (!clientConnected()) {
    driver.drive(0, 0, 0);  // safety stop if client disconnects
    setAuxMotor(false);     // a held bumper must not latch on when the link dies
    odom.setCommand(0, 0, 0);
  }

  // Runs every loop; the filter self-limits to its own update interval.
  odom.update();

  if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = millis();
    sendToClient("ODOM," + odom.telemetry());
  }
}
