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

// ---- Status LEDs ----------------------------------------------------------
// A driver looking at the robot needs to know the link state without reading a
// screen. "Connected" means the whole chain is up: WiFi associated AND the
// driver station heard from recently. WiFi alone is not enough -- an
// associated board with a dead client cannot be driven.
//
// Both LEDs are driven every loop rather than only on transitions, so a glitch
// can never leave them lying about the state.
#define LED_CONNECTED_PIN    42
#define LED_DISCONNECTED_PIN 41

#if ODOM_CALIBRATION_MODE
// runOdometryCalibration() never returns, so loop() -- and therefore
// serviceStatusLeds() -- never runs in calibration mode. A blink driven from
// the main flow is impossible. This task owns the LED instead, so it keeps
// flashing no matter how long the calibration routine blocks.
#define CALIB_BLINK_MS 250

static void calibrationBlinkTask(void* /*unused*/) {
  for (;;) {
    digitalWrite(LED_CONNECTED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(CALIB_BLINK_MS));
    digitalWrite(LED_CONNECTED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(CALIB_BLINK_MS));
  }
}
#endif

static void serviceStatusLeds() {
  const bool linkUp = (WiFi.status() == WL_CONNECTED) && clientConnected();
  digitalWrite(LED_CONNECTED_PIN,    linkUp ? HIGH : LOW);
  digitalWrite(LED_DISCONNECTED_PIN, linkUp ? LOW  : HIGH);
}

// ---- Teleop heading hold (PID) --------------------------------------------
// Closes a loop around the gyro so the robot drives the heading you left it
// on. Without this, driving "straight" is open-loop hope: uneven motors, a
// dragging wheel or carpet nap all yaw the robot slowly and the driver spends
// the match correcting by hand.
//
// This controls HEADING, not speed, and that is not a stylistic choice --
// there are no encoders (Motor is open-loop MCPWM), so commanded velocity is
// the only "velocity" available. A PID on that would be feeding back its own
// output. Heading is the one quantity actually measured, via the gyro.
//
// Only active while the driver is NOT commanding a turn. The moment the turn
// stick moves, the driver owns the heading and we re-latch the target to
// wherever they leave it.
#define HEADING_HOLD_ENABLED   1
#define HEADING_KP             1.6f
#define HEADING_KI             0.0f   // start at zero; see the note below
#define HEADING_KD             0.10f
#define HEADING_MAX_CORRECTION 35     // percent turn command, clamped
#define HEADING_DEADBAND_DEG   1.0f   // ignore noise; do not chase the gyro
#define HEADING_I_LIMIT        10.0f  // anti-windup clamp on the I accumulator

static float headingTarget   = 0.0f;
static bool  headingHoldArmed = false;
static float headingErrSum   = 0.0f;
static float headingLastErr  = 0.0f;
static unsigned long headingLastMs = 0;

// Shortest signed distance between two headings, in degrees.
static float headingError(float target, float actual) {
  float e = target - actual;
  while (e > 180.0f)  e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

static void releaseHeadingHold() {
  headingHoldArmed = false;
  headingErrSum = 0.0f;
  headingLastErr = 0.0f;
}

// Returns the turn command to actually send: the driver's own turn when they
// are steering, or a correction when they are not.
static int applyHeadingHold(int x, int y, int turn) {
#if !HEADING_HOLD_ENABLED
  return turn;
#else
  const bool driverSteering = (turn != 0);
  const bool moving         = (x != 0 || y != 0);

  if (driverSteering || !moving) {
    // Driver owns the heading, or we are parked. Re-latch continuously so the
    // target is current the instant they let go.
    headingTarget = odom.getThetaDeg();
    releaseHeadingHold();
    return turn;
  }

  unsigned long now = millis();

  if (!headingHoldArmed) {
    // First loop of a hold: latch the target and seed the derivative, so the
    // D term does not spike off a bogus first sample.
    headingTarget = odom.getThetaDeg();
    headingHoldArmed = true;
    headingLastErr = 0.0f;
    headingErrSum = 0.0f;
    headingLastMs = now;
    return 0;
  }

  float dt = (now - headingLastMs) / 1000.0f;
  headingLastMs = now;
  if (dt <= 0.0f || dt > 0.5f) {
    // A stalled or absurd dt means a hiccup in the link; skip this sample
    // rather than feeding a garbage derivative into the loop.
    return 0;
  }

  float err = headingError(headingTarget, odom.getThetaDeg());

  if (fabsf(err) < HEADING_DEADBAND_DEG) {
    headingLastErr = err;
    return 0;
  }

  headingErrSum += err * dt;
  headingErrSum = constrain(headingErrSum, -HEADING_I_LIMIT, HEADING_I_LIMIT);

  float derivative = (err - headingLastErr) / dt;
  headingLastErr = err;

  float output = HEADING_KP * err
               + HEADING_KI * headingErrSum
               + HEADING_KD * derivative;

  return (int)constrain(output, -(float)HEADING_MAX_CORRECTION,
                                 (float)HEADING_MAX_CORRECTION);
#endif
}

// ---- Active braking -------------------------------------------------------
// Cutting power to a mecanum drive leaves it coasting on its own momentum. To
// actually stop, we drive the motors backwards briefly ("plugging") and then
// release. The pulse is triggered on a release-to-zero or a direction reversal.
//
// This is done here rather than on the driver station because it is timing
// critical: the control link runs at 90-800ms of jitter, so a reverse pulse
// scheduled from the Pi would land at an unpredictable moment. On the board it
// is deterministic.
//
// BRAKE_SCALE is a percentage of the speed the robot was doing. Too high and
// the robot snaps backwards; too low and it still drifts. Start conservative.
#define BRAKE_SCALE   60    // percent of previous command, applied in reverse
#define BRAKE_MS      70    // how long to hold the reverse pulse
// Commands smaller than this were not carrying enough momentum to be worth
// braking, and pulsing on them just makes the robot feel twitchy.
#define BRAKE_MIN_CMD 20

static int lastX = 0, lastY = 0, lastTurn = 0;
static unsigned long brakeUntil = 0;
static int brakeX = 0, brakeY = 0, brakeTurn = 0;
static int pendingX = 0, pendingY = 0, pendingTurn = 0;

// True when an axis is being told to stop or to flip direction while it was
// carrying real speed.
static bool needsBrake(int prev, int next) {
  if (abs(prev) < BRAKE_MIN_CMD) return false;
  if (next == 0) return true;                       // released
  return (prev > 0) != (next > 0);                  // reversed
}

static int brakeComponent(int prev, int next) {
  return needsBrake(prev, next) ? -(prev * BRAKE_SCALE) / 100 : next;
}

// Single entry point for every drive command, so odometry and the brake state
// machine cannot drift out of sync with what the motors are actually doing.
static void applyDrive(int x, int y, int turn) {
  if (needsBrake(lastX, x) || needsBrake(lastY, y) || needsBrake(lastTurn, turn)) {
    brakeX    = brakeComponent(lastX, x);
    brakeY    = brakeComponent(lastY, y);
    brakeTurn = brakeComponent(lastTurn, turn);
    pendingX = x; pendingY = y; pendingTurn = turn;
    brakeUntil = millis() + BRAKE_MS;

    driver.drive(brakeX, brakeY, brakeTurn);
    odom.setCommand(brakeX, brakeY, brakeTurn);
  } else {
    // Heading correction is applied to the OUTPUT only. The brake state below
    // still tracks the driver's own turn, so a small PID correction can never
    // look like a direction reversal and trigger a spurious brake pulse.
    int outTurn = applyHeadingHold(x, y, turn);
    driver.drive(x, y, outTurn);
    odom.setCommand(x, y, outTurn);
  }

  lastX = x; lastY = y; lastTurn = turn;
}

// Ends the reverse pulse on time. Called every loop, so the pulse length does
// not depend on when the next packet happens to arrive.
static void serviceBrake() {
  if (brakeUntil == 0 || millis() < brakeUntil) return;
  brakeUntil = 0;
  driver.drive(pendingX, pendingY, pendingTurn);
  odom.setCommand(pendingX, pendingY, pendingTurn);
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

  // Set up before anything can block, so the disconnected LED is lit for the
  // whole boot rather than coming on only once WiFi has failed.
  pinMode(LED_CONNECTED_PIN, OUTPUT);
  pinMode(LED_DISCONNECTED_PIN, OUTPUT);
  digitalWrite(LED_CONNECTED_PIN, LOW);
  digitalWrite(LED_DISCONNECTED_PIN, HIGH);
  delay(1000);
  Serial.println("=== BOOT: starting IMU ===");

  // Must be still for this -- begin() calibrates the gyro. The accelerometer
  // is never read by the odometry, so it needs no calibration.
  imu.begin();

  odom.begin();
  Serial.println("=== IMU + odometry ready ===");

#if ODOM_CALIBRATION_MODE
  driver.setBrake();
  // Green flashing = in calibration, not driving. The disconnected LED is
  // cleared first so the pair cannot be read as a link state, which is what
  // they mean in every other mode.
  digitalWrite(LED_DISCONNECTED_PIN, LOW);
  xTaskCreate(calibrationBlinkTask, "calibBlink", 2048, nullptr, 1, nullptr);

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
  serviceStatusLeds();

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
      applyDrive(0, 0, 0);
      return;
    }

    if (data.length() == 1 && data.equals("x")) {
      // Hard stop, deliberately NOT routed through applyDrive: a kill switch
      // must cut power immediately, not spin the motors backwards first.
      brakeUntil = 0;
      lastX = lastY = lastTurn = 0;
      releaseHeadingHold();   // no PID output should survive a kill
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

      // applyDrive owns both the motors and the odometry command: the filter's
      // motion model is built from what we send, so it has to see the brake
      // pulse too or the model silently goes stale.
      applyDrive(x, y, turn);
    }
  }

  serviceBrake();

  if (!clientConnected()) {
    // Safety stop if the client disconnects. Routed through applyDrive so the
    // robot brakes to a halt rather than coasting away on its last command.
    // Skipped while a pulse is in flight -- this branch runs every loop, and
    // re-issuing zero would cancel the brake it just started.
    if (brakeUntil == 0) applyDrive(0, 0, 0);
    setAuxMotor(false);     // a held bumper must not latch on when the link dies
  }

  // Runs every loop; the filter self-limits to its own update interval.
  odom.update();

  if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = millis();
    sendToClient("ODOM," + odom.telemetry());
  }
}
