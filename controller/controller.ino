#include <EchoLib.h>
#include <WiFi.h>

WiFiServerBridge server("Zippy", "robot2024", 8888);
MotorControllers motors;
MecanumDrive driver(motors, 1, 3, 4, 2); // fl, fr, bl, br

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BOOT: starting WiFi access point ===");

  server.enableDebug(true);
  server.setTimeoutConnectionDependent(true);
  server.begin();

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
    return;
  }

  String data = server.readData();
  if (data.length() == 0) return;
  if (data.length() == 1 && data.equals("x")) {
      raise(9);
  }

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
  }
}
