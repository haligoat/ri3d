#include <EchoLib.h>
#include <Bluepad32.h>

MotorControllers motors;

MecanumDrive driver(motors, 2, 3, 1, 4); //fl, fr, bl, br

ControllerPtr curController;

unsigned long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BOOT: setup() started ===");

  Serial.println("Calling BP32.setup()...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("BP32.setup() returned OK");

  driver.setBrake();
  Serial.println("=== BOOT COMPLETE: entering loop() ===");
}


void loop() {
  //BP32.update();


  int x = 0;
  int y = 0;
  int turn = 0;
  /*
  takeInput(&x, &y, &turn);
  driver.drive(x, y, turn);
  */

  if (Serial.available()>0) {
    char c = Serial.read();
    Serial.print("Received: '");
    Serial.print(c);
    Serial.println("'");

    if (c == 'w') {
      y = 50;
    }else if (c == 's') {
      y = -50;
    }

    if (c == 'a') {
      x = -12;
    }else if (c == 'd') {
      x = 50;
    }
  }
  driver.drive(x, y, turn);
  if (x!= 0 || y!= 0){
    delay(2000);
  }
  x = 0;
  y = 0;
  turn = 0;

}

void takeInput(int *oX, int *oY, int *oTurn) {
  if (curController && curController->isConnected()) {
    *oX = curController->axisX();
    *oY = -curController->axisY();
    *oTurn = curController->axisRX();
  }else {
    *oX = 0;
    *oY = 0;
    *oTurn = 0;
  }
  convertValuesForDriver(oX, oY, oTurn);
}

void convertValuesForDriver(int *ox, int *oy, int *oturn) {
  if (*ox<50 && *ox>-50){
    *ox = 0;
  }
  if (*oy<50 && *oy>-50){
    *oy = 0;
  }
  if (*oturn<50 && *oturn>-50){
    *oturn = 0;
  }
  scale(ox, oy, oturn);
}

void scale(int *oX, int *oY, int *oTurn) {
  *oX = *oX * 255/511;
  *oY = *oY * 255/511;
  *oTurn = *oTurn * 255/511;
}

void onConnectedController(ControllerPtr ctrlr){
  Serial.println("Controller Connected");
  curController = ctrlr;
}

void onDisconnectedController(ControllerPtr ctrlr){
  Serial.println("Controller Disconnected");
  curController = nullptr;
}
