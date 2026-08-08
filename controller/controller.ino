#include <EchoLib.h>
#include <Bluepad32.h>

MotorControllers motors;

MecanumDrive driver(motors, 1, 2, 3, 4); //fl, fr, bl, br

ControllerPtr curController;
void setup() {
  Serial.begin(115200);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  driver.setBrake();
}


void loop() {
  BP32.update();

  int x;
  int y;
  int turn;

  takeInput(&x, &y, &turn);
  driver.drive(x, y, turn);
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

