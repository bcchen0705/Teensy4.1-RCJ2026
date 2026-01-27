#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>

BallCam ballCam;

void setup(){
  Hardware::initPins();
}
void loop(){
  ballCam.update();
  readBNO085Yaw();
}