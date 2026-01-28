#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>


void setup(){
  Hardware::initPins();
}
void loop(){
  readBallCam();
  readBNO085Yaw();
}