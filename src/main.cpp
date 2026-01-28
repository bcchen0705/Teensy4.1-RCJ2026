#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>

int ballvx;
int ballvy;

void setup(){
  Hardware::initPins();
}
void loop(){
  ballData.readBallCam();
  gyroData.readBNO085Yaw();
  if(ballData.valid = true){    
    Serial.print("Angle: "); Serial.println(ballData.angle);

    float angle_rad = ballData.angle * DtoR_const;
    ballvx = (int)round(10.0f * cos(angle_rad));
    ballvy = (int)round(10.0f * sin(angle_rad));
    Serial.print("vx");Serial.println(ballvx);
    Serial.print("vy");Serial.println(ballvy);
    
    Vector_Motion(ballvx,ballvy);
  } 
  else {
    MotorStop();
  }
}