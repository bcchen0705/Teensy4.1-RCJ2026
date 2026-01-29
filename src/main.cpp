#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>

int ballvx;
int ballvy;
float omega;

void setup(){
  Hardware::initPins();
}
void loop(){
  ballData.readBallCam();
  gyroData.readBNO085Yaw();

  if(ballData.valid){    //有球
    Serial.print("Angle: "); Serial.println(ballData.angle);
    Serial.print("Dist: "); Serial.println(ballData.dist);

    //轉成弧度
    float angle_rad = ballData.angle * DtoR_const;

    //計算vx vy
    ballvx = (int)round(15.0f * cos(angle_rad));
    ballvy = (int)round(15.0f * sin(angle_rad));
    Serial.print("vx");Serial.println(ballvx);
    Serial.print("vy");Serial.println(ballvy);
    
    //誤差
    float error = ballData.angle - gyroData.heading;

    if(fabs(error) > 20.0f){
      gyroData.control.robot_heading = ballData.angle;
    }

    Vector_Motion(ballvx,ballvy);
  }
  else { //無球
    Serial.println("No Ball Detected");
    Vector_Motion(0,0);
  }

}