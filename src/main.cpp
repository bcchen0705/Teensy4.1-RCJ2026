#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>
#include <LineSensor.h>

int ballvx;
int ballvy;

void setup(){
  Hardware::initPins();
}
void loop(){
  ballData.readBallCam();
  gyroData.readBNO085Yaw();
  lineSensor.update();

  if(ballData.valid){    //有球
    Serial.print("Angle: "); Serial.println(ballData.angle);
    Serial.print("Dist: "); Serial.println(ballData.dist);

    //轉成弧度
    float moving_degree = ballData.angle;
    float ballspeed = constrain(map(ballData.dist, 40, 100, 20, 50),20, 50);
    float offset = 0;

    ballspeed = constrain(ballspeed, 20, 50);
    
    if(ballData.dist >= 70){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else if(ballData.angle <= 105 && ballData.angle >= 75){
      ballspeed = 30;
      moving_degree = 90;
      offset = 0;
    }
    else{
      if (ballData.dist < 42){
        offset = 90;
      }
      float offsetRatio = exp(-0.05 * (ballData.dist - 42));
      offsetRatio = constrain(offsetRatio, 0.0, 1.0);
      offset = 45 + 45 * offsetRatio;
      
      float side;
      if(ballData.angle > 90 && ballData.angle < 270){
        side = 1;
      }
      else{side = -1;}

      moving_degree = ballData.angle + (offset * side);
      moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

    }
      
    //計算vx vy
    ballvx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
    ballvy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

    Serial.printf("moving%f",moving_degree);Serial.print("");
    Serial.print("vx");Serial.println(ballvx);
    Serial.print("vy");Serial.println(ballvy);
    
    //誤差
    /*float error = ballData.angle - gyroData.heading;

    if(fabs(error) > 20.0f){
      gyroData.control.robot_heading = ballData.angle;
    }
    */
    Vector_Motion(ballvx,ballvy);
    //delay(300);
  }
  else { //無球
    Serial.println("No Ball Detected");
    Vector_Motion(0,0);
  }

}