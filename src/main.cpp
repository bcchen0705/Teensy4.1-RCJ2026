#include <BallCam.h>
#include <Gyro.h>
#include <Hardware.h>
#include <Motor.h>
#include <LineSensor.h>

int ballvx;
int ballvy;

// --- [全域狀態變數] ---
float lineVx = 0, lineVy = 0;
bool overhalf = false;
bool first_detect = false;
float init_lineDegree = -1;
uint32_t speed_timer = 0;

void setup(){
  Hardware::initPins();
}
void loop(){
  //讀數據
  ballData.readBallCam();
  gyroData.readBNO085Yaw();
  lineSensor.update();

  // A : 向量合成
  float sumX = 0, sumY = 0;
  int count = 0;
  bool linedetected = false;

  for(int i = 0; i < 32; i++){
    if(bitRead(lineSensor.lineData.state, i)){
      float deg = LineSensor::linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count ++;
      linedetected = true;
    }
  }

  // B : 反彈
  if(linedetected || overhalf){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 

    if(!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      Serial.println("LINE DETECTED !!!");
    }

    float diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}

    float finalDegree;
    //反方向逃跑
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f,360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }

    lineVx = 50.0f * cos(finalDegree * DtoR_const);
    lineVy = 50.0f * sin(finalDegree * DtoR_const);

    Vector_Motion((int)lineVx, (int)lineVy);

    // 無線
    if (!linedetected && (millis() - speed_timer > 500)){
      overhalf = false;
      first_detect = false;
      Serial.println("back to field");
    }
    return;  //踩線不追球
  }
  if(!linedetected){
    first_detect = false;
    overhalf = false;
  }
  // 無線追球
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