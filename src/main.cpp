#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define possession_Threshold 205

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
//uint32_t lastTargetTime = 0;
//static float rotate = 0;
//static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

void setup(){
  Robot_Init();
}
void loop(){
  ballsensor();

  if (ballData.valid) { // --- IF 開始 ---
    float ballDegree = ballDegreelist[ballData.angle];
    float offset = 0;
    float ballspeed = map(ballData.dist, 0, 12, 20, 40);
    float moving_Degree = ballDegree;

    if(ballData.dist > 7){
      moving_Degree = ballDegree;
    }
    
    else {
      double offsetRatio = exp(-0.55 * (ballData.dist - 7));
      offsetRatio = (offsetRatio > 1) ? 1 : offsetRatio;
      offset = 95 * offsetRatio;
      offset = (ballDegree > 90) ? offset : -offset;
      offset = (ballDegree < 270) ? offset : -offset;

      moving_Degree = ballDegree + offset;
    }
    if (ballDegree == 87.5 || ballDegree == 92.5) {
      offset = 0;
      ballspeed = 30;
      moving_Degree = 90;
    }
    
    moving_Degree = fmod(moving_Degree + 360.0f, 360.0f) ;
    ballData.Vx = ballspeed * cos(moving_Degree * DtoR_const);
    ballData.Vy = ballspeed * sin(moving_Degree * DtoR_const);

    //if (ballData.possession < possession_Threshold) {
      //ballData.Vx = 0;
      //ballData.Vy = 30;
    //}
    ballspeed = constrain(ballspeed, 20, 40);

    
    String packet = String(ballData.Vx) + "," + String(ballData.Vy); 
    Serial8.println(packet);
    //Serial.print("Angle: "); Serial.println(ballDegree);
    //Serial.print("Dist: "); Serial.println(ballData.dist);
    Serial.print("vx= ");Serial.println(ballData.Vx);
    Serial.print("vy= ");Serial.println(ballData.Vy);
  } 
  else { 
    Serial8.println("No Ball Detected");
    Serial.println("0 ");
  }
      
    // 無線 追球
  /*if(ballData.valid){   //有球
    //Serial.print("Angle: "); Serial.println(ballData.angle);
    //Serial.print("Dist: "); Serial.println(ballData.dist);

    //轉成弧度
    float moving_degree = ballData.angle;
    float ballspeed = constrain(map(ballData.dist, 40, 100, 20, 40),20, 40);
    float offset = 0;

    ballspeed = constrain(ballspeed, 20, 40);
    
    if(ballData.dist >= 50){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else{
      float side;

      if(ballData.angle > 95 && ballData.angle < 270){
        side = 1;
        float offsetRatio = exp(-0.05 * (ballData.dist - 40));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 45 + 45 * offsetRatio;
        if (ballData.dist < 40){
          offset = 90;
        }
        moving_degree = ballData.angle + (offset * side);
      }

      else if(ballData.angle < 72 || ballData.angle >= 270){
        side = -1;
        float offsetRatio = exp(-0.05 * (ballData.dist - 35));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 45 + 45 * offsetRatio;
        if (ballData.dist <= 35){
          offset = 90;
        }
        moving_degree = ballData.angle + (offset * side);
      }
      else{
        ballspeed = 30;
        side = 0;
        offset = 0;
        moving_degree = 90;
      }
    }
    if(ballData.angle <= 95 && ballData.angle >= 72){
      ballspeed = 30;
      moving_degree = 90;
      offset = 0;
    }
    moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

    //計算vx vy
    ballData.Vx = (int)round(20 * cos(moving_degree * DtoR_const));
    ballData.Vy = (int)round(20 * sin(moving_degree * DtoR_const));
    Serial.print("angle= ");Serial.println(ballData.angle);
    Serial.print("dist= ");Serial.println(ballData.dist);
    Serial.print("moving= ");Serial.println(moving_degree);
    //Serial.print("vx= ");Serial.println(ballData.Vx);
    //Serial.print("vy= ");Serial.println(ballData.Vy);
    
    String packet = String(ballData.Vx) + "," + String(ballData.Vy); + "," + String(gyroData.heading);
    Serial8.println(packet);
    
    
  }
  else { //無球
    Serial8.println("No Ball Detected");
    Serial.println("0 ");
  }
  */
}