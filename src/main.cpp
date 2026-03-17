#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
uint32_t lastTargetTime = 0;
static float rotate = 0;
static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

void setup(){
  Robot_Init();
}
void loop(){
  readBNO085Yaw();
  readBallCam();
  readCameraData();

    // 無線 追球
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
    ballData.Vx = (int)round(MAX_V * cos(moving_degree * DtoR_const));
    ballData.Vy = (int)round(MAX_V * sin(moving_degree * DtoR_const));

    if(ballData.valid){
    String packet = String(ballData.Vx) + "," + String(ballData.Vy) + "," + String(gyroData.heading);
    Serial8.println(packet);
    }
  }
  else { //無球
    Serial8.println("No Ball Detected");
  }

}