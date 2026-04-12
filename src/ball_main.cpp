#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}

void loop(){
  readBNO085Yaw();
  readBallCam();
  static uint32_t lastDisplayTime = 0;
  if(ballData.valid){   //有球
    //Serial.print("Angle: "); Serial.println(ballData.angle);
    //Serial.print("Dist: "); Serial.println(ballData.dist);
    if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      
      // 顯示指南針 (Heading) 輔助確認感測器是否正常
      display.setCursor(0, 10);
      //display.printf("pitch: %.1f", gyroData.pitch);
      display.printf("angle: %.1f\n", ballData.angle);
      display.printf("dist: %d\n",ballData.dist);
      display.display();
      lastDisplayTime = millis();
    }
    //轉成弧度
    float moving_degree = ballData.angle;
    float offset = 0;

  
    float ballspeed = constrain(map(ballData.dist, 35, 100, 20, 40),20, 40);
   
    
    if(ballData.dist >= 65){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else{
      float angleError = ballData.angle - 90;
      float angleFactor = 1.0 - constrain(abs(angleError) / 90.0, 0.0, 1.0);
      float ballspeed = ballspeed * (0.8 + 0.2 * angleFactor);
      float side;
      if(ballData.angle >= 80 && ballData.angle <= 100){
        ballspeed = 40;
        side = 0;
        offset = 0;
        moving_degree = 90;
      }
      else if(ballData.angle > 100 && ballData.angle < 270){
        side = 1;
        float offsetRatio = exp(-1.2 * (ballData.dist - 50));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 70 + 30 * offsetRatio;
        if (ballData.dist <= 45){
          offset = 100;
        }
        moving_degree = ballData.angle + (offset * side);
      }

      else if(ballData.angle < 80 || ballData.angle >= 270){
        side = -1;
        float offsetRatio = exp(-0.8 * (ballData.dist - 50));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 70 + 30 * offsetRatio;
        if (ballData.dist <= 45){
          offset = 90;
        }
        moving_degree = ballData.angle + (offset * side);
      }
      
     /*
       float angleError = ballData.angle - 90;

      // 距離影響（遠才需要切入）
      float offsetRatio = constrain((ballData.dist - 55) / 80.0, 0.0, 1.0);

      // 角度影響（越歪才需要修）
      float angleFactor = pow(constrain(abs(angleError) / 90.0, 0.0, 1.0), 1.2);

      // 最終 offset
      offset = 90 * offsetRatio * angleFactor;

      float side = (angleError > 0) ? 1 : -1;

      moving_degree = ballData.angle + offset * side;

      // 正前方 → 直接衝
      if(abs(angleError) < 10){
        moving_degree = 90;
        offset = 0;
      }
        */
    }
    
    moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

    //計算vx vy
    ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
    ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));
    if(ballData.dist <= 38 && ballData.angle <= 95 && ballData.angle >= 85){
      ballData.Vx *= 0.1;
      ballData.Vy = 40;
      moving_degree = 90;
      offset = 0;
    }
    Serial.print("angle= ");Serial.println(ballData.angle);
    Serial.print("dist= ");Serial.println(ballData.dist);
    Serial.print("moving= ");Serial.println(moving_degree);
    Serial.print("vx= ");Serial.println(ballData.Vx);
    Serial.print("vy= ");Serial.println(ballData.Vy);
    
    uint8_t packet[8];
    int16_t vx_i = (int16_t)(ballData.Vx);
    int16_t vy_i = (int16_t)(ballData.Vy);
    // Header
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    // vx
    packet[2] = vx_i & 0xFF;
    packet[3] = (vx_i >> 8) & 0xFF;
    // vy
    packet[4] = vy_i & 0xFF;
    packet[5] = (vy_i >> 8) & 0xFF;
    // checksum
    uint8_t sum = 0;
    for(int i = 2; i <= 5; i++){
      sum += packet[i];
    }
    packet[6] = sum;
    // end
    packet[7] = 0xEE;

    Serial8.write(packet, 8);
    
    
  }
  else { //無球
    drawMessage("NO BALL");
    uint8_t packet[8] = {0xAA,0xAA,0,0,0,0,0,0xEE};
    Serial8.write(packet, 8);
    Serial.println("0 ");
  }
}