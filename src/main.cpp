#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define possession_Threshold 150

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
//uint32_t lastTargetTime = 0;
//static float rotate = 0;
//static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

void setup(){
  Robot_Init();
  //drawMessage("START");
}

void loop(){
  
  readussensor();
  readBallCam();
  static uint32_t lastDisplayTime = 0;

  Serial.println(usData.dist_l);
  if (digitalRead(BTN_ENTER) == LOW) {
    Serial8.write(0xCC); // 傳送校準指令
    drawMessage("SCANNING...");
    delay(200); 
  }
  if (digitalRead(BTN_ESC) == LOW) {
    Serial8.write(0xAD); // 傳送結束指令
    delay(200); 
  }
  if(Serial8.available()){
    
      uint8_t c = Serial8.read();
      if(c == 0xDD){
      drawMessage("SAVED!");
      delay(1000); // 讓 SAVED 停一下
        
        // 回到初始狀態
      drawMessage("READY");
      delay(200);
      display.clearDisplay();
      }
  
  }
  //static uint32_t lastDisplayTime = 0;
  /*
  static uint32_t lastDisplayTime = 0;
  if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // 顯示指南針 (Heading) 輔助確認感測器是否正常
    display.setCursor(0, 10);
    //display.printf("pitch: %.1f", gyroData.pitch);
    display.printf("f: %d\n", usData.dist_f);
    display.printf("l: %d\n",usData.dist_l);
    display.printf("b: %d\n", usData.dist_b);
    display.printf("r: %d\n",usData.dist_r );
    display.display();
    lastDisplayTime = millis();
  }*/
 /*
  if (ballData.valid) { 
    float ballDegree = ballDegreelist[ballData.angle];
    float offset = 0;
    float ballspeed = map(ballData.dist, 6, 12, 20, 50);
    float moving_Degree = ballDegree;
    static uint32_t lastDisplayTime = 0;



    if (ballDegree >= 0 && ballDegree <= 180) {
      double offsetRatio = exp(-0.8 * (ballData.dist - 6)); 
      offset = 50.0 * offsetRatio; 
    
      if (ballDegree > 90) moving_Degree = ballDegree + offset;
      else moving_Degree = ballDegree - offset;
    } 
    else {
      double offsetRatio = exp(-1.0 * (ballData.dist - 8)); 
      offset = 90.0 * offsetRatio; 
    
      if (ballDegree > 270) moving_Degree = ballDegree - offset; 
      else moving_Degree = ballDegree + offset; 
  }
  
    if (ballDegree == 87.5 || ballDegree == 92.5) {
      offset = 0;
      ballspeed = 50;
      moving_Degree = 90;
    }
    
    moving_Degree = fmod(moving_Degree + 360.0f, 360.0f) ;
    ballData.Vx = ballspeed * cos(moving_Degree * DtoR_const);
    ballData.Vy = ballspeed * sin(moving_Degree * DtoR_const);

    //if (ballData.possession < possession_Threshold) {
      //ballData.Vx = 0;
      //ballData.Vy = 30;
    //}
    //ballspeed = constrain(ballspeed, 20, 50);
    
    if(usData.dist_r <= 26 ){if(ballData.Vx > 0)ballData.Vx = 0;}
    else if(usData.dist_r <= 30){if(ballData.Vx > 0)ballData.Vx *= 0.5; }
    else if(usData.dist_r <= 33){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}

    if(usData.dist_l <= 18 ){if(ballData.Vx < 0)ballData.Vx = 0;}
    else if(usData.dist_l <= 22){
      if(ballData.Vx < 0){ballData.Vx *= 0.5;}}
    else if(usData.dist_l <= 25){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}
    if(usData.dist_l <= 35){
      if(usData.dist_b <= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
      else if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy = 0.6;}
      else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
    }
    
    if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      
      // 顯示指南針 (Heading) 輔助確認感測器是否正常
      display.setCursor(0, 10);
      //display.printf("pitch: %.1f", gyroData.pitch);
      display.printf("angle: %.1f\n", ballDegree);
      display.printf("dist: %d\n",ballData.dist);
      display.display();
      lastDisplayTime = millis();
    }

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
    //Serial.print("Angle: "); Serial.println(ballDegree);
    //Serial.print("Dist: "); Serial.println(ballData.dist);
    Serial.print("vx= ");Serial.println(ballData.Vx);
    //Serial.print("vy= ");Serial.println(ballData.Vy);
  } 
  else { 
    drawMessage("NO BALL");
    uint8_t packet[8] = {0xAA,0xAA,0,0,0,0,0,0xEE};
    Serial8.write(packet, 8);
    Serial.println("0 ");
  }*/
      
    // 無線 追球
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

  
    float ballspeed = constrain(map(ballData.dist, 25, 60, 20, 40),20, 40);
   
    
    if(ballData.dist >= 65){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else{
      float angleError = fabs(ballData.angle - 90);
      float angleFactor = 1.0 - constrain(abs(angleError) / 90.0, 0.0, 1.0);
      //float offsetFactor = constrain(angleError / 90.0, 0.0, 1.0); 
      ballspeed = ballspeed * (0.9 + 0.1 * angleFactor);
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
        offset = 30 + 70 * offsetRatio;
        //offset = 100 * offsetRatio * offsetFactor;
        moving_degree = ballData.angle + (offset * side);
      }

      else if(ballData.angle < 80 || ballData.angle >= 270){
        side = -1;
        float offsetRatio = exp(-0.8 * (ballData.dist - 50));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 30 + 70 * offsetRatio;
        //offset = 100 * offsetRatio * offsetFactor;
        moving_degree = ballData.angle + (offset * side);
      }
    
    }
    
    moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

    //計算vx vy
    ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
    ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));
    
    if(ballData.dist <= 41 && ballData.angle <= 100 && ballData.angle >= 80){
      ballData.Vx = 0;
      ballData.Vy = 40;
      moving_degree = 90;
      offset = 0;
    }
    //右邊線
    if(usData.dist_r <= 26 ){if(ballData.Vx > 0)ballData.Vx = 0;}
    else if(usData.dist_r <= 30){if(ballData.Vx > 0)ballData.Vx *= 0.5; }
    else if(usData.dist_r <= 33){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}
    //左邊線
    if(usData.dist_l <= 18 ){if(ballData.Vx < 0)ballData.Vx = 0;}
    else if(usData.dist_l <= 22){if(ballData.Vx < 0){ballData.Vx *= 0.5;}}
    else if(usData.dist_l <= 25){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}
    //-------------------------------------------------------------------
    
    //前角落
    if(usData.dist_l <= 35){
      if(usData.dist_f <= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
      else if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
      else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_r <= 35){
      if(usData.dist_f <= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
      else if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
      else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
    }
    //後角落
    if(usData.dist_l <= 35){
      if(usData.dist_b <= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
      else if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
      else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_r <= 35){
      if(usData.dist_b<= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
      else if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
      else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_b<= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
    if(usData.dist_f<= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
    //-------------------------------------------------------------------

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