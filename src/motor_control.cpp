#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

float vx;float vy;

void move();

void move(){
  RobotIKControl(vx, vy, 0);
}

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}

void loop(){
  readBNO085Yaw();

  if (Serial8.available()) {
    // 讀取整行資料
    String packet = Serial8.readStringUntil('\n');
    packet.trim();

    // Debug: 先看原始字串到底長怎樣
    // 如果這裡印出來全是 "1111"，代表 MainCore 發出的東西就錯了
    Serial.print("Received Packet: "); Serial.println(packet);

    if (packet == "No Ball Detected") {
      vx = 0; vy = 0;
    } 
    else {
      // 解析格式 "Vx,Vy,Heading"
      int firstComma = packet.indexOf(',');
      int secondComma = packet.indexOf(',', firstComma + 1);

      if (firstComma != -1) {
        vx = packet.substring(0, firstComma).toFloat();
        vy = packet.substring(firstComma + 1, secondComma).toFloat();
        // 如果需要 Gyro，也可以解析：float targetHeading = packet.substring(secondComma + 1).toFloat();
      }
    }
    Serial.print("vx= ");Serial.println(vx);
    Serial.print("vy= ");Serial.println(vy);
    Vector_Motion(vx,vy);
  }
  
}