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
}

void loop(){
  if(Serial8.available()){
    String packet = Serial8.readStringUntil('\n');
    packet.trim();

    int firstComma = packet.indexOf(',');
    
    if(firstComma != -1){
      int secondComma = packet.indexOf(',',firstComma + 1);
      
      if(secondComma != -1){
        ballData.Vx = packet.substring(0, firstComma).toFloat();
        ballData.Vy = packet.substring(firstComma + 1, secondComma).toFloat();
        gyroData.heading = packet.substring(secondComma + 1).toFloat();
        
        vx = ballData.Vx;
        vy = ballData.Vy;
      }
    }
    else if(packet == "No Ball Detected"){
      vx = 0;
      vy = 0;
    }
  }
  
  move();
}