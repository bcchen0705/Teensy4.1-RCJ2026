#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
void setup() {
    Robot_Init();
    pinMode(13,OUTPUT);
    digitalWrite(13,HIGH);
}
void loop(){
    if(Serial4.available() >=4 ){
        Serial.println(Serial4.read());
    }
}