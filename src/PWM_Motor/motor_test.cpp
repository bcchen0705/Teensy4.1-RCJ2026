#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>

#define pwmPin1 10    // PWM 控制腳
#define DIRA_1 11   // 方向控制腳1
#define DIRB_1 12

// Motor 2 Pins
#define pwmPin2 2    // PWM 控制腳
#define DIRA_2 3   // 方向控制腳1
#define DIRB_2 4

// Motor 3 Pins
#define pwmPin3 23    // PWM 控制腳
#define DIRA_3 36   // 方向控制腳1
#define DIRB_3 37
// Motor 4 Pins
#define pwmPin4 5   // PWM 控制腳
#define DIRA_4 6    // 方向控制腳1
#define DIRB_4 9
void setup(){
    Robot_Init();
}
void loop(){
    Vector_Motion(0,0);
    readBNO085Yaw();
    //Serial.print("heading");Serial.println(gyroData.heading);
    //Serial.print("pitch");Serial.println(gyroData.pitch);    

    /*
    SetMotorSpeed(1,-10);
    SetMotorSpeed(2,10);
    SetMotorSpeed(3,-10);
    SetMotorSpeed(4,-10);
    */
}