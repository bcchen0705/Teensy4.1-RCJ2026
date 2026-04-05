#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}
void loop(){
  Vector_Motion(0, 20);
}