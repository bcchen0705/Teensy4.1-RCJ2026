#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#define front A15
#define left A14
#define back A17
#define right A16



void setup(){
  Robot_Init();
  pinMode(front, INPUT);
  pinMode(left, INPUT);
  pinMode(back, INPUT);
  pinMode(right, INPUT);
}
void loop(){
  float dist_b_raw = analogRead(back) * 520.0f / 1024.0f;
  float dist_l_raw = analogRead(left) * 520.0f / 1024.0f;
  float dist_r_raw = analogRead(right) * 520.0f / 1024.0f;
  float dist_f_raw = analogRead(front) * 520.0f / 1024.0f;
  //Serial.print("front= ");Serial.println(dist_f_raw);
  //Serial.print("left= ");Serial.println(dist_l_raw);
  //Serial.print("back= ");Serial.println(dist_b_raw);
  //Serial.print("right= ");Serial.println(dist_r_raw);
  
  if(dist_r_raw <= 25){
    vx = 0;
  }
  else if(dist_r_raw <= 30){
    vx = 0.5 * 30;
  }
  else if(dist_r_raw <= 40){
    vx = 0.7 * 30;
  }
  else{
    vx = 30;
  }
  Serial.print("vx ");Serial.println(vx);
  

}