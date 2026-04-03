#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define TRIG1 32
#define TRIG2 31
#define TRIG3 30
#define TRIG4 29
#define TRIG5 28
#define TRIG6 27
#define TRIG7 26

bool left;
bool right;
bool middle;

int vx;
int vy;

void setup(){
  Robot_Init();

  pinMode(TRIG1, INPUT_PULLDOWN);
  pinMode(TRIG2, INPUT_PULLDOWN);
  pinMode(TRIG3, INPUT_PULLDOWN);
  pinMode(TRIG4, INPUT_PULLDOWN);
  pinMode(TRIG5, INPUT_PULLDOWN);
  pinMode(TRIG6, INPUT_PULLDOWN);
  pinMode(TRIG7, INPUT_PULLDOWN);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  digitalWrite(TRIG3, LOW);
  digitalWrite(TRIG4, LOW);
  digitalWrite(TRIG5, LOW);
  digitalWrite(TRIG6, LOW);
  digitalWrite(TRIG7, LOW);
  Serial2.begin(115200);
}

void loop(){
  readBNO085Yaw();

  if(digitalRead(TRIG1) == 1|| digitalRead(TRIG2) == 1 ){
    vx = -30;
    vy = 0;
    
  }
  else if(digitalRead(TRIG6) == 1 ){
    vx = 30 ;
    vy = 0;
    
  }
  else if(digitalRead(TRIG5) == 1|| digitalRead(TRIG4) == 1|| digitalRead(TRIG3) == 1){
    vx=0;vy=0;
  }
  else{
    vx =0;
    vy =0;
  }
 
  Serial.print("TRIG1 ");Serial.println(digitalRead(TRIG1));
  Serial.print("TRIG2 ");Serial.println(digitalRead(TRIG2));
  Serial.print("TRIG3 ");Serial.println(digitalRead(TRIG3));
  Serial.print("TRIG4 ");Serial.println(digitalRead(TRIG4));
  Serial.print("TRIG5 ");Serial.println(digitalRead(TRIG5));
  Serial.print("TRIG6 ");Serial.println(digitalRead(TRIG6));
  Vector_Motion(vx, vy);    
}