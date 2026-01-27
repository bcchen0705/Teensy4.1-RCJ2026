#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Motor.h>
#include <math.h>
#include <Gyro.h>

#ifndef MAX_V
#define MAX_V 40
#endif

#ifndef DtoR_const
#define DtoR_const 0.0174532925f  // PI / 180
#endif

void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * MAX_V, 1.5 * MAX_V);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 4:
      analogWrite(pwmPin1, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_1,HIGH);
        digitalWrite(DIRB_1,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,HIGH);
      } else{
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,LOW);
      }
      break;
    case 3:
      analogWrite(pwmPin2, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_2,HIGH);
        digitalWrite(DIRB_2,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,HIGH);
      } else{
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,LOW);
      }
      break;
    case 2:
      analogWrite(pwmPin3, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_3,HIGH);
        digitalWrite(DIRB_3,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,HIGH);
      } else{
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,LOW);
      }
      break;
    case 1:
      analogWrite(pwmPin4, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_4,HIGH);
        digitalWrite(DIRB_4,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,HIGH);
      } else{
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,LOW);
      }
      break;
  }
}

void MotorStop(){
  digitalWrite(DIRA_1,LOW);
  digitalWrite(DIRB_1,LOW);
  digitalWrite(DIRA_2,LOW);
  digitalWrite(DIRB_2,LOW);
  digitalWrite(DIRA_3,LOW);
  digitalWrite(DIRB_3,LOW);
  digitalWrite(DIRA_4,LOW);
  digitalWrite(DIRB_4,LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}

void RobotIKControl(int8_t vx, int8_t vy, float omega){
  // Note: Cast omega to int8_t for consistent data types in the IK control matrix
  int8_t p1 = -vx + vy + (int8_t)omega;
  int8_t p2 = -vx - vy + (int8_t)omega;
  int8_t p3 = vx - vy + (int8_t)omega;
  int8_t p4 = vx + vy + (int8_t)omega;
  SetMotorSpeed(1, p1);
  SetMotorSpeed(2, p2);
  SetMotorSpeed(3, p3);
  SetMotorSpeed(4, p4);
}

void Vector_Motion(float Vx, float Vy){  
  float omega = 0.0;
  float current_gyro_heading = gyroData.heading;
  float sensor_heading = 90.0 - current_gyro_heading;
  float e = control.robot_heading - sensor_heading;
  if(fabs(e) > control.heading_threshold){
      omega = e * control.P_factor;
  }
  RobotIKControl((int8_t)Vx, (int8_t)Vy, omega);
}

void Degree_Motion(float moving_degree, int8_t speed){
  if(moving_degree > 360.0 || moving_degree < 0.0){
      MotorStop();
  }
  float moving_degree_rad = moving_degree * DtoR_const;
  float Vx = cos(moving_degree_rad) * speed;
  float Vy = sin(moving_degree_rad) * speed;
  Vector_Motion(Vx, Vy);
}
