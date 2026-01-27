#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Hardware.h"



// 馬達控制函式
void SetMotorSpeed(uint8_t port, int8_t speed);
void MotorStop();
void RobotIKControl(int8_t vx, int8_t vy, float omega);
void Vector_Motion(float Vx, float Vy);
void Degree_Motion(float moving_degree, int8_t speed);

#endif