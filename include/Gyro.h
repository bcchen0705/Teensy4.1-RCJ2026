#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>

class GyroData{
    public:
        float heading = 0.0;
        float pitch = 0.0;
        bool valid = false;
        float omega;
        struct RobotControl {
            float robot_heading = 90.0;        // Target heading
            float P_factor = 0.7;              // Proportional gain
            float heading_threshold = 10.0;    // Deadband (degrees)
            int8_t vx = 0;
            int8_t vy = 0;
        }control;

        void readBNO085Yaw();
};
extern GyroData gyroData;
#endif