#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <BallCam.h>
#include <Hardware.h>

uint8_t BallCam::buffer[6] = {0};
uint8_t BallCam::index = 0;

void readBallCam();

void readBallCam(){
    ballData.valid = false;

    while(Serial4.available()){
        uint8_t b = Serial4.read();
        if(index == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[index++] = b;

        if(index == 6){
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
               ballData.angle = buffer[1] | (buffer[2] << 8);
               ballData.dist = buffer[3] | (buffer[4] << 8);
               ballData.valid = true;
               if(ballData.angle == 65535 || ballData.dist == 65535){
                ballData.valid = false;
               }
            }
            index = 0;  // reset buffer
        }
    }
}