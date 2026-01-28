#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <BallCam.h>
#include <Hardware.h>

BallCam ballData;

void BallCam::readBallCam(){
    static uint8_t buffer[6] = {0};
    static uint8_t idx = 0;
    while(Serial4.available()){
        uint8_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
               ballData.angle = buffer[1] | (buffer[2] << 8);
               ballData.dist = buffer[3] | (buffer[4] << 8);
            
               if(ballData.angle != 65535 || ballData.dist != 65535)
                ballData.valid = true;
               else{
                ballData.valid = false;
               }
            }
            else{
                ballData.valid = false;
            }
            idx = 0;  // reset buffer
        }
    }
}