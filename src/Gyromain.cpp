#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>


void setup(){
  Robot_Init();

}
void loop(){
  ballsensor();
  uint8_t packet[6];
  int16_t Deg;
  if(ballData.valid){
    float ballDegree = ballDegreelist[ballData.angle];
    float ball_error = ballDegree - 90;

    if (ball_error > 180) ball_error -= 360;
    if (ball_error < -180) ball_error += 360;

    Deg = (int16_t)(ball_error);
    
     }
  else{
    Deg = 0;
  }
   // Header
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    // vx
    packet[2] = Deg & 0xFF;
    packet[3] = (Deg >> 8) & 0xFF;
    uint8_t sum = 0;
    for(int i = 0; i <= 3; i++){
      sum += packet[i];
    }
    packet[4] = sum;
    // end
    packet[5] = 0xEE;

    Serial8.write(packet, 6);


}