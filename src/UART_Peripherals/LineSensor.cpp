#include "LineSensor.h"
#include <Motor.h>

float LineSensor::linesensorDegreelist[32] = {
  0.00,  11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75,
  90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75,
  180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75,
  270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
};

int8_t LineSensor::linesensor_ver_cor[32] = {
    1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1, 0,
    -1,-2,-3,-4,-5,-6,-7,-8,-7,-6,-5,-4,-3,-2,-1, 0
};

//對應 .h 的 extern
LineSensor lineSensor;

LineSensor::LineSensor(){
  lineData.state = 0;
  lineData.valid = false;
}

// 更新函式實作
void LineSensor::update(){
  uint8_t buffer[7];
  Serial5.write(0xdd);

  while(!Serial5.available());

  Serial5.readBytes(buffer,7);
  lineData.valid = false;

  if(buffer[0] != 0xAA) return;
  
  if(buffer[0] == 0xAA && buffer[6] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;

    if(checksum == buffer[5]){
      lineData.valid = true;
      lineData.state = 
      (uint32_t)buffer[1] | 
      ((uint32_t)buffer[2] << 8) | 
      ((uint32_t)buffer[3] << 16) | 
      ((uint32_t)buffer[4] << 24);     
      if(lineData.state != 0){
        Vector_Motion(0,0);  // Stop robot if line detected
      }
    }
  }
  else{
    lineData.valid = false;  // checksum error
  }
}