#include "LineSensor.h"
#include <Motor.h>

float LineSensor::linesensorDegreelist[32] = {
  90.00,  78.75,  67.50,  56.25,  45.00,  33.75,  22.50,  11.25,   // 前 -> 右
  0.00, 348.75, 337.50, 326.25, 315.00, 303.75, 292.50, 281.25,   // 右 -> 後
 270.00, 258.75, 247.50, 236.25, 225.00, 213.75, 202.50, 191.25,   // 後 -> 左
 180.00, 168.75, 157.50, 146.25, 135.00, 123.75, 112.50, 101.25
      
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
  Serial7.write(0xdd);
  while(!Serial7.available());
  Serial7.readBytes(buffer,7);
  lineData.valid = false;

  if(buffer[0] != 0xAA) return;
  
  if(buffer[0] == 0xAA && buffer[6] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;

    if(checksum == buffer[5]){
      lineData.valid = true;
      uint32_t current raw = 
      (uint32_t)buffer[1] | 
      ((uint32_t)buffer[2] << 8) | 
      ((uint32_t)buffer[3] << 16) | 
      ((uint32_t)buffer[4] << 24);
      
      //過濾邏輯
      static uint32_t last_raw = 0;
      
      lineData.state = current_raw & last_raw;
      
      last_raw = current_raw;
      // -----------------------


      if(lineData.state != 0){
        Vector_Motion(0,0);  // Stop robot if line detected
      }
    }
  }
  else{
    lineData.valid = false;  // checksum error
  }
}