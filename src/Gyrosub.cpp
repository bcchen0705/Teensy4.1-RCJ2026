#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

 float Deg_offset;

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}

void readMainCore() {
while (Serial8.available()) {



    // ===== DATA FRAME =====
    if (Serial8.available() < 6) return;

    if (Serial8.read() != 0xAA) continue;
    if (Serial8.read() != 0xAA) continue;

    uint8_t buffer[6];
    buffer[0] = 0xAA;
    buffer[1] = 0xAA;

    for (int i = 2; i < 6; i++) {
      buffer[i] = Serial8.read();
    }

    // check end
    if (buffer[5] != 0xEE) continue;

    // checksum
    uint8_t sum = 0;
    for (int i = 0; i <= 3; i++) {
      sum += buffer[i];
    }
    if (sum != buffer[4]) continue;

    // decode
    int16_t Deg = (buffer[3] << 8) | buffer[2];

    Deg_offset = Deg;

    return; // 一次只處理一包
  }
}

void loop(){
  readMainCore();
  readBNO085Yaw();
  Vector_Motion(0,0,0);

}