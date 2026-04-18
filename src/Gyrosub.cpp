#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

float omega;
uint8_t goal_valid = 0x00;  // ✅ 拉到全域

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}

void readMainCore() {
  // 使用 static 確保函數結束後 index 不會歸零
  static uint8_t buffer[7];
  static int index = 0;

  while (Serial8.available() > 0) {
    uint8_t b = Serial8.read();

    // 狀態 0 & 1：找標頭 0xAA
    if (index == 0 || index == 1) {
      if (b == 0xAA) {
        buffer[index++] = b;
      } else {
        index = 0; // 不是 AA 就重來
      }
      continue;
    }

    // 狀態 2~5：填充後續數據
    buffer[index++] = b;

    // 收滿 6 byte 開始檢查
    if (index == 7) {
      index = 0; // 準備下一包

      // 檢查結尾 0xEE
      if (buffer[6] != 0xEE) continue;

      // 檢查校驗和 Checksum
      uint8_t sum = 0;
      for (int i = 0; i <= 4; i++) {
        sum += buffer[i];
      }
      
      if (sum == buffer[5]) {
        // 解析數據
        int16_t finalomega = (int16_t)((buffer[3] << 8) | buffer[2]);
        omega = finalomega;
        goal_valid = buffer[4];
        // Serial.println(Deg_offset); // Debug 用
      }
    }
  }
}
void loop(){
  readMainCore();
  readBNO085Yaw();

  float omg = -omega/1500;
  Serial.println(omg, 3);
  if(goal_valid == 0x00){
    Vector_Motion(0,0,0,1);
    Serial.println("reset");
  }
  else if(goal_valid == 0xFF){
    Vector_Motion(4df0,0,omg,0);
  }
}