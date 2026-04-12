#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

float final_omega;
float out_omega;
void setup(){
  Robot_Init();

}
void loop(){
  readcamera();
  
  if(camData.goal_valid){
    Serial.print("X= ");Serial.println(camData.goal_x);
    //Serial.print("Y= ");Serial.println(camData.goal_y);
    //Serial.print("W= ");Serial.println(camData.goal_w);
    //Serial.print("H= ");Serial.println(camData.goal_h);
    Serial.println("------");
  }
  int final_omega = 0;

  // 2. 判斷邏輯（建議將左右眼邏輯整合，避免互相覆蓋）
  // 假設兩邊都看到，則需要一個綜合判斷
  int error = camData.goal_x - 125;

  // 比例控制（重點）
  final_omega = error * 0.1;

  // 限制最大輸出
  if (final_omega > 10) final_omega = 10;
  if (final_omega < -10) final_omega = -10;

  // 死區（避免抖）
  if (abs(error) < 70) {
    final_omega = 0;
  }
  Serial.println(final_omega);
  // 3. 封包準備
  uint8_t packet[6];
  int16_t out_omega = (int16_t)final_omega;

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  
  // 這裡修正了：把 omega 放入封包，並確保變數名稱正確
  packet[2] = out_omega & 0xFF;
  packet[3] = (out_omega >> 8) & 0xFF;

  // 4. Checksum 計算
  uint8_t sum = 0;
  for(int i = 0; i <= 3; i++){
    sum += packet[i];
  }
  packet[4] = sum;
  packet[5] = 0xEE;

  Serial8.write(packet, 6);
}