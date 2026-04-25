#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

bool btnPressed(int pin) {
    static unsigned long last[40] = {0};
    if (digitalRead(pin) == LOW && millis() - last[pin] > 200) {
        last[pin] = millis();
        return true;
    }
    return false;
}

void sendPacket() {
    int16_t ball_angle = (int16_t)ballData.angle;
    int16_t ball_dist  = (int16_t)ballData.dist;
    uint8_t ball_valid = ballData.valid ? 0xFF : 0x00;
    int16_t goal_x     = (int16_t)camData.goal_x;
    uint8_t goal_valid = camData.goal_valid ? 0xFF : 0x00;
    int16_t us_f       = (int16_t)usData.dist_f;
    int16_t us_b       = (int16_t)usData.dist_b;
    int16_t us_l       = (int16_t)usData.dist_l;
    int16_t us_r       = (int16_t)usData.dist_r;

    uint8_t packet[21];
    packet[0]  = 0xAA;
    packet[1]  = 0xAA;
    packet[2]  = ball_angle & 0xFF;
    packet[3]  = (ball_angle >> 8) & 0xFF;
    packet[4]  = ball_dist  & 0xFF;
    packet[5]  = (ball_dist  >> 8) & 0xFF;
    packet[6]  = ball_valid;
    packet[7]  = goal_x & 0xFF;
    packet[8]  = (goal_x >> 8) & 0xFF;
    packet[9]  = goal_valid;
    packet[10] = us_f & 0xFF;
    packet[11] = (us_f >> 8) & 0xFF;
    packet[12] = us_b & 0xFF;
    packet[13] = (us_b >> 8) & 0xFF;
    packet[14] = us_l & 0xFF;
    packet[15] = (us_l >> 8) & 0xFF;
    packet[16] = us_r & 0xFF;
    packet[17] = (us_r >> 8) & 0xFF;

    uint8_t sum = 0;
    for (int i = 2; i <= 17; i++) sum += packet[i];
    packet[18] = sum;
    packet[19] = 0xEE;

    Serial8.write(packet, 20);
}

void setup() {
    Robot_Init();
    Serial.println("testmain ready");
}

void loop() {
    // 按鈕通知 sub
    if (btnPressed(BTN_UP)) {
        Serial8.write(0xBB);  // BTN_UP 訊號
        Serial.println("BTN_UP");
    }

    // 讀所有感測器
    readBNO085Yaw();
    readussensor();
    readBallCam();
    readcamera();

    // 送封包
    sendPacket();
}