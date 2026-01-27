#include "Gyro.h"

// 全域變數
GyroData gyroData;
RobotControl control;

void readBNO085Yaw() {
    const int PACKET_SIZE = 19;
    uint8_t buffer[PACKET_SIZE];
    gyroData.valid = false; // Reset flag before read attempt

    while (Serial2.available() >= PACKET_SIZE) {
        buffer[0] = Serial2.read();
        if (buffer[0] != 0xAA) continue;
        buffer[1] = Serial2.read();
        if (buffer[1] != 0xAA) continue;

        // 讀剩下 17 bytes
        for (int i = 2; i < PACKET_SIZE; i++) {
            buffer[i] = Serial2.read();
        }

        // 校驗和
        uint8_t esti_checksum = 0;
        for (int i = 2; i <= 16; i++) {
            esti_checksum += buffer[i];
        }
        esti_checksum %= 256;

        if (esti_checksum != buffer[18]) {
            continue; // Checksum 錯誤跳過
        }

        // Little Endian 轉 yaw / pitch
        int16_t yaw_raw   = (int16_t)((buffer[4] << 8) | buffer[3]);
        int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);

        if (abs(yaw_raw) <= 18000) {
            gyroData.heading = yaw_raw * 0.01f;
            gyroData.valid = true;
        }
        if (abs(pitch_raw) <= 18000) {
            gyroData.pitch = pitch_raw * 0.01f;
        }

        break; // 每次只處理一個 packet
    }
}