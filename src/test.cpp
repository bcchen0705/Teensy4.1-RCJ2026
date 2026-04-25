#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define ORBIT_RADIUS  50.0f   // 目標距離 cm
#define ORBIT_SPEED   35.0f   // 切線速度
#define ORBIT_KR       0.8f   // 徑向增益
#define FACE_KP        0.15f  // 面球旋轉增益

bool running = false;

bool btnPressed(int pin){
    static unsigned long last[40] = {0};
    if(digitalRead(pin) == LOW && millis() - last[pin] > 200){
        last[pin] = millis();
        return true;
    }
    return false;
}

void setup(){
    Robot_Init();
    drawMessage("TEST READY");
}

void loop(){
    if(btnPressed(BTN_UP))   { running = true;  drawMessage("RUN"); }
    if(btnPressed(BTN_ESC))  { running = false; MotorStop(); drawMessage("STOP"); }
    if(!running) return;

    readBNO085Yaw();
    readBallCam();
    //readussensor();

    float Vx = 0, Vy = 0, omega = 0;

    if(ballData.valid){
        int ballAngle = ballData.angle;
        int ballDist  = ballData.dist;

        // 切線（順時針）
        float tang = fmod(ballAngle - 90.0f + 360.0f, 360.0f);
        Vx = ORBIT_SPEED * cos(tang * DtoR_const);
        Vy = ORBIT_SPEED * sin(tang * DtoR_const);

        // 徑向修正
        float radErr = constrain(ORBIT_KR * (ballDist - ORBIT_RADIUS), -30.0f, 30.0f);
        Vx += radErr * cos(ballAngle * DtoR_const);
        Vy += radErr * sin(ballAngle * DtoR_const);

        // 面球旋轉：讓 ballAngle → 90
        omega = (ballAngle - 90.0f) * FACE_KP;
        omega = constrain(omega, -15.0f, 15.0f);

      
    }
    Serial.print("Vx");Serial.println(Vx);
    Serial.print("Vy");Serial.println(Vy);
    Serial.print("omega");Serial.println(omega);
    // 打包送 sub
    int16_t vx_i = (int16_t)Vx;
    int16_t vy_i = (int16_t)Vy;
    int16_t om_i = (int16_t)(omega * 100.0f);

    uint8_t pkt[11];
    pkt[0] = 0xAA; pkt[1] = 0xAA;
    pkt[2] = vx_i & 0xFF;        pkt[3] = (vx_i >> 8) & 0xFF;
    pkt[4] = vy_i & 0xFF;        pkt[5] = (vy_i >> 8) & 0xFF;
    pkt[6] = om_i & 0xFF;        pkt[7] = (om_i >> 8) & 0xFF;
    pkt[8] = 0x00;
    uint8_t sum = 0;
    for(int i = 2; i <= 8; i++) sum += pkt[i];
    pkt[9] = sum; pkt[10] = 0xEE;
    Serial8.write(pkt, 11);
}