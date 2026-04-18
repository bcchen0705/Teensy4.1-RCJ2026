#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

uint8_t goal_valid = 0x00;
float last_error = 0;
unsigned long last_time = 0;
float final_omega = 0;

void setup(){
    Robot_Init();
}

void loop(){
    readBNO085Yaw();
    readBallCam();
    readussensor();
    readcamera();

    // ── 球門 omega 計算 ──────────────────────────────
    if(camData.goal_valid){
        goal_valid = 0xFF;
        int error = camData.goal_x - 160;

        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0f;
        float d_error = 0;
        if(dt > 0 && last_time != 0)
            d_error = (error - last_error) / dt;
        last_error = error;
        last_time = now;

        final_omega = 0.05f * error + 0.01f * d_error;
        final_omega = constrain(final_omega, -10.0f, 10.0f);
        if(abs(error) < 60) final_omega = 0;
    }
    else{
        goal_valid  = 0x00;
        final_omega = 0;
        last_error  = 0;
        last_time   = 0;
    }

    // ── 球 vx/vy 計算 ────────────────────────────────
    if(ballData.valid){
        float moving_degree = ballData.angle;
        float offset = 0;
        float ballspeed = constrain(map(ballData.dist, 25, 55, 40, 70), 40, 70);

        if(ballData.dist >= 65){
            moving_degree = ballData.angle;
            offset = 0;
        }
        else{
            float angleError  = fabs(ballData.angle - 90);
            float angleFactor = 1.0 - constrain(angleError / 90.0, 0.0, 1.0);
            ballspeed = ballspeed * (0.6 + 0.4 * angleFactor);

            if(ballData.angle >= 75 && ballData.angle <= 105){
                ballspeed = 50;
                offset = 0;
                moving_degree = ballData.angle;
            }
            else{
                float side = (ballData.angle > 105 && ballData.angle < 270) ? 1.0f : -1.0f;
                float offsetRatio = constrain(exp(-1.5 * (ballData.dist - 55)), 0.0, 1.0);
                offset = 90 * offsetRatio;
                moving_degree = ballData.angle + offset * side;
            }
        }

        moving_degree = fmod(moving_degree + 360.0f, 360.0f);
        ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
        ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

        if(ballData.dist <= 37 && ballData.angle >= 80 && ballData.angle <= 100){
            ballData.Vx = 0;
            ballData.Vy = 80;
        }

        // 壁面限制（省略不變，保留原本）
        if(usData.dist_r <= 22){if(ballData.Vx > 0)ballData.Vx = 0;}
        else if(usData.dist_r <= 25){if(ballData.Vx > 0)ballData.Vx *= 0.5;}
        else if(usData.dist_r <= 30){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
        if(usData.dist_l <= 22){if(ballData.Vx < 0)ballData.Vx = 0;}
        else if(usData.dist_l <= 25){if(ballData.Vx < 0)ballData.Vx *= 0.5;}
        else if(usData.dist_l <= 30){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
        if(usData.dist_b <= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
        if(usData.dist_f <= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
    }
    else{
        ballData.Vx = 0;
        ballData.Vy = 0;
    }

    // ── 合併封包送給 Motor sub ───────────────────────
    // AA AA vx_l vx_h vy_l vy_h omega_l omega_h goal_valid checksum EE → 11 bytes
    int16_t vx_i    = (int16_t)ballData.Vx;
    int16_t vy_i    = (int16_t)ballData.Vy;
    int16_t omega_i = (int16_t)final_omega;

    uint8_t packet[11];
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    packet[2] = vx_i    & 0xFF;
    packet[3] = (vx_i   >> 8) & 0xFF;
    packet[4] = vy_i    & 0xFF;
    packet[5] = (vy_i   >> 8) & 0xFF;
    packet[6] = omega_i & 0xFF;
    packet[7] = (omega_i >> 8) & 0xFF;
    packet[8] = goal_valid;

    uint8_t sum = 0;
    for(int i = 2; i <= 8; i++) sum += packet[i];
    packet[9]  = sum;
    packet[10] = 0xEE;

    Serial8.write(packet, 11);
}
