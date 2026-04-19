#include <Arduino.h>
#include <Robot.h>

unsigned long startTime = 0;

void setup(){
    Robot_Init();
    Serial.begin(115200);
    startTime = millis(); // 紀錄開始時間
}

void loop(){
    unsigned long currentTime = millis() - startTime;

    if (currentTime < 5000) {
        // 在這 5 秒內，loop 會跑成千上萬次
        // 每次都會進來執行 RobotIKControl，速度就會 1, 2, 3... 慢慢加上去
        Vector_Motion(0, 80, 0, 1);
    } 
    else if (currentTime < 7000) {
        // 5 秒到 7 秒之間，執行停止動作，速度會慢慢減下來
        Vector_Motion(0, 0, 0, 1);
    } 
    else {
        // 重新計時，循環動作
        startTime = millis();
    }

    // 這裡可以加一個極小的延時（例如 10ms），控制加減速的節奏
    delay(10); 
}