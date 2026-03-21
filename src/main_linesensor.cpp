#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

// 定義目前的顯示狀態
enum DisplayState { IDLE, SCANNING, SAVED };
DisplayState currentScreen = IDLE;

void drawMessage(const char* msg);
void print();

void setup(){
  Robot_Init();

  drawMessage("START");
  currentScreen = IDLE;
}

// 封裝一個刷螢幕的函式，確保不會亂閃
void drawMessage(const char* msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(msg);
  display.display(); // 只有呼叫這函式時才會真正動到螢幕
}


void loop(){
  // --- 1. 處理發送指令 (按鈕) ---
  if (digitalRead(BTN_ENTER) == LOW) {
    Serial8.print('C'); // 傳送校準指令
    delay(200); 
  }
  if (digitalRead(BTN_ESC) == LOW) {
    Serial8.print('E'); // 傳送結束指令
    delay(200);
  }


  // --- 2. 處理接收回報 (螢幕更新) ---
  if (Serial8.available() > 0) {
    char feedback = Serial8.read();
    
    // 只有當「狀態改變」時才刷螢幕
    if (feedback == 'B' && currentScreen != SCANNING) {
      drawMessage("SCANNING...");
      currentScreen = SCANNING;
    } 
    else if (feedback == 'D') {
      drawMessage("SAVED!");
      currentScreen = SAVED;
      delay(1000); // 讓 SAVED 停一下
      
      // 回到初始狀態
      drawMessage("READY");
      currentScreen = IDLE;
    }
  }
}


