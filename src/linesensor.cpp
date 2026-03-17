#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#include <EEPROM.h>

#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define LS_count 32

struct LineData{uint32_t state = 0xFFFFFFFF;} lineData;

int readMux(int ch, int sigPin);
void line_calibrate();
void linesensor_update();
void moveBackInBounds();

const uint8_t mapTable[32] = {
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31
};

uint16_t max_ls[LS_count];
uint16_t avg_ls[LS_count];
uint16_t min_ls[LS_count];

//SPEED
float lineVx = 0;
float lineVy = 0;

float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;
uint32_t speed_timer = 0;


int readMux(int ch, int sigPin) {

  digitalWrite(s0, (ch >> 0) & 1);
  digitalWrite(s1, (ch >> 1) & 1);
  digitalWrite(s2, (ch >> 2) & 1);
  digitalWrite(s3, (ch >> 3) & 1);
  //delay(1);
  delayMicroseconds(50);
  uint16_t temp = analogRead(sigPin);
  return temp;
}

void line_calibrate(){
  for(int i=0; i<LS_count; i++){
    max_ls[i] = 0;
    min_ls[i] = 4095;
  }
  while(digitalRead(BTN_ESC)){
    for(uint8_t i = 0; i < LS_count; i++){
    uint16_t reading = readMux(i, (i < 16) ? M1 : M2);

    if(reading > max_ls[i]) max_ls[i] = reading;
    if(reading < min_ls[i]) min_ls[i] = reading;
    } 
  }

  for(uint8_t i = 0; i < LS_count; i++){
      avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
  }
  EEPROM.put(0, avg_ls);

  /*for(uint8_t i = 0; i < LS_count; i++){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(avg_ls[i]);
    delay(5);
  }*/
}

void linesensor_update(){
  lineData.state = 0xFFFFFFFF;
  for (uint8_t i = 0; i < LS_count; i++) {
    uint16_t reading = readMux(i, (i < 16) ? M1 : M2);;
    
    if (reading < avg_ls[i]) {
      lineData.state &= ~(1UL << i); 
    }
  }
}

void moveBackInBounds(){
//-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  for(int i = 0; i < LS_count; i++){
    if(bitRead(lineData.state, i) == 0){
      //if(i==0){continue;}
      
      //Serial.printf("read%d", i);
      
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count > 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      
      //Serial.println("LINE DETECTED !!!");
      //Serial.print("initlineDegree =");Serial.println(init_lineDegree);
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}
    
    //Serial.print("diff =");Serial.println(diff);


    //-----BACK TO FIELD-----
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f, 360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    //Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 50.0f *cos(finalDegree * DtoR_const);
    lineVy = 50.0f *sin(finalDegree * DtoR_const);

    Vector_Motion(lineVx, lineVy);
    
    //Serial.print("lineVx =");Serial.println(lineVx);
    //Serial.print("lineVy =");Serial.println(lineVy);
   
  }
  else{
    first_detect = false;
    Vector_Motion(0, 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(M1, INPUT);
  pinMode(M2, INPUT);
  
  EEPROM.begin();
  EEPROM.get(0, avg_ls);
  // 初始化 max / min
  
}

void loop(){
  while(!digitalRead(BTN_ENTER)){
    line_calibrate();
    EEPROM.get(0, avg_ls);
  }

  linesensor_update();
  moveBackInBounds();
}
  
