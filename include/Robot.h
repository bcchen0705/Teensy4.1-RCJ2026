#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity

//Line Sensor
#define EMERGENCY_THRESHOLD 90

//BALL SEARCHING THRESHOLD
#define BALL_Threshold 5
#define TOTAL_BALL_SENSORS 10

//ROBOT MAX SPEED
#define MAX_V 50

//ROBOT DEFENSE PARAMETERS
#define MAX_VX 60
#define MAX_VY 60
#define Def_offset 2.5
#define Back_safe 35//cm
#define Side_safe 45//cm
#define Back_limit 15
#define Side_limit 45

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f

//按鈕
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26
int _page = 0;      // 0: 主選單, 1: 掃描頁面
int _cursor = 0;    // 選單游標位置
unsigned long _lastPress = 0; 
unsigned long _lastUpdate = 0;
// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Motor 4 Pins
#define pwmPin1 2    // PWM 控制腳
#define DIRA_1 3   // 方向控制腳1
#define DIRB_1 4

// Motor 3 Pins
#define pwmPin2 10    // PWM 控制腳
#define DIRA_2 11   // 方向控制腳1
#define DIRB_2 12

// Motor 2 Pins
#define pwmPin3 5    // PWM 控制腳
#define DIRA_3 6   // 方向控制腳1
#define DIRB_3 9

// Motor 1 Pins
#define pwmPin4 23  // PWM 控制腳
#define DIRA_4 36    // 方向控制腳1
#define DIRB_4 37 

//US Sensor
#define front_us A13
#define back_us A8
#define left_us A12
#define right_us A14
#define alpha 0.75
float pos_x_f = 0.0;
float pos_y_f = 0.0;

//Outside Line Sensor
#define back_ls 41     
#define left_ls 40    
#define right_ls 39 

//Kicker
#define Charge_Pin 33 //FET1
#define Kicker_Pin 32 //FET2


//Interrupt
volatile bool backtouch = false;
volatile bool lefttouch = false;
volatile bool righttouch = false;

// --- GLOBAL OBJECTS & STRUCTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct GyroData{float heading = 0.0; float pitch = 0.0; bool valid = false;} gyroData;
//struct LineData{uint32_t state = 0x3FFFF; bool valid = false;} lineData;
struct BallData{uint16_t dist = 255; uint16_t angle = 255; uint16_t possession = 255; bool valid = false; float Vx; float Vy;} ballData;
struct USSensor{uint16_t dist_b = 0; uint16_t dist_l = 0; uint16_t dist_r = 0;uint16_t dist_f = 0; } usData;
struct CamData{uint16_t x = 65535;uint16_t y = 65535;uint16_t w = 65535;uint16_t h = 65535; bool valid = false;} targetData;
struct LeftEye{uint16_t x = 65535;uint16_t y = 65535;uint16_t w = 65535;uint16_t h = 65535; bool valid = false;} leftData;
struct RightEye{uint16_t x = 65535;uint16_t y = 65535;uint16_t w = 65535;uint16_t h = 65535; bool valid = false;} rightData;

float ballDegreelist[16]={22.5,45,67.5,87.5,92.5,112.5,135,157.5,202.5,225,247.5,265,275,292.5,315,337.5};
float linesensorDegreelist[32] = {
    0.00, 11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75, 
    90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75, 
    180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75, 
    270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
};
//int8_t linesensor_ver_cor[18]={1,2,3,4,5,4,3,2,1,-1,-2,-3,-4,-5,-4,-3,-2,-1};

// --- ROBOT CONTROL STRUCT (New: For P-control state) ---
struct RobotControl{
    float robot_heading = 90.0;        // Target heading
    float P_factor = 0.5;             // Proportional gain
    float heading_threshold = 10.0;     // Deadband (degrees)
    int8_t vx = 0;
    int8_t vy = 0;
    bool picked_up = false;
} control;


// --- FUNCTION PROTOTYPES ---
// Including prototypes for the new functions and existing ones
void Robot_Init();
void readBNO085Yaw();
void LeftEye();
void RightEye();
void ballsensor();
void linesensor();
void positionEst();
void showStart();
void showLine();
void showRunScreen();
void showMessage(const char* message, int textSize = 2, int x = -1, int y = -1);
void showSensors(float gyro, int ballAngle);
void SetMotorSpeed(uint8_t port, int8_t speed);
void MotorStop();
void RobotIKControl(int8_t vx, int8_t vy, float omega);
void Vector_Motion(float Vx, float Vy);
void FC_Vector_Motion(int WVx, int WVy, float target_heading);
void Degree_Motion(float moving_degree, int8_t speed);
void kicker_control(bool);
bool menuUpdate() ;
bool white_line_processing();
void backlstouch();
void leftlstouch();
void rightlstouch();
void readBallCam();
// ******************************************************
// --- FUNCTION IMPLEMENTATIONS (Existing & New) ---
// ******************************************************

void Robot_Init(){
  Serial.begin(115200);
  
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  Serial8.begin(115200);
  
  pinMode(pwmPin1,OUTPUT);
  pinMode(DIRA_1,OUTPUT);
  pinMode(DIRB_1,OUTPUT);

  pinMode(pwmPin2,OUTPUT);
  pinMode(DIRA_2,OUTPUT);
  pinMode(DIRB_2,OUTPUT);

  pinMode(pwmPin3,OUTPUT);
  pinMode(DIRA_3,OUTPUT);
  pinMode(DIRB_3,OUTPUT);

  pinMode(pwmPin4,OUTPUT);
  pinMode(DIRA_4,OUTPUT);
  pinMode(DIRB_4,OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_ESC, INPUT_PULLUP);

  pinMode(front_us, INPUT);
  pinMode(back_us, INPUT);
  pinMode(left_us, INPUT);
  pinMode(right_us, INPUT);
  
  pinMode(Kicker_Pin, OUTPUT);
  pinMode(Charge_Pin, OUTPUT);

  digitalWrite(Kicker_Pin, LOW);
  digitalWrite(Charge_Pin, LOW);

  pinMode(back_ls, INPUT_PULLUP);
  pinMode(left_ls, INPUT_PULLUP);
  pinMode(right_ls, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(back_ls), backlstouch, RISING);
  attachInterrupt(digitalPinToInterrupt(left_ls), leftlstouch, RISING);
  attachInterrupt(digitalPinToInterrupt(right_ls), rightlstouch, RISING);

  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  kicker_control(0);

}

void readBNO085Yaw(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.valid = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if(esti_checksum != buffer[18]){
      //Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);

    //Serial.print("yaw_raw: ");
    //Serial.println(yaw_raw);

    // Convert to degrees if within range
    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.valid = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

void LeftEye(){
  static uint8_t buffer[10];
  uint8_t index = 0;
  leftData.valid = false;
  while (Serial5.available()){
    uint8_t b = Serial5.read();
    if(index == 0 && b != 0xCC){
      continue;  // 等待開頭 0xCC
    }
    buffer[index++] = b;
    if(index == 10){  // 收滿 10 bytes
      if(buffer[0] == 0xCC && buffer[9] == 0xEE){
        leftData.x = buffer[1] | (buffer[2] << 8);
        leftData.y = buffer[3] | (buffer[4] << 8);
        leftData.w = buffer[5] | (buffer[6] << 8);
        leftData.h = buffer[7] | (buffer[8] << 8);
        leftData.valid = true;  
        if(leftData.x == 65535 || leftData.y == 65535 || leftData.w == 65535 || leftData.h == 65535){
          leftData.valid = false;  
        }            
      }
      index = 0;  // reset buffer
    }
  }
}
void RightEye(){
  static uint8_t buffer[10];
  uint8_t index = 0;
  rightData.valid = false;
  while (Serial3.available()){
    uint8_t b = Serial3.read();
    if(index == 0 && b != 0xCC){
      continue;  // 等待開頭 0xCC
    }
    buffer[index++] = b;
    if(index == 10){  // 收滿 10 bytes
      if(buffer[0] == 0xCC && buffer[9] == 0xEE){
        rightData.x = buffer[1] | (buffer[2] << 8);
        rightData.y = buffer[3] | (buffer[4] << 8);
        rightData.w = buffer[5] | (buffer[6] << 8);
        rightData.h = buffer[7] | (buffer[8] << 8);
        rightData.valid = true;  
        if(rightData.x == 65535 || rightData.y == 65535 || rightData.w == 65535 || rightData.h == 65535){
          rightData.valid = false;  
        }            
      }
      index = 0;  // reset buffer
    }
  }
}
void ballsensor(){
  // 發送請求封包，通知感測器回傳資料
  uint8_t b[4];
  ballData.valid = false;

  Serial6.write(0xBB);
  while(!Serial6.available());
  Serial6.readBytes(b,4);
  if(b[1]==0xFF){
      ballData.valid = false;
      ballData.angle = 255;
      ballData.dist = 255;
  }
  else if(b[0]==0xAA){
    uint8_t temp =b[1];
    ballData.valid = true;
    ballData.angle = (temp & 0x0F);
    ballData.dist = (temp & 0xF0)>>4;
    ballData.possession = (uint8_t)((1-alpha) * b[2] + ballData.possession * alpha);
  }
  else{
    ballData.valid = false;
  }
}

/*void readBallCam(){
    
    static uint16_t buffer[6] = {0};
    static uint16_t idx = 0;
    while(Serial4.available()){
        uint16_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){ //裝包 共6組
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
              ballData.angle = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
              ballData.dist  = (uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8);
            
               if(ballData.angle != 65535 && ballData.dist != 65535)
                ballData.valid = true;
               else{
                ballData.valid = false;
               }  //無球
            }
            else{
                ballData.valid = false;
            }  //無數據
            idx = 0;  // reset buffer
        }  
    }
}*/
/*
void linesensor(){
  uint8_t buffer[7];
  Serial7.write(0xdd);
  while(!Serial7.available());
  Serial7.readBytes(buffer,7);
  lineData.valid = false;
  if(buffer[0] != 0xaa) return;
  if(buffer[0] == 0xAA && buffer[6] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;
    if(checksum == buffer[5]){
      lineData.valid = true;
      lineData.state = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);     
      if(lineData.state != 0b111111111111111111){
        Vector_Motion(0,0);  // Stop robot if line detected
      }
    }
  }
  else{
    lineData.valid = false;  // checksum error
  }
}*/
/*
void readussensor(){
  // static variables remember their values between calls
  static float dist_b_f = 0.0f;
  static float dist_l_f = 0.0f;
  static float dist_r_f = 0.0f;
  static float dist_f_f = 0.0f;

  // read raw ADC and convert to cm (or mm depending on your scaling)
  float dist_b_raw = analogRead(back_us) * 520.0f / 1024.0f;
  float dist_l_raw = analogRead(left_us) * 520.0f / 1024.0f;
  float dist_r_raw = analogRead(right_us) * 520.0f / 1024.0f;
  float dist_f_raw = analogRead(front_us) * 520.0f / 1024.0f;
  // complementary (low-pass) filtering
  dist_b_f = alpha * dist_b_f + (1.0f - alpha) * dist_b_raw;
  dist_l_f = alpha * dist_l_f + (1.0f - alpha) * dist_l_raw;
  dist_r_f = alpha * dist_r_f + (1.0f - alpha) * dist_r_raw;
  dist_f_f = alpha * dist_f_f + (1.0f - alpha) * dist_f_raw;
  // assign filtered values to struct
  usData.dist_b = dist_b_f;
  usData.dist_l = dist_l_f;
  usData.dist_r = dist_r_f;
  usData.dist_f = dist_f_f;
}

*/
/*void showUS(float dist1, float dist2, float dist3) {
  display.setTextSize(1);
  display.setCursor(60, 0);  display.print("d_l ");  display.println(dist1);
  display.setCursor(60, 15); display.print("d_r");  display.println(dist2);
  display.setCursor(60, 30); display.print("d_b "); display.println(dist3);
  display.display();
}*/

/*Actuators Part*/
void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * MAX_V, 1.5 * MAX_V);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 4:
      analogWrite(pwmPin1, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_1,HIGH);
        digitalWrite(DIRB_1,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,HIGH);
      } else{
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,LOW);
      }
      break;
    case 3:
      analogWrite(pwmPin2, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_2,HIGH);
        digitalWrite(DIRB_2,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,HIGH);
      } else{
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,LOW);
      }
      break;
    case 2:
      analogWrite(pwmPin3, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_3,HIGH);
        digitalWrite(DIRB_3,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,HIGH);
      } else{
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,LOW);
      }
      break;
    case 1:
      analogWrite(pwmPin4, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_4,HIGH);
        digitalWrite(DIRB_4,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,HIGH);
      } else{
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,LOW);
      }
      break;
  }
}

void MotorStop(){
  digitalWrite(DIRA_1,LOW);
  digitalWrite(DIRB_1,LOW);
  digitalWrite(DIRA_2,LOW);
  digitalWrite(DIRB_2,LOW);
  digitalWrite(DIRA_3,LOW);
  digitalWrite(DIRB_3,LOW);
  digitalWrite(DIRA_4,LOW);
  digitalWrite(DIRB_4,LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}

void RobotIKControl(int8_t vx, int8_t vy, float omega){
  // Note: Cast omega to int8_t for consistent data types in the IK control matrix
  int8_t p1 = -vx + vy + (int8_t)omega;
  int8_t p2 = -vx - vy + (int8_t)omega;
  int8_t p3 = vx - vy + (int8_t)omega;
  int8_t p4 = vx + vy + (int8_t)omega;
  SetMotorSpeed(1, p1);
  SetMotorSpeed(2, p2);
  SetMotorSpeed(3, p3);
  SetMotorSpeed(4, p4);
}

void Vector_Motion(float Vx, float Vy){  
  float omega = 0.0;
  float current_gyro_heading = gyroData.heading;
  float sensor_heading = 90.0 - current_gyro_heading;
  float e = control.robot_heading - sensor_heading;
  if(fabs(e) > control.heading_threshold){
      omega = e * control.P_factor;
  }
  RobotIKControl((int8_t)Vx, (int8_t)Vy, omega);
}

void FC_Vector_Motion(int WVx, int WVy, float target_heading) {
    // 1. Convert gyro to Radians (math functions use radians)
    float rad = (target_heading-90)* (M_PI / 180.0);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    // 2. Rotate World Vectors to Robot Frame
    int8_t robot_vx = (int8_t)(WVx * cos_h + WVy * sin_h);
    int8_t robot_vy = (int8_t)(-WVx * sin_h + WVy * cos_h);
    //Serial.printf("robot %d, %d\n", robot_vx, robot_vy);
    // 3. Calculate Heading Correction (Omega)
    float omega = 0;
    float current_gyro_heading = 90 - gyroData.heading;
    // Normalize error to find the shortest path to target_heading
    float e = target_heading - current_gyro_heading;
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    if (fabs(e) > control.heading_threshold) {
        omega = e * control.P_factor ;
    }
    //Serial.printf("omege%d\n", omega);
    // 4. Send to IK Control
    RobotIKControl(robot_vx, robot_vy, (int8_t)omega);
}

void Degree_Motion(float moving_degree, int8_t speed){
  if(moving_degree > 360.0 || moving_degree < 0.0){
      MotorStop();
  }
  float moving_degree_rad = moving_degree * DtoR_const;
  float Vx = cos(moving_degree_rad) * speed;
  float Vy = sin(moving_degree_rad) * speed;
  Vector_Motion(Vx, Vy);
}


void kicker_control(bool kick = false){
  static uint64_t charge_start = 0;
  static uint64_t last_charge_done = 0;
  static bool charging_state = false;

  const uint32_t CHARGE_DURATION = 5000;   // ms needed to charge
  const uint32_t CHARGE_TIMEOUT  = 8000;  // ms before recharging automatically

  uint64_t now = millis();

  // Auto-recharge if too long since last charge
  if(charging_state && (now - last_charge_done > CHARGE_TIMEOUT)){
    charging_state = false;
  }

  // Start charging if not charged and not already charging
  if(!charging_state && charge_start == 0){
    charge_start = now;
    //Serial.println("Charge");
    digitalWrite(Charge_Pin, HIGH);
    digitalWrite(Kicker_Pin, LOW);
  }

  // Stop charging when duration is met
  if(charge_start != 0 && (now - charge_start >= CHARGE_DURATION)){
    digitalWrite(Charge_Pin, LOW);
    digitalWrite(Kicker_Pin, LOW);
    //Serial.println("Charge End");
    charging_state = true;
    charge_start = 0;
    last_charge_done = now;
  }

  // Perform kick if charged
  if(kick && charging_state){
    digitalWrite(Kicker_Pin, HIGH);
    delay(10);
    digitalWrite(Kicker_Pin, LOW);
    //delay(10);
    // After kick, reset to recharge again
    Serial.println("kick");
    charging_state = false;
  }
}

// INTERRUPT
void backlstouch(){ backtouch = true; }
void leftlstouch(){ lefttouch = true; }
void rightlstouch(){ righttouch = true; }