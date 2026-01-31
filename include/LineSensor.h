#ifndef LINESENSOR_H 
#define LINESENSOR_H
#include <Arduino.h>

#define EMERGENCY_THRESHOLD 50 

struct LineData {
  uint32_t state = 0;
  bool valid = false;
};

class LineSensor{
    public:
        LineSensor();
        void update();
        static float linesensorDegreelist[32];
        LineData lineData;
    private:
        static int8_t linesensor_ver_cor[32];
};
extern LineSensor lineSensor;

#endif