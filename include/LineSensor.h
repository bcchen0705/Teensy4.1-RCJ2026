#ifndef LINESENSOR_H 
#define LINESENSOR_H
#include <Arduino.h>

struct LineData {
  uint32_t state = 0;
  bool valid = false;
};

class LineSensor{
    public:
        LineSensor();
        void update();

        LineData lineData;
    private:
        static float linesensorDegreelist[32];
        static int8_t linesensor_ver_cor[32];
};
extern LineSensor lineSensor;

#endif