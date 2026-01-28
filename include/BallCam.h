#ifndef BALLCAM_H 
#define BALLCAM_H
#include <Arduino.h>

// === Ball Camera Data 結構 ===
struct BallCamData{
    uint16_t angle = 65535;
    uint16_t dist = 65535;
    bool valid = false;
};
extern BallCamData ballCamData;

void readBallCam();

#endif
