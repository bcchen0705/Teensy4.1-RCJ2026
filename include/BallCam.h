#ifndef BALLCAM_H 
#define BALLCAM_H
#include <Arduino.h>

class BallCam{
    public:
        uint16_t angle = 65535;
        uint16_t dist = 65535;
        bool valid = false;

        static void readBallCam();
};
extern BallCam ballData;

#endif
