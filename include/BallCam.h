#ifndef BALLCAM_H 
#define BALLCAM_H
#include <Arduino.h>

// === Ball Camera Data 結構 ===
struct BallCamData{
    uint16_t angle = 65535;
    uint16_t dist = 65535;
    bool valid = false;
};

class BallCam{
    private:
        static uint8_t buffer[6];
        static uint8_t index = 0;
    public:
        BallCamData data;
        void update();
};

#endif
