#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    extern volatile bool magnetic_idol;
    extern bool seen_magnet;

    void setupServos();
    void setupHallSensor();

    void leftGoLowerLimit();
    void leftGoUpperLimit();
    void leftGoMiddle();
    

    void rightGoLowerLimit();
    void rightGoUpperLimit();
    void rightGoMiddle();
    

    void open();
    void close(int angle);
    void handleMagneticField();

    bool searchForMagneticField();

    void closeNoMagnet(int angle);
}

#endif