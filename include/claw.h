#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    extern volatile bool magnetic_idol;

    void setupServos();
    void setupHallSensor();

    void leftGoLowerLimit();
    void leftGoUpperLimit();
    void leftGoMiddle();
    

    void rightGoLowerLimit();
    void rightGoUpperLimit();
    void rightGoMiddle();
    

    void open();
    void close();
    void handleMagneticField();
}

#endif