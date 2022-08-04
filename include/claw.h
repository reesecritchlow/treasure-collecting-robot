#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    void setupServos();
    void setupHallSensor();

    void goUpperLimit();
    void goMiddle();
    void goLowerLimit();
    void open();
    void close1();
    void close2();
    void close3();
    void handleMagneticField();
}

#endif