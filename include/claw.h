#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    void setupServos();

    void goUpperLimit();
    void goMiddle();
    void goLowerLimit();
    void open();
    void close();
}

#endif