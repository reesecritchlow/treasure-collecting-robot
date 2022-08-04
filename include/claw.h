#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    void setupServos();
    void setupHallSensor();

    void leftGoLowerLimit();
    void leftGoUpperLimit();
    void leftGoMiddle();
    

    void rightGoLowerLimit();
    void rightGoUpperLimit();
    void rightGoMiddle();
    

    void open();
    void close1();
    void close2();
    void close3();
    void handleMagneticField();
}

#endif