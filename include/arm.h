/** @file */
#ifndef ARM_H
#define ARM_H
#include <stdlib.h>
#include <Arduino.h>
#include <AccelStepper.h>

namespace Arm {
    extern int move_distance;
    extern int idol_position;
    extern bool see_idol_right;
    extern bool see_idol_left;

    void armSetup();

    bool armSetHome();
    void armGoHome();
    void armGoTo();
    void armSleep();
    int armCurrPos();

    int armSenseForIdol();
    void armSonarSetup();

    int armGetDistanceToGo();
}

#endif