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
    extern bool left_sonar_on;
    extern int pickup_count;
    extern int min_dist;

    void setupArm();
    void setupSonars();

    bool setHome();
    void goHome();
    void goTo();
    void sleep();
    int currPos();
    void wake();

    int senseForIdol();

    int getDistanceToGo();

    void setSecondDistance();

    void secondScanLoop();

    void setupSecondScan();

    int getDistance(uint8_t trig_pin, uint8_t echo_pin);
}

#endif