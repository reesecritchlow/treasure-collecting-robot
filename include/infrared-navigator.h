#ifndef INFRARED_NAVIGATOR_H
#define INFRARED_NAVIGATOR_H

#include <Arduino.h>
#include "pid-system.h"
#include "drivetrain.h"

namespace Infrared {
    extern double right_signal;
    extern double left_signal;
    extern int last_pid_multiplier;
    extern int current_pid_multiplier;

    double readRightSensor();
    double readLeftSensor();

    void calculatePIDMultiplier();

    void runPIDCycle();

    void setupInfrared();
}



#endif