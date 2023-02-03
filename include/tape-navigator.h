#ifndef TAPE_NAVIGATOR_H
#define TAPE_NAVIGATOR_H

#include <stdlib.h>
#include <Arduino.h>
#include "pid-system.h"
#include "drivetrain.h"
#include "infrared-navigator.h"

namespace Tape {
    extern double right_reflectance;
    extern double middle_reflectance;
    extern double left_reflectance;
    extern int last_pid_multiplier;
    extern int current_pid_multiplier;
    extern double transformed_PID;
    extern bool tapeLost;

    extern int second_tape_state;
    extern int third_tape_state;

    /**
     * @brief Calculates the current state of the tape tracker, modifies the current_pid_multiplier
     * variable to contain the current error state.
     * 
     */
    void calculateTapeError();

    /**
     * @brief Sets up pins for the reflectance sensors.
     * 
     */
    void setupTapeTracking();

    /**
     * @brief Changes drive speed/direction of the robot based on the 
     * current state of the tape tracking system.
     * 
     */
    void runPIDCycle();
}

#endif