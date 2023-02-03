#ifndef INFRARED_NAVIGATOR_H
#define INFRARED_NAVIGATOR_H

#include <Arduino.h>
#include "pid-system.h"
#include "drivetrain.h"

namespace Infrared {
    extern double right_signal; // Strength of the IR signal coming from the right sensor.
    extern double left_signal; // Strength of the IR signal coming from the left sensor.
    extern int last_pid_multiplier;
    extern int current_pid_multiplier;

    /**
     * @brief Reads the current value of the right IR sensor, modifying right_signal.
     * 
     */
    void readRightSensor();

    /**
     * @brief Reads the current value of the left IR sensor, modifying left_signal.
     * 
     */
    void readLeftSensor();

    /**
     * @brief Calculates a PID multiplier based on the left and right signal values.
     * 
     */
    void calculatePIDMultiplier();

    /**
     * @brief Translates the pid multiplier to the drivetrain and changes robot 
     * drive characteristics accordingly.
     * 
     */
    void runPIDCycle();

    /**
     * @brief Sets up pins for the Infrared sensors.
     * 
     */
    void setupInfrared();
}



#endif