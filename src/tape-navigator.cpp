#include <Arduino.h>
#include "tape-navigator.h"
#include "drivetrain.h"
#include "infrared-navigator.h"
#include "config.h"

namespace Tape {

    double right_reflectance = analogRead(RIGHT_REFLECTANCE_PIN);
    double middle_reflectance = analogRead(MIDDLE_REFLECTANCE_PIN);
    double left_reflectance = analogRead(LEFT_REFLECTANCE_PIN);
    int last_pid_multiplier = DEFAULT_PID_STATE;
    int current_pid_multiplier = DEFAULT_PID_STATE;
    double transformed_PID = 0.0;

    void setupTapeTracking() {
        pinMode(RIGHT_REFLECTANCE_PIN, INPUT);
        pinMode(MIDDLE_REFLECTANCE_PIN, INPUT);
        pinMode(LEFT_REFLECTANCE_PIN, INPUT);
    }

    void calculateTapePIDMultiplier() {
        last_pid_multiplier = current_pid_multiplier;
        right_reflectance = analogRead(RIGHT_REFLECTANCE_PIN);
        middle_reflectance = analogRead(MIDDLE_REFLECTANCE_PIN);
        left_reflectance = analogRead(LEFT_REFLECTANCE_PIN);

        bool left = OFF, right = OFF, mid = OFF; // using each sensor, determine where we are on the tape
        if (left_reflectance > DEFAULT_REFLECTANCE_THRESHOLD)
        {
            left = ON;
        }
        if (right_reflectance > DEFAULT_REFLECTANCE_THRESHOLD)
        {
            right = ON;
        }
        if (middle_reflectance > DEFAULT_REFLECTANCE_THRESHOLD)
        {
            mid = ON;
        } 
        // Now determine which state
        if (left && right && mid)
        {
            current_pid_multiplier = 0; // switch to IR
            return;
        }
        else if (!left && !right && !mid && last_pid_multiplier == 0) // review if all are high or low
        {
            current_pid_multiplier = -1; // chicken wire, go slight left
            return;
        }
        else if (!left && !right && !mid && last_pid_multiplier > 0)
        {
            current_pid_multiplier = 7;
            return;
        }
        else if (!left && !right && !mid && last_pid_multiplier < 0)
        {
            current_pid_multiplier = -7;
            return;
        }
        else if (!left && !mid && right)
        {
            current_pid_multiplier = -2;
            return;
        }
        else if (left && !mid && !right)
        {
            current_pid_multiplier = 2;
            return;
        }
        else if (!left && mid && right)
        {
            current_pid_multiplier = -1;
            return;
        }
            else if (left && mid && !right)
        {
            current_pid_multiplier = 1;
            return;
        }
        current_pid_multiplier = 0; // mid sees tape, go straight
    }

    void runPIDCycle() {
        calculateTapePIDMultiplier();
        transformed_PID = PID::computePID(current_pid_multiplier);
        Drivetrain::changeDrivePID(transformed_PID);
    }
}    