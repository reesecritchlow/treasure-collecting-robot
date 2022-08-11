#include <Arduino.h>
#include "tape-navigator.h"
#include "drivetrain.h"
#include "infrared-navigator.h"
#include "config.h"

namespace Tape {

    double right_reflectance = analogRead(TAPE_RIGHT_SENSOR_PIN);
    double middle_reflectance = analogRead(TAPE_MIDDLE_SENSOR_PIN);
    double left_reflectance = analogRead(TAPE_LEFT_SENSOR_PIN);
    int last_pid_multiplier = DEFAULT_PID_STATE;
    int current_pid_multiplier = DEFAULT_PID_STATE;
    double transformed_PID = 0.0;
    bool tapeLost = false;

    int third_tape_state = THIRD_TAPE_STATE;
    int second_tape_state = SECOND_TAPE_STATE;

    void setupTapeTracking() {
        pinMode(TAPE_RIGHT_SENSOR_PIN, INPUT);
        pinMode(TAPE_MIDDLE_SENSOR_PIN, INPUT);
        pinMode(TAPE_LEFT_SENSOR_PIN, INPUT);
    }

    void calculateTapePIDMultiplier() {
        last_pid_multiplier = current_pid_multiplier;
        right_reflectance = analogRead(TAPE_RIGHT_SENSOR_PIN);
        middle_reflectance = analogRead(TAPE_MIDDLE_SENSOR_PIN);
        left_reflectance = analogRead(TAPE_LEFT_SENSOR_PIN);

        if (left_reflectance > CHICKEN_WIRE_THRESHOLD &&
            middle_reflectance > CHICKEN_WIRE_THRESHOLD &&
            right_reflectance > CHICKEN_WIRE_THRESHOLD) {
            current_pid_multiplier = 0;
            tapeLost = true;
            return;
        }

        bool left = OFF, right = OFF, mid = OFF; // using each sensor, determine where we are on the tape
        if (left_reflectance > TAPE_REFLECTANCE_THRESHOLD) {
            left = ON;
        }
        if (right_reflectance > TAPE_REFLECTANCE_THRESHOLD) {
            right = ON;
        }
        if (middle_reflectance > TAPE_REFLECTANCE_THRESHOLD) {
            mid = ON;
        }
        // Now determine which state
        if (left && right && mid) {
            current_pid_multiplier = 0;
            tapeLost = false;
            return;
        } else if (!left && !right && !mid && last_pid_multiplier == 0) // review if all are high or low
        {
            current_pid_multiplier = 0;
            return;
        } else if (!left && !right && !mid && last_pid_multiplier > 0) {
            current_pid_multiplier = third_tape_state;
            return;
        } else if (!left && !right && !mid && last_pid_multiplier < 0) {
            current_pid_multiplier = -third_tape_state;
            return;
        } else if (!left && !mid && right) {
            current_pid_multiplier = -SECOND_TAPE_STATE;
            return;
        } else if (left && !mid && !right) {
            current_pid_multiplier = SECOND_TAPE_STATE;
            return;
        } else if (!left && mid && right) {
            current_pid_multiplier = -FIRST_TAPE_STATE;
            tapeLost = false;
            return;
        } else if (left && mid && !right) {
            current_pid_multiplier = FIRST_TAPE_STATE;
            tapeLost = false;
            return;
        }
        tapeLost = false;
        current_pid_multiplier = 0; // mid sees tape, go straight
    }

    void runPIDCycle() {
        calculateTapePIDMultiplier();
        transformed_PID = PID::computePID(current_pid_multiplier);
        Drivetrain::changeDrivePID(transformed_PID);
    }
}    