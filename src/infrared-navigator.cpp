#include <Arduino.h>
#include "infrared-navigator.h"
#include "config.h"

namespace Infrared {
    double right_signal = analogRead(DEFAULT_RIGHT_SENSOR_PIN);
    double left_signal = analogRead(DEFAULT_RIGHT_SENSOR_PIN);
    int last_pid_multiplier = 0;
    int current_pid_multiplier = 0;

    void setupInfrared() {
        pinMode(DEFAULT_RIGHT_SENSOR_PIN, INPUT);
        pinMode(DEFAULT_LEFT_SENSOR_PIN, INPUT);
        pinMode(DEFAULT_RIGHT_CAP_RESET_PIN, OUTPUT);
        pinMode(DEFAULT_LEFT_CAP_RESET_PIN, OUTPUT);
    }

    double readRightSensor() {
        digitalWrite(DEFAULT_RIGHT_CAP_RESET_PIN, HIGH);
        delayMicroseconds(CAP_DELAY);
        digitalWrite(DEFAULT_RIGHT_CAP_RESET_PIN, LOW);
        delayMicroseconds(READ_DELAY);
        return analogRead(DEFAULT_RIGHT_SENSOR_PIN);
    }

    double readLeftSensor() {
        digitalWrite(DEFAULT_LEFT_CAP_RESET_PIN, HIGH);
        delayMicroseconds(CAP_DELAY);
        digitalWrite(DEFAULT_LEFT_CAP_RESET_PIN, LOW);
        delayMicroseconds(READ_DELAY);
        return analogRead(DEFAULT_LEFT_SENSOR_PIN);
    }

    void calculatePIDMultiplier() {
        last_pid_multiplier = current_pid_multiplier;

        bool left = OFF, right = OFF; // using each sensor, determine where we are pointed
        right_signal = readRightSensor();
        left_signal = readLeftSensor();

        if (left_signal > DEFAULT_LEFT_INFRARED_THRESHOLD) {
            left = ON;
        }
        if (right_signal > DEFAULT_RIGHT_INFRARED_THRESHOLD) {
            right = ON;
        }
        
        // Now determine which state
        if (left && right)
        {
            current_pid_multiplier = 0; // roughly pointed straight
            return;
        }
        else if (!left && !right && last_pid_multiplier > 0)
        {
            current_pid_multiplier = 3;
            return;
        }
        else if (!left && !right && last_pid_multiplier < 0)
        {
            current_pid_multiplier = -3;
            return;
        }
        else if (!left && right)
        {
            current_pid_multiplier = -1;
            return;
        }
        else if (left && !right)
        {
            current_pid_multiplier = 1;
            return;
        }

        current_pid_multiplier = 0; // if everything is failing just go straight
    }

    void runPIDCycle() {
        calculatePIDMultiplier();
        double transformed_PID_value = PID::computePID(current_pid_multiplier);
        Drivetrain::changeDrivePID(transformed_PID_value);
    }
}

