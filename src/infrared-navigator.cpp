#include <Arduino.h>
#include "infrared-navigator.h"
#include "config.h"

namespace Infrared {
    double right_signal = analogRead(INFRARED_RIGHT_SENSOR_PIN);
    double left_signal = analogRead(INFRARED_RIGHT_SENSOR_PIN);
    int last_pid_multiplier = 0;
    int current_pid_multiplier = 0;

    void setupInfrared() {
        pinMode(INFRARED_RIGHT_SENSOR_PIN, INPUT);
        pinMode(INFRARED_LEFT_SENSOR_PIN, INPUT);
        pinMode(INFRARED_RIGHT_CAP_RESET_PIN, OUTPUT);
        pinMode(INFRARED_LEFT_CAP_RESET_PIN, OUTPUT);
    }

    void readRightSensor() {
        digitalWrite(INFRARED_RIGHT_CAP_RESET_PIN, HIGH);
        delayMicroseconds(CAP_DELAY);
        digitalWrite(INFRARED_RIGHT_CAP_RESET_PIN, LOW);
        delayMicroseconds(READ_DELAY);
        right_signal = analogRead(INFRARED_RIGHT_SENSOR_PIN);
    }

    void readLeftSensor() {
        digitalWrite(INFRARED_LEFT_CAP_RESET_PIN, HIGH);
        delayMicroseconds(CAP_DELAY);
        digitalWrite(INFRARED_LEFT_CAP_RESET_PIN, LOW);
        delayMicroseconds(READ_DELAY);
        left_signal = analogRead(INFRARED_LEFT_SENSOR_PIN);
    }

    void calculatePIDMultiplier() {
        last_pid_multiplier = current_pid_multiplier;

        bool left = OFF, right = OFF; // using each sensor, determine where we are pointed
        readRightSensor();
        readLeftSensor();

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

