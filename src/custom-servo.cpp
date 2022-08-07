#include "custom-servo.h"
#include <Arduino.h>
#include "config.h"

double convertPosition(double original_position) {
    return map(original_position, LIBRARY_SERVO_MINIMUM_RANGE, LIBRARY_SERVO_MAXIMUM_RANGE, CUSTOM_SERVO_MINIMUM_RANGE, CUSTOM_SERVO_MAXIMUM_RANGE);
}

namespace LeftTiltServo {
    double last_position;

    void write(double position) {
        double modified_position = convertPosition(position);
        last_position = modified_position;
        pwm_start(SERVO_PIN_TILT_LEFT, CUSTOM_SERVO_READ_FREQUENCY, modified_position, CUSTOM_SERVO_RESOLUTION);
    }

    void stopSignal() {
        pwm_start(SERVO_PIN_TILT_LEFT, CUSTOM_SERVO_READ_FREQUENCY, 0, CUSTOM_SERVO_RESOLUTION);
    }
}

namespace RightTiltServo {
    double last_position;

    void write(double position) {
        double modified_position = convertPosition(position);
        last_position = modified_position;
        pwm_start(SERVO_PIN_TILT_RIGHT, CUSTOM_SERVO_READ_FREQUENCY, modified_position, CUSTOM_SERVO_RESOLUTION);
    }

    void stopSignal() {
        pwm_start(SERVO_PIN_TILT_RIGHT, CUSTOM_SERVO_READ_FREQUENCY, 0, CUSTOM_SERVO_RESOLUTION);
    }
}

namespace GrabServo {
    double last_position;

    void write(double position) {
        double modified_position = convertPosition(position);
        last_position = modified_position;
        pwm_start(SERVO_PIN_GRAB, 100, 50, RESOLUTION_10B_COMPARE_FORMAT);
    }

    void stopSignal() {
        pwm_start(SERVO_PIN_GRAB, CUSTOM_SERVO_READ_FREQUENCY, 0, CUSTOM_SERVO_RESOLUTION);
    }
}