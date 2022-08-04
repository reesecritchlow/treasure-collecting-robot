#include <Arduino.h>
#include "drivetrain.h"
#include "config.h"

namespace Drivetrain {
    double speed_multiplier = DEFAULT_SPEED_MULTIPLIER;
    double right_speed = DEFAULT_BASE_SPEED;
    double left_speed = DEFAULT_BASE_SPEED;
    bool right_direction = true;
    bool left_direction = true;

    // Starts Driving the robot forwards
    void startDrive() {
        pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
        pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
    }

    // Changes drive speeds based on PID modifier value produced from a Navigator object
    void changeDrivePID(double pid_modifier_value) {
        if ((DEFAULT_BASE_SPEED + pid_modifier_value) * speed_multiplier > 0) {
            pwm_stop(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN);
            right_speed = (DEFAULT_BASE_SPEED + pid_modifier_value) * speed_multiplier;
            pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
            right_direction = true;
        } else {
            pwm_stop(DEFAULT_LEFT_FORWARD_MOTOR_PIN);
            left_speed = -1 * (DEFAULT_BASE_SPEED - pid_modifier_value) * speed_multiplier;
            pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
            left_direction = false;
        }

        if ((DEFAULT_BASE_SPEED - pid_modifier_value) * speed_multiplier > 0) {
            pwm_stop(DEFAULT_LEFT_BACKWARD_MOTOR_PIN);
            left_speed = (DEFAULT_BASE_SPEED - pid_modifier_value) * speed_multiplier;
            pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
            left_direction = true;
        } else {
            pwm_stop(DEFAULT_RIGHT_FORWARD_MOTOR_PIN);
            right_speed = -1 * (DEFAULT_BASE_SPEED - pid_modifier_value) * speed_multiplier;
            pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
            right_direction = false;
        }
    }

    void changeDrivePIDSpin(double pid_modifier_value, bool direction) {
        // True = clockwise, false = counterclockwise
        left_speed = (DEFAULT_BASE_SPEED - pid_modifier_value) * speed_multiplier;
        right_speed = (DEFAULT_BASE_SPEED + pid_modifier_value) * speed_multiplier;

        if (direction) {
            pwm_stop(DEFAULT_LEFT_FORWARD_MOTOR_PIN);
            pwm_stop(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN);

            pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
            pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);

            left_direction = false;
            right_direction = true;

            return;
        }

        pwm_stop(DEFAULT_LEFT_BACKWARD_MOTOR_PIN);
        pwm_stop(DEFAULT_RIGHT_FORWARD_MOTOR_PIN);

        pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
        pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);

        left_direction = true;
        right_direction = false;
    }

    // Changes overall speed multiplier
    void changeDriveMultiplier(double drive_multiplier) {
        speed_multiplier = drive_multiplier;
        right_speed = right_speed * drive_multiplier;
        left_speed = left_speed * drive_multiplier;

        if (right_direction) {
            pwm_stop(DEFAULT_RIGHT_FORWARD_MOTOR_PIN);
            pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
        } else {
            pwm_stop(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN);
            pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
        }

        if (left_direction) {
            pwm_stop(DEFAULT_LEFT_FORWARD_MOTOR_PIN);
            pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
        } else {
            pwm_stop(DEFAULT_LEFT_BACKWARD_MOTOR_PIN);
            pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
        }
    }

    // Kills all motor function.
    void killDrive() {
        pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
        pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
        pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
        pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
    }

    void halt() {
        if (right_direction) {
            pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
            pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
        } else {
            pwm_start(DEFAULT_RIGHT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, right_speed, DEFAULT_RESOLUTION);
            pwm_start(DEFAULT_RIGHT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
        }

        if (left_direction) {
            pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
            pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
        } else {
            pwm_start(DEFAULT_LEFT_FORWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, left_speed, DEFAULT_RESOLUTION);
            pwm_start(DEFAULT_LEFT_BACKWARD_MOTOR_PIN, DEFAULT_CLOCK_FREQUENCY, 0, DEFAULT_RESOLUTION);
        }

        delay(BRAKING_TIME);

        killDrive();
    }
}
