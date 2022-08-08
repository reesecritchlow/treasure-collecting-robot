#include <Arduino.h>
#include "drivetrain.h"
#include "config.h"
#include "display-manager.h"

namespace Drivetrain {
    double speed_multiplier = DRIVETRAIN_SPEED_MULTIPLIER;
    double right_speed = DRIVETRAIN_BASE_SPEED;
    double left_speed = DRIVETRAIN_BASE_SPEED;
    bool right_direction = true;
    bool left_direction = true;

    // Starts Driving the robot forwards
    void startDrive() {
        pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
        pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
    }

    // Changes drive speeds based on PID modifier value produced from a Navigator object
    void changeDrivePID(double pid_modifier_value) {
        if ((DRIVETRAIN_BASE_SPEED + pid_modifier_value) * speed_multiplier > 0) {
            pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            right_speed = (DRIVETRAIN_BASE_SPEED + pid_modifier_value) * speed_multiplier;
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
            right_direction = true;
        } else {
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            right_speed = -1 * (DRIVETRAIN_BASE_SPEED + pid_modifier_value) * speed_multiplier;
            pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
            right_direction = false;
        }

        if ((DRIVETRAIN_BASE_SPEED - pid_modifier_value) * speed_multiplier > 0) {
            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            left_speed = (DRIVETRAIN_BASE_SPEED - pid_modifier_value) * speed_multiplier;
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
            left_direction = true;
        } else {
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            left_speed = -1 * (DRIVETRAIN_BASE_SPEED - pid_modifier_value) * speed_multiplier;
            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
            left_direction = false ;
        }
    }

    void changeDrivePIDSpin(double pid_modifier_value, bool direction) {
        // True = clockwise, false = counterclockwise
        left_speed = (DRIVETRAIN_BASE_SPEED - pid_modifier_value) * speed_multiplier;
        right_speed = (DRIVETRAIN_BASE_SPEED + pid_modifier_value) * speed_multiplier;

        if (direction) {
            pwm_stop(LEFT_FORWARD_MOTOR_PIN);
            pwm_stop(RIGHT_BACKWARD_MOTOR_PIN);

            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);

            left_direction = false;
            right_direction = true;

            return;
        }

        pwm_stop(LEFT_BACKWARD_MOTOR_PIN);
        pwm_stop(RIGHT_FORWARD_MOTOR_PIN);

        pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
        pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);

        left_direction = true;
        right_direction = false;
    }

    // Changes overall speed multiplier
    void changeDriveMultiplier(double drive_multiplier) {
        speed_multiplier = drive_multiplier;
        right_speed = right_speed * drive_multiplier;
        left_speed = left_speed * drive_multiplier;

        if (right_direction) {
            pwm_stop(RIGHT_FORWARD_MOTOR_PIN);
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
        } else {
            pwm_stop(RIGHT_BACKWARD_MOTOR_PIN);
            pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
        }

        if (left_direction) {
            pwm_stop(LEFT_FORWARD_MOTOR_PIN);
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
        } else {
            pwm_stop(LEFT_BACKWARD_MOTOR_PIN);
            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
        }
    }

    // Kills all motor function.
    void killDrive() {
        pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
        pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
        pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
        pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
    }

    void halt() {
        if (left_direction) {
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
        } else {
            pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, left_speed, PWM_SIGNAL_RESOLUTION);
            
        }
        if (right_direction) {
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
        } else {
            pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, right_speed, PWM_SIGNAL_RESOLUTION);
        }
        delay(BRAKING_TIME);
        killDrive();
    }

    void haltTape() {
        pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
        pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);

        pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
        pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);

        delay(BRAKING_TIME);
        killDrive();
    }
}
