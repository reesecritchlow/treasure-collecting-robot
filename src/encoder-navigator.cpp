#include <Arduino.h>
#include "encoder-navigator.h"
#include "pid-system.h"
#include "config.h"

namespace Encoders {
    volatile int right_count = 0;
    volatile int left_count = 0;

    double right_destination_count;
    double left_destination_count;

    int right_direction;
    int left_direction;

    void rightEncoderInterrupt() {
        right_count++;
    }

    void leftEncoderInterrupt() {
        left_count++;
    }

    void encoderDriveStraight() {
        double transformed_PID_value = PID::computePID((right_count - left_count) * ENCODER_SCALING_VALUE);
        Drivetrain::changeDrivePID(transformed_PID_value);
    }

    void encoderSpin(bool direction) {
        double transformed_PID_value = PID::computePID((right_count - left_count) * ENCODER_SCALING_VALUE);
        Drivetrain::changeDrivePIDSpin(transformed_PID_value, direction);
    }

    void resetEncoderCounts() {
        right_count = 0;
        left_count = 0;
    }

    void setupEncoders() {
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderInterrupt, RISING);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderInterrupt, RISING);
    }

    void setStraightDestinationDistance(double distance_cm) {
        resetEncoderCounts();
        right_destination_count = distance_cm / DISTANCE_CM_PER_STEP;
        left_destination_count = distance_cm / DISTANCE_CM_PER_STEP;
    }

    void setSpinDestinationDistance(double angle_degrees) {
        double angle_radians = angle_degrees * PI / 180.0;
        resetEncoderCounts();
        right_destination_count = angle_radians * SPIN_RADIUS / DISTANCE_CM_PER_STEP;
        left_destination_count = angle_radians * SPIN_RADIUS / DISTANCE_CM_PER_STEP;
    }

    bool checkDestinationDistance() {
        return !(Encoders::left_count < Encoders::left_destination_count &&
               Encoders::right_count < Encoders::right_destination_count);
    }
}