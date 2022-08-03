#ifndef ENCODER_SYSTEM_H
#define ENCODER_SYSTEM_H

#include "pid-system.h"
#include "drivetrain.h"

namespace Encoders {
    extern volatile int right_count;
    extern volatile int left_count;

    extern double right_destination_count;
    extern double left_destination_count;

    extern int right_direction;
    extern int left_direction;

    void encoderDriveStraight();
    void encoderSpin(bool direction);

    void rightEncoderInterrupt();
    void leftEncoderInterrupt();

    void resetEncoderCounts();

    void setupEncoders();

    void setStraightDestinationDistance(double distance_cm);

    void setSpinDestinationDistance(double angle_degrees);
}



#endif