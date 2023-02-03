#ifndef ENCODER_SYSTEM_H
#define ENCODER_SYSTEM_H

#include "pid-system.h"
#include "drivetrain.h"

namespace Encoders {
    extern volatile int right_count; // Current number of counts in the right encoder.
    extern volatile int left_count; // Current number of counts in the left encoder.

    extern double right_destination_count; // Number of target counts for the right encoder.
    extern double left_destination_count; // Number of target counts for the left encoder.

    extern int right_direction; // Direction of the right side to drive.
    extern int left_direction; // Direction of the left side to drive.

    /**
     * @brief Drives the robot straight forward, adjusting based on the deviance between the right_count
     * and the left_count.
     * 
     */
    void encoderDriveStraight();

    /**
     * @brief Spins the robot in a specified direction, adjusting based on the deviance between the 
     * right_count and the left_count.
     * 
     * @param direction Direction to spin in, true for CW, false for CCW. 
     */
    void encoderSpin(bool direction);

    /**
     * @brief Interrupt handler for the right encoder pulses. Increments right_count by 1 each time.
     * 
     */
    void rightEncoderInterrupt();

    /**
     * @brief Interrupt handler for the left encoder pulses. Increments left_count by 1 each time.
     * 
     */
    void leftEncoderInterrupt();

    /**
     * @brief Resets left_count and right_count to zero.
     * 
     */
    void resetEncoderCounts();

    /**
     * @brief Attaches the rightEncoderInterrupt and leftEncoderInterrupt to their
     * corresponding interrupt pins.
     * 
     */
    void setupEncoders();

    /**
     * @brief Converts a distance in centimeters to a number of counts for the encoders,
     * and sets left/right_counts accordingly.
     * 
     * @param distance_cm Distance in cm for the robot to drive in a straight line for.
     */
    void setStraightDestinationDistance(double distance_cm);

    /**
     * @brief Converts an angle in degrees to a number of counts of the encoders, and sets
     * left/right_counts accordingly.
     * 
     * @param angle_degrees 
     */
    void setSpinDestinationDistance(double angle_degrees);

    /**
     *
     * @return true if the encoders have reached their desired distance, false if not.
     */
    bool checkDestinationDistance();
}



#endif