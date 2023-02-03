#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

namespace Drivetrain {
    extern double speed_multiplier;
    extern double right_speed;
    extern double left_speed;
    extern bool right_direction;
    extern bool left_direction;

    /**
     * @brief Starts the motors spinning forwards with the baseline PWM signal/speed.
     * 
     */
    void startDrive();

    /**
     * @brief Changes the current speed inputted to the motors based on a modifier value.
     * Results in the robot turning more or less in a direction based on the modifier value.
     * 
     * @param pid_modifier_value    Amount of directional change for the robot, outputted from
     * the PID::computePID function.
     */
    void changeDrivePID(double pid_modifier_value);

    /**
     * @brief Spins the robot in a specified direction, and is adjusted by a PID modifier value.
     * 
     * @param pid_modifier_value    PID Modifier value, outputted from the PID::computePID function.
     * @param direction             Direction to spin in, true for clockwise, false for counterclockwise.
     */
    void changeDrivePIDSpin(double pid_modifier_value, bool direction);

    /**
     * @brief Stops all signals going to the motors. Sends a PWM signal with zero duty cycle to each motor.
     * 
     */
    void killDrive();

    /**
     * @brief Changes the base speed of the robot.
     * 
     * @param drive_multiplier Multiplier to adjust the speed by.
     */
    void changeDriveMultiplier(double drive_multiplier);

    /**
     * @brief Applies a "braking" function to the motors (sends a PWM signal in reverse)
     * for a short period of time.
     * 
     */
    void haltEncoders();

    /**
     * @brief Variant of haltEncoders to compensate for strange behaviours of haltEncoders for
     * the first idol pickup.
     * 
     */
    void haltFirstIdol();
}

#endif