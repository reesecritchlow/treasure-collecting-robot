#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

namespace Drivetrain {
    extern double speed_multiplier;
    extern double right_speed;
    extern double left_speed;
    extern bool right_direction;
    extern bool left_direction;

    void startDrivetrainStraight();
    void changeDrivePID(double pid_modifier_value);
    void changeDrivePIDSpin(double pid_modifier_value, bool direction);
    void killDrive();
    void changeDriveMultiplier(double drive_multiplier);
    void halt();
}

#endif