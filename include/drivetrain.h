#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>

namespace Drivetrain {
    extern double speed_multiplier;
    extern double right_speed;
    extern double left_speed;
    extern bool right_direction;
    extern bool left_direction;

    void startDrive();
    void startDrivetrainStraight();
    void changeDrivePID(double pid_modifier_value);
    void changeDrivePIDStraight(double pid_multiplier_value);
    void changeDrivePIDSpin(double pid_modifier_value, bool direction);
    void killDrive();
    void changeDriveMultiplier(double drive_multiplier);
    void changeDrivePIDBackwards(double pid_modifier_value);
    void haltEncoders();
    void haltFirstIdol();
}

#endif