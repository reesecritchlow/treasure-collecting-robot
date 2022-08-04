#include "config.h"
#include "claw.h"
#include "arm.h"

Servo servoGrab;
Servo servoTiltLeft;
Servo servoTiltRight;

namespace Claw {
    void goLowerLimit() {
        if(Arm::see_idol_left) {
            servoTiltLeft.write(LEFT_LOWER_ANGLE);
            return;
        }
        servoTiltRight.write(RIGHT_LOWER_ANGLE);
    }

    void open() {
        servoGrab.write(SERVO_OPEN_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void close() {
        servoGrab.write(SERVO_CLOSE_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void goMiddle() {
        if(Arm::see_idol_left) {
            servoTiltLeft.write((LEFT_LOWER_ANGLE + LEFT_UPPER_ANGLE)/2);
            delay(SERVO_WAIT_TIME);
            return; 
        }
        servoTiltRight.write((RIGHT_LOWER_ANGLE + RIGHT_UPPER_ANGLE)/2);
        delay(SERVO_WAIT_TIME);
    }

    void goUpperLimit() {
        if(Arm::see_idol_left) {
            servoTiltLeft.write(LEFT_UPPER_ANGLE);
            return;
        }
        servoTiltRight.write(RIGHT_UPPER_ANGLE);
    }

    void setupServos() {
        servoGrab.attach(SERVO_PIN_GRAB);
        servoTiltLeft.attach(SERVO_PIN_TILT_LEFT);
        servoTiltRight.attach(SERVO_PIN_TILT_RIGHT);

        servoTiltLeft.write(LEFT_LOWER_ANGLE);
        servoTiltRight.write(RIGHT_LOWER_ANGLE);
        servoGrab.write(SERVO_OPEN_ANGLE);

        delay(SERVO_WAIT_TIME);

        servoTiltLeft.write(LEFT_UPPER_ANGLE);
        servoTiltRight.write(RIGHT_UPPER_ANGLE);
        servoGrab.write(SERVO_CLOSE_ANGLE);
    }
}