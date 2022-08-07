#include "config.h"
#include "claw.h"
#include "arm.h"
#include "state-machine.h"

Servo servoGrab;
Servo servoTiltLeft;
Servo servoTiltRight;

namespace Claw {

    volatile bool magnetic_idol = false;
    bool seen_magnet = false;

    void leftGoLowerLimit() {
        servoTiltLeft.write(LEFT_LOWER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void rightGoLowerLimit() {
        servoTiltRight.write(RIGHT_LOWER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void leftGoUpperLimit() {
        servoTiltLeft.write(LEFT_UPPER_ANGLE);
         delay(SERVO_WAIT_TIME);
   }

    void rightGoUpperLimit() {
        servoTiltRight.write(RIGHT_UPPER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void open() {
        servoGrab.write(SERVO_OPEN_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void close(int angle) {
        if (!magnetic_idol) {
            servoGrab.write(SERVO_CLOSE_ANGLE/SERVO_ANGLE_DIVISION * angle);
            delay(SERVO_WAIT_TIME/4);
        }
    } 

    void leftGoMiddle() {
        servoTiltLeft.write((LEFT_LOWER_ANGLE + LEFT_UPPER_ANGLE)/2);
        delay(SERVO_WAIT_TIME);
    }

    void rightGoMiddle() {
        servoTiltRight.write((RIGHT_LOWER_ANGLE + RIGHT_UPPER_ANGLE)/2);
        delay(SERVO_WAIT_TIME);
    }

    void setupServos() {
        servoGrab.attach(SERVO_PIN_GRAB);
        servoTiltLeft.attach(SERVO_PIN_TILT_LEFT);
        servoTiltRight.attach(SERVO_PIN_TILT_RIGHT);

        servoTiltLeft.write(LEFT_LOWER_ANGLE);
        servoTiltRight.write(RIGHT_LOWER_ANGLE);
        open();

        delay(SERVO_WAIT_TIME);

        servoTiltLeft.write(LEFT_UPPER_ANGLE);
        servoTiltRight.write(RIGHT_UPPER_ANGLE);
        close(SERVO_ANGLE_DIVISION);
        // servoGrab.write(SERVO_CLOSE_ANGLE);
    }

    void setupHallSensor() {
        pinMode(MAGNET_INTERRUPT_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MAGNET_INTERRUPT_PIN), handleMagneticField, FALLING);
    }

    void handleMagneticField() {
        magnetic_idol = true;
        StateMachine::StateHandler = StateMachine::state_magneticField;
     }
}