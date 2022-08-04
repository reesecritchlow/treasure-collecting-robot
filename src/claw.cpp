#include "config.h"
#include "claw.h"
#include "arm.h"
#include "state-machine.h"

Servo servoGrab;
Servo servoTiltLeft;
Servo servoTiltRight;

namespace Claw {
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

    void close() {
        servoGrab.write(SERVO_CLOSE_ANGLE/4);
        delay(SERVO_WAIT_TIME/2);

        servoGrab.write(SERVO_CLOSE_ANGLE/2);
        delay(SERVO_WAIT_TIME);

        servoGrab.write(SERVO_CLOSE_ANGLE);
        delay(SERVO_WAIT_TIME);
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
        servoGrab.write(SERVO_CLOSE_ANGLE);
    }

    void setupHallSensor() {
        pinMode(MAGNET_INTERRUPT_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MAGNET_INTERRUPT_PIN), , FALLING);
    }

    void handleMagneticField() {
        
       
    }
}