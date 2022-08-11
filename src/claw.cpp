#include "config.h"
#include "claw.h"
#include "arm.h"
#include "state-machine.h"
#include "custom-servo.h"
#include "display-manager.h"

namespace Claw {

    volatile bool magnetic_idol = false;
    bool seen_magnet = false;

    void leftGoLowerLimit() {
        LeftTiltServo::write(LEFT_LOWER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void rightGoLowerLimit() {
        RightTiltServo::write(RIGHT_LOWER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void leftGoUpperLimit() {
        LeftTiltServo::write(LEFT_UPPER_ANGLE);
        delay(SERVO_WAIT_TIME);
   }

    void rightGoUpperLimit() {
        RightTiltServo::write(RIGHT_UPPER_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void open() {
        GrabServo::write(SERVO_OPEN_ANGLE);
        delay(SERVO_WAIT_TIME);
    }

    void close(int angle) {

            GrabServo::write(SERVO_CLOSE_ANGLE/SERVO_ANGLE_DIVISION * angle);
            delay(SERVO_WAIT_TIME/4);

    }

    //FOUDN IT

    void closeNoMagnet(int angle) {
        GrabServo::write(SERVO_CLOSE_ANGLE/SERVO_ANGLE_DIVISION * angle);
        delay(SERVO_WAIT_TIME/4);
    } 

    void leftGoMiddle() {
        
        LeftTiltServo::write((int)(LEFT_LOWER_ANGLE + LEFT_UPPER_ANGLE)/2);
        delay(SERVO_WAIT_TIME);
    }

    void rightGoMiddle() {
        RightTiltServo::write((int)(RIGHT_LOWER_ANGLE + RIGHT_UPPER_ANGLE)/2);
        delay(SERVO_WAIT_TIME);
    }

    void setupServos() {
        LeftTiltServo::write(LEFT_LOWER_ANGLE - 20);
        RightTiltServo::write(RIGHT_LOWER_ANGLE + 20);
        open();

        delay(SERVO_WAIT_TIME);


        LeftTiltServo::write(LEFT_UPPER_ANGLE);
        RightTiltServo::write(RIGHT_UPPER_ANGLE);
        GrabServo::write(SERVO_CLOSE_ANGLE);
    }

    void setupHallSensor() {
        pinMode(MAGNET_INTERRUPT_PIN, INPUT_PULLUP);
    }

    /**
     * @brief Waits 1 second, scans again then handles situation
     * 
     */
    void handleMagneticField() {

        delay(1000);

        if (digitalRead(MAGNET_INTERRUPT_PIN) < 1) {
            magnetic_idol = true;
            detachInterrupt(MAGNET_INTERRUPT_PIN);
            Claw::seen_magnet = true;
            Claw::leftGoMiddle();
            Claw::rightGoMiddle();
            StateMachine::searching_for_idol = false;
            return;            
        }
    }

    bool searchForMagneticField() {
        if (digitalRead(MAGNET_INTERRUPT_PIN) < 1) {
            magnetic_idol = true;
            Claw::seen_magnet = true;
            Claw::open();
            Claw::leftGoMiddle();
            Claw::rightGoMiddle();

            int clawCounter = 0;

            while(clawCounter <= SERVO_ANGLE_DIVISION) {
                Claw::closeNoMagnet(clawCounter);
                clawCounter += 1;
            }

            StateMachine::searching_for_idol = false;
            StateMachine::StateHandler = StateMachine::state_armHome;
            return true;
        }
        return false;
    }
}