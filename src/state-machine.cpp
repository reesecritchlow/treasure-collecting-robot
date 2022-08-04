#include "drivetrain.h"
#include "tape-navigator.h"
#include "infrared-navigator.h"
#include "encoder-navigator.h"
#include "config.h"
#include "display-manager.h"
#include "arm.h"
#include "claw.h"

namespace StateMachine {

    int cycleCounter = 0;
    int clawCounter = 0;

    void state1_tape_following();

    void (*StateHandler)() = state1_tape_following;

    void state2_infrared_tracking();
    void state3_drive_straight();
    void state5_do_nothing();

    void state1_tape_following() {
        digitalWrite(INTERNAL_LED, HIGH);
            Tape::runPIDCycle();
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayTapeMetrics();
        }
        Infrared::left_signal = Infrared::readLeftSensor();
        Infrared::right_signal = Infrared::readRightSensor();
    }

    void state2_infrared_tracking() {
        Infrared::runPIDCycle();
    }

    void state_do_nothing() {
    
    }

    void state6_drive_straight_again() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }

        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderDriveStraight();
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::stopDrive();
        digitalWrite(INTERNAL_LED, HIGH);

        StateHandler = state5_do_nothing;
    }

    void state4_spin() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }
        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderSpin(CLOCKWISE);
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::stopDrive();
        digitalWrite(INTERNAL_LED, LOW);
        Encoders::setStraightDestinationDistance(10.0);
        StateHandler = state6_drive_straight_again;
    }

    void state3_drive_straight() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }

        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderDriveStraight();
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::stopDrive();
        digitalWrite(INTERNAL_LED, HIGH);
        Encoders::setSpinDestinationDistance(45.0);
        StateHandler = state4_spin;
    }

    void state_sensingIdol() {
        Arm::idol_position = Arm::senseForIdol();
        Serial3.println(Arm::idol_position);
        Arm::move_distance = Arm::idol_position; // = idol_position
        if(Arm::idol_position != 0)
            StateHandler = state_moveToIdol;
    }

    void state_moveToIdol() {
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            delay(1000);
            StateHandler = state_grabbingIdol;
        }
    }

    void state_grabbingIdol() {
        Claw::open();
        if(Arm::see_idol_left) {
            Claw::leftGoLowerLimit();
            if(!Claw::seen_magnet) {
                while(!(Claw::magnetic_idol) && (clawCounter <= SERVO_ANGLE_DIVISION)) {
                    Claw::close(clawCounter);
                    clawCounter += 1;
                }
                if(Claw::magnetic_idol) {
                    Claw::open();
                    delay(SERVO_WAIT_TIME);
                    Claw::leftGoUpperLimit();
                    return;
                }
            } else {
                Claw::close(SERVO_ANGLE_DIVISION);
            }
            Claw::leftGoMiddle();
            StateHandler = state_goToBin;
            clawCounter = 0;
            return;
        }
        // insert above code *****************************************************************
        if(Arm::see_idol_right) {
            Claw::rightGoLowerLimit();
            if(!Claw::seen_magnet) {
                while(!(Claw::magnetic_idol) && (clawCounter <= SERVO_ANGLE_DIVISION)) {
                    Claw::close(clawCounter);
                    clawCounter += 1;
                }
                if(Claw::magnetic_idol) {
                    Claw::open();
                    delay(SERVO_WAIT_TIME);
                    Claw::leftGoUpperLimit();
                    return;
                }
            } else {
                Claw::close(SERVO_ANGLE_DIVISION);
            }
            Claw::rightGoMiddle();
            StateHandler = state_goToBin;
            clawCounter = 0;
            return;
        }
    }

    void state_goToBin() {
        if(Arm::idol_position > 0) {
            Arm::move_distance = -BIN_DIST;
        } else if (Arm::idol_position < 0) {
            Arm::move_distance = BIN_DIST;
        }
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            StateHandler = state_goingHome;
            Serial3.println("droppingIdol");
        }
    }

    void state_dropIdol() {
        delay(1000);
        Claw::open();
        StateHandler = state_goingHome;
    }

    void state_armThruArch() {
        Arm::goHome();
        Claw::leftGoLowerLimit();
        Claw::rightGoLowerLimit();
         
        Claw::close(SERVO_ANGLE_DIVISION);
    }

    void state_goingHome() {
        Arm::goHome();
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        Arm::see_idol_left = false;
        Arm::see_idol_right = false;
        StateHandler = state_do_nothing;
    }

    void state_magneticField() {

        //claw open fully

        //claw raise up

        //stepper go home
        Claw::seen_magnet = true;
        detachInterrupt(MAGNET_INTERRUPT_PIN);
        StateHandler = state_goingHome;
    }
}