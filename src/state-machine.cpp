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
    bool following_tape = true;

    void state_tape_following();

    // Initial State
    void (*StateHandler)() = state_tape_following;

    void state_infrared_tracking();
    void state_drive_straight();
    void state_do_nothing();
    void state_moveToIdol();
    void state_grabIdol();
    void state_lowerArmForIdol();
    void state_goToBin();
    void state_raiseForDrop();
    void state_goingHome();
    void state_dropIdol();
    void state_armThruArch();
    void state_search_for_infrared_at_arch();
    void state_spin_for_third_idol();
    void state_drive_to_third_idol();

    void state_tape_following() {
        Arm::idol_position = Arm::senseForIdol();

        digitalWrite(INTERNAL_LED, HIGH);
            Tape::runPIDCycle();
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayTapeMetrics();
        }

       /* if(Arm::idol_position != 0) {
            StateHandler = state_moveToIdol;
            Drivetrain::halt();
        }*/
        Infrared::readRightSensor();

        if (Infrared::right_signal < INFRARED_TRANSITION_LEFT_THRESHOLD) {
            return;
        }
        Drivetrain::halt();
        StateHandler = state_search_for_infrared_at_arch;
        Encoders::setSpinDestinationDistance(30.0);
    }

    void state_search_for_infrared_at_arch() {
        Encoders::encoderSpin(COUNTER_CLOCKWISE);
        Infrared::readRightSensor();
        Infrared::readLeftSensor();
        if (
                Infrared::left_signal < INFRARED_ARCH_ALIGNMENT_THRESHOLD ||
                Infrared::right_signal < INFRARED_ARCH_ALIGNMENT_THRESHOLD ||
                !Encoders::checkDestinationDistance()
                ) {
            return;
        }
        Drivetrain::halt();
        Encoders::setStraightDestinationDistance(10.0);
    }

    void state_push_out_of_arch() {
        if (Encoders::checkDestinationDistance()) {
            StateHandler = state_spin_for_third_idol;
            Encoders::setSpinDestinationDistance(30.0);
        }
        Encoders::encoderDriveStraight();
    }

    void state_spin_for_third_idol() {
        if (Encoders::checkDestinationDistance()) {
            Encoders::setStraightDestinationDistance(20.0);
            StateHandler = state_drive_to_third_idol;
            return;
        }
        Encoders::encoderSpin(COUNTER_CLOCKWISE);
    }

    void state_drive_to_third_idol() {
        if (Encoders::checkDestinationDistance()) {
            StateHandler = state_do_nothing;
        }
        Encoders::encoderDriveStraight();
    }

    void state_sensingIdol() {
        Arm::idol_position = Arm::senseForIdol();
        // Serial3.println(Arm::idol_position);
        Arm::move_distance = Arm::idol_position;
        if(Arm::idol_position != 0)
            StateHandler = state_moveToIdol;
    }

    void state_infrared_tracking() {
        following_tape = false;
        Infrared::runPIDCycle();
    }

    void state_do_nothing() {
    
    }

    void state_drive_straight_again() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }

        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderDriveStraight();
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::halt();
        digitalWrite(INTERNAL_LED, HIGH);

        StateHandler = state_do_nothing;
    }

    void state_spin() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }
        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderSpin(CLOCKWISE);
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::halt();
        digitalWrite(INTERNAL_LED, LOW);
        Encoders::setStraightDestinationDistance(10.0);
        StateHandler = state_drive_straight_again;
    }

    void state_drive_straight() {
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }

        if (Encoders::left_count < Encoders::left_destination_count || Encoders::right_count < Encoders::right_destination_count) {
            Encoders::encoderDriveStraight();
            return;
        }
        Display::displayEncoderMetrics();
        Drivetrain::halt();
        digitalWrite(INTERNAL_LED, HIGH);
        Encoders::setSpinDestinationDistance(45.0);
        StateHandler = state_spin;
    }


    void state_moveToIdol() {
        Arm::move_distance = Arm::idol_position;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            delay(1000);
            StateHandler = state_lowerArmForIdol;
        }
    }

    void state_lowerArmForIdol() {
        Claw::open();
        if(Arm::see_idol_left) {
            Claw::leftGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        if(Arm::see_idol_right) {
            Claw::rightGoLowerLimit();
            StateHandler = state_grabIdol;
        }
    }

    void state_grabIdol() {
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
        clawCounter = 0;
        StateHandler = state_raiseForDrop;
    }

    void state_raiseForDrop() {
        if (Arm::see_idol_left) {
            Claw::leftGoMiddle();
        }
        if (Arm::see_idol_right) {
            Claw::rightGoMiddle();
        }
        StateHandler = state_goToBin;
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
        if(following_tape) {
            StateHandler = state_tape_following;
            return;
        }
        StateHandler = state_infrared_tracking;
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