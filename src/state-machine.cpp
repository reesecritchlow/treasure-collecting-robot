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
    bool chicken_wire_crossed = false;
    bool search_direction = true;
    int idol_count = 0;

    void state_tape_following();
    void test_encoders();
    void state_armHome();
    void state_clawLoop();

    // Initial State
    void (*StateHandler)() = state_armHome;
    void (*LastMainState)() = state_tape_following;


    void state_infrared_tracking();
    void state_drive_straight();
    void state_do_nothing();
    void state_moveToIdol();
    void state_grabIdol();
    void state_lowerArmForIdol();
    void state_goToBin();
    void state_raiseForDrop();
    void state_dropIdol();
    void state_armThruArch();
    void state_search_for_infrared_at_arch();
    void state_spin_for_third_idol();
    void state_drive_to_third_idol();
    void state_chicken_wire_drive_straight();
    void state_tape_homing();
    void state_infrared_homing();
    void state_armHomeSetup();

    void state_tape_following() {
        // Loop Operations
        Arm::idol_position = Arm::senseForIdol();
        Tape::runPIDCycle();

        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayTapeMetrics();
        }

        // Conditional Exits

        // Loss of Tape
        if (Tape::tapeLost) {
            if (!chicken_wire_crossed) {
                Display::displayState();
                Drivetrain::killDrive();
                PID::newPIDSystem(ENCODER_KP, ENCODER_KI, ENCODER_KD);
                StateHandler = state_chicken_wire_drive_straight;
                return;
            }
            Drivetrain::halt();
            StateHandler = state_infrared_homing;
            return;
        }

        // Idol Sensed
        if(Arm::idol_position != 0) {
            Arm::wake();
            StateHandler = state_moveToIdol;
            LastMainState = state_tape_following;
            Drivetrain::halt();
        }

        // Infrared::readRightSensor();
        // Infrared Sensed
        // if (Infrared::right_signal >= INFRARED_TRANSITION_LEFT_THRESHOLD) {
        //     StateHandler = state_infrared_homing;
        // }

    }

    void state_chicken_wire_drive_straight() {
        Encoders::setStraightDestinationDistance(CHICKEN_WIRE_DISTANCE);
        while (!Encoders::checkDestinationDistance()) {
            Encoders::encoderDriveStraight();
            cycleCounter++; // TODO: remove these (RC)

            if (cycleCounter % PRINT_LOOP_COUNT) {
                Display::displayEncoderMetrics();
            }
        }
        Display::displayEncoderMetrics();
        Drivetrain::halt();
        chicken_wire_crossed = true;
        StateHandler = state_tape_homing;
    }

    void state_tape_homing() {
        double search_angle = 45.0;
        Drivetrain::speed_multiplier = 0.5;
        while (Tape::tapeLost) {
            delay(1000);
            Encoders::setSpinDestinationDistance(search_angle);
            while (!Encoders::checkDestinationDistance()) {
                cycleCounter++;
                Encoders::encoderSpin(search_direction);
                Tape::calculateTapePIDMultiplier();
                if (cycleCounter % PRINT_LOOP_COUNT) {
                    Display::displayEncoderMetrics();
                }
                if (Tape::current_pid_multiplier == 0 && !Tape::tapeLost) {
                    PID::newPIDSystem(TAPE_KP, TAPE_KI, TAPE_KD);
                    Tape::tapeLost = false;
                    Drivetrain::halt();
                    delay(1000);
                    StateHandler = state_tape_following;
                    break;
                }
            }
            search_direction = !search_direction;
            search_angle *= 2;
            Drivetrain::halt();
        }
        Drivetrain::speed_multiplier = 1.0;
        digitalWrite(INTERNAL_LED, HIGH);
    }

    void state_infrared_homing() {
        double search_angle = 45.0;
        bool infrared_lost = true;
        while (infrared_lost) {
            Encoders::setSpinDestinationDistance(search_angle);
            while (!Encoders::checkDestinationDistance()) {
                cycleCounter++;
                Encoders::encoderSpin(search_direction);
                Infrared::calculatePIDMultiplier();
                if (Infrared::current_pid_multiplier == 0) {
                    infrared_lost = false;
                    StateHandler = state_infrared_tracking;
                    break;
                }
            }
            search_direction = !search_direction;
            search_angle *= 2;
        }
    }

    void test_encoders() {
        Encoders::encoderDriveStraight();
        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayEncoderMetrics();
        }

        if (Encoders::left_count > Encoders::left_destination_count ||
            Encoders::right_count > Encoders::right_destination_count) {
            Drivetrain::halt();
            StateHandler = state_do_nothing;
        }
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


    // ============== Sensed Idol States ===============
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
        if (Arm::see_idol_left) {
            Claw::leftGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        if (Arm::see_idol_right) {
            Claw::rightGoLowerLimit();
            StateHandler = state_grabIdol;
        }
    }

    void state_grabIdol() {
        if (!Claw::seen_magnet) {
            while(!(Claw::magnetic_idol) && (clawCounter <= SERVO_ANGLE_DIVISION)) {
                Claw::close(clawCounter);
                clawCounter += 1;
            }
            if (Claw::magnetic_idol) {
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
        if (Arm::idol_position > 0) {
            Arm::move_distance = -BIN_DIST;
        } else if (Arm::idol_position < 0) {
            Arm::move_distance = BIN_DIST;
        }
        Arm::goTo();
        if (Arm::getDistanceToGo() == 0) {
            StateHandler = state_dropIdol;
        }
    }

    void state_dropIdol() {
        delay(1000);
        Claw::open();
        StateHandler = state_armHome;
    }

    // =========== Arm movement states ============
    void state_armThruArch() {
        Arm::goHome();
        Claw::leftGoLowerLimit();
        Claw::rightGoLowerLimit();
         
        Claw::close(SERVO_ANGLE_DIVISION);
    }

    void state_armHome() {
        // Arm::goHome();
        // if (Arm::getDistanceToGo() == 0) {
        //     StateHandler = LastMainState;   
        // }
        Arm::move_distance = 0;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            delay(1000);
            StateHandler = state_armHomeSetup;
        }
    }

    void state_armHomeSetup() {
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        Arm::see_idol_left = false;
        Arm::see_idol_right = false;
        StateHandler = LastMainState;
    }

    void state_magneticField() {

        //claw open fully

        //claw raise up

        //stepper go home
        Claw::seen_magnet = true;
        detachInterrupt(MAGNET_INTERRUPT_PIN);
        StateHandler = state_armHome;
    }

    void state_clawLoop() {
        Claw::leftGoUpperLimit();
        // delay(1000);
        Claw::leftGoLowerLimit();
    }
}