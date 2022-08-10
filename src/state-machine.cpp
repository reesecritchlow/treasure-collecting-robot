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
    bool arch_mode = false;
    int idol_count = 0;
    volatile bool searching_for_idol = true; // Determines whether the current state should be searching for an idol or not with sonars

    void state_tape_following();
    void test_encoders();
    void state_armHome();
    void state_clawLoop();

    // Initial State
    void (*StateHandler)() = state_armHome;
    void (*QueuedState)();
    void (*LastMainState)() = state_tape_following;

    void state_infrared_tracking();
    void state_do_nothing();
    void state_moveToIdol();
    void state_grabIdol();
    void state_lowerArmForIdol();
    void state_goToBin();
    void state_raiseForDrop();
    void state_dropIdol();
    void state_armThruArch();
    void state_chicken_wire_drive_straight();
    void state_tape_homing();
    void state_infrared_homing();
    void state_armHomeSetup();
    void state_magneticField();
    void state_temp_drive_straight();
    void state_infrared_tracking_no_idol_search();

    void state_tape_following() {
        // Loop Operations
        
        Tape::runPIDCycle();

        if (cycleCounter % PRINT_LOOP_COUNT == 0) {
            Display::displayTapeMetrics();
        }

        // Conditional Exits

        Infrared::readRightSensor();
        Infrared::readLeftSensor();
        //Infrared Sensed
        if (Infrared::right_signal >= INFRARED_TRANSITION_RIGHT_THRESHOLD || Infrared::left_signal >= INFRARED_TRANSITION_LEFT_THRESHOLD) {
            Display::display_handler.clearDisplay();
            Display::display_handler.setCursor(0,0);
            Display::display_handler.print("Infrared Detected");
            Display::display_handler.display();
            arch_mode = false;
            searching_for_idol = false;
            Arm::min_dist = SONAR_MAX_RANGE + 1;
            Arm::left_sonar_on = false;
            Encoders::setStraightDestinationDistance(50.0);
            StateHandler = state_infrared_tracking_no_idol_search;
            return;
        }
        
        Arm::idol_position = Arm::senseForIdol();
        // Loss of Tape
        if (Tape::tapeLost) {
            Display::getDisplayReady();
            if (!chicken_wire_crossed) {
                Display::displayState();
                Drivetrain::killDrive();
                PID::newPIDSystem(ENCODER_KP, ENCODER_KI, ENCODER_KD);
                StateHandler = state_chicken_wire_drive_straight;
                return;
            }
            Drivetrain::haltEncoders();
            StateHandler = state_infrared_homing;
            StateHandler = state_do_nothing;
            return;
        }

        // Idol Sensed
        if (searching_for_idol && Arm::idol_position != 0) {
            Display::display_handler.clearDisplay();
            Display::display_handler.setCursor(0,0);
            Display::display_handler.print("Arm Position: ");
            Display::display_handler.print(Arm::idol_position);
            Display::display_handler.display();
            digitalWrite(PB2, LOW);
            Arm::pickup_count++;
            Drivetrain::haltFirstIdol();
            delay(2000);
            Arm::wake();
            Encoders::setStraightDestinationDistance(5.0);
            QueuedState = state_moveToIdol;
            StateHandler = state_temp_drive_straight;
            LastMainState = state_tape_following;
        }
    }

    void state_temp_drive_straight() {
        Arm::setupSecondScan();
        while (!Encoders::checkDestinationDistance()) {
            Encoders::encoderDriveStraight();
            Arm::secondScanLoop();
        }
        Drivetrain::haltEncoders();
        delay(1000);
        Arm::secondScanLoop();
        Arm::setSecondDistance();
        Display::getDisplayReady();
        Display::display_handler.print("Second Scan: ");
        Display::display_handler.println(Arm::idol_position);
        Display::display_handler.display();
        StateHandler = QueuedState;
    }

    void state_chicken_wire_drive_straight() {
        Encoders::setStraightDestinationDistance(CHICKEN_WIRE_DISTANCE);
        Drivetrain::startDrive();
        while (!Encoders::checkDestinationDistance()) {
            // swag
        }
        Display::displayEncoderMetrics();
        Drivetrain::haltEncoders();
        chicken_wire_crossed = true;
        StateHandler = state_tape_homing;
    }

    void state_tape_homing() {
        double search_angle = 30.0;
        Drivetrain::speed_multiplier = 1.0;
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
                if ((Tape::current_pid_multiplier == 0 ||
                    Tape::current_pid_multiplier == FIRST_TAPE_STATE ||
                    /*Tape::current_pid_multiplier == SECOND_TAPE_STATE ||*/
                    Tape::current_pid_multiplier == -1 * FIRST_TAPE_STATE /*||
                    Tape::current_pid_multiplier == -1 * SECOND_TAPE_STATE*/)
                    && !Tape::tapeLost) {
                    PID::newPIDSystem(TAPE_KP, TAPE_KI, TAPE_KD);
                    Tape::tapeLost = false;
                    Drivetrain::haltEncoders();
                    delay(1000);
                    if (Claw::magnetic_idol) {
                        Claw::magnetic_idol = false;
                    }
                    if (!arch_mode) {
                        Arm::min_dist = SONAR_MAX_RANGE + 1;
                        searching_for_idol = true;
                    }
                    StateHandler = state_tape_following;
                    break;
                }
            }
            search_direction = !search_direction;
            search_angle *= 1.5;
            Drivetrain::haltEncoders();
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

    void state_infrared_tracking_no_idol_search() {
        if (cycleCounter % 100 == 0) {
            Display::displayInfraredMetrics();
        }
        Infrared::runPIDCycle();
        if (Encoders::checkDestinationDistance()) {
            StateHandler = state_infrared_tracking;
            Arm::min_dist = SONAR_MAX_RANGE + 1;
        }
    }

    void state_infrared_tracking() {
        Arm::idol_position = Arm::senseForIdol();
        Infrared::runPIDCycle();
        if (Arm::idol_position != 0) {
            Display::display_handler.clearDisplay();
            Display::display_handler.setCursor(0,0);
            Display::display_handler.print("Arm Position: ");
            Display::display_handler.print(Arm::idol_position);
            Display::display_handler.display();
            digitalWrite(PB2, LOW);
            Arm::pickup_count++;
            Drivetrain::haltFirstIdol();
            delay(2000);
            Arm::wake();

            QueuedState = state_moveToIdol;
            StateHandler = state_moveToIdol;
            LastMainState = state_infrared_tracking;
        }
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
        Drivetrain::haltEncoders();
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
        Drivetrain::haltEncoders();
        digitalWrite(INTERNAL_LED, LOW);
        Encoders::setStraightDestinationDistance(10.0);
        StateHandler = state_drive_straight_again;
    }

    // ============== Sensed Idol States ===============
    void state_moveToIdol() {
        Arm::move_distance = Arm::idol_position + SONAR_OFFSET;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            delay(1000);
            StateHandler = state_lowerArmForIdol;
        }
    }

    void state_lowerArmForIdol() {
        Claw::open();
        if (Arm::see_idol_right) {
            Claw::leftGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        if (Arm::see_idol_left) {
            Claw::rightGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        if (Claw::magnetic_idol) {
            Display::display_handler.clearDisplay();
            Display::display_handler.println("magnet on lower");
            Display::display_handler.display();
            StateHandler = state_armHome;
            return;
        }
    }

    void state_grabIdol() {
        while(!(Claw::magnetic_idol) && (clawCounter <= SERVO_ANGLE_DIVISION)) {
            Claw::close(clawCounter);
            clawCounter += 1;
        }

        if (Claw::magnetic_idol) {
            Display::display_handler.clearDisplay();
            Display::display_handler.println("magnet on grab");
            Display::display_handler.display();
            StateHandler = state_armHome;
            return;
        }
        // } else {
        //     Display::display_handler.println("close");
        //     Claw::close(SERVO_ANGLE_DIVISION);
        // }
        Display::display_handler.clearDisplay();
        Display::display_handler.println("grab complete");
        Display::display_handler.display();
        clawCounter = 0;
        StateHandler = state_raiseForDrop;
    }

    void state_raiseForDrop() {
        Display::display_handler.clearDisplay();
        Display::display_handler.println("raise for drop");
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        Display::display_handler.display();   
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
        Claw::leftGoMiddle();
        Claw::rightGoMiddle();
        Claw::open();
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        StateHandler = state_armHome;
        searching_for_idol = false;
    }

    // =========== Arm movement states ============
    void state_armThruArch() {
        // Arm::move_distance = 0;
        // Arm::goTo();
        // if(Arm::getDistanceToGo() == 0) {
        delay(400);
        Claw::leftGoMiddle();
        Claw::rightGoMiddle();
        Claw::close(SERVO_ANGLE_DIVISION);
        arch_mode = true;
        Tape::tapeLost = true;
        StateHandler = state_tape_homing;
        Display::display_handler.clearDisplay();
        Display::display_handler.println("arms in position for arch");
        Display::display_handler.display();
        // }
    }

    void state_armHome() {   
        Arm::move_distance = 0;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            Display::display_handler.clearDisplay();
            Display::display_handler.println("arm home");
            Display::display_handler.display();
            delay(1000);
            Arm::min_dist = SONAR_MAX_RANGE + 1;  
            if (chicken_wire_crossed) {
                StateHandler = state_armThruArch;
                Display::display_handler.clearDisplay();
                Display::display_handler.println("arm thru arch state");
                Display::display_handler.display();
                return;
            }
            StateHandler = state_armHomeSetup;
        }
    }

    void state_armHomeSetup() {
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        Arm::see_idol_left = false;
        Arm::see_idol_right = false;
        PID::resetPID();
        StateHandler = LastMainState;
    }
}