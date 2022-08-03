#include "drivetrain.h"
#include "tape-navigator.h"
#include "infrared-navigator.h"
#include "encoder-navigator.h"
#include "config.h"
#include "display-manager.h"

namespace StateMachine {

    int cycleCounter = 0;

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

    void state5_do_nothing() {
    
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

}