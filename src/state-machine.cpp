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
    // QueuedState: For some states, a state after is selected by the prior function to control determinacy. 
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

    /**
     * @brief Standard tape following state. Keeps the robot on a line with a difference reflectance from
     * its surroundings. Reads data from the tape reflectance sensors and uses a PID control 
     * loop to send an appropriate signal to the motors. 
     * 
     */
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
            Encoders::setStraightDestinationDistance(118.0);
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
            Encoders::setStraightDestinationDistance(118.0);
            StateHandler = state_infrared_tracking_no_idol_search;
            return;
        }

        // Idol Sensed
        if (searching_for_idol && Arm::idol_position != 0) {
            idol_count++;
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
            Encoders::setStraightDestinationDistance(IDOL_PICKUP_OFFSET);
            QueuedState = state_moveToIdol;
            StateHandler = state_temp_drive_straight;
            LastMainState = state_tape_following;
            if (chicken_wire_crossed) {
                Tape::third_tape_state = THIRD_TAPE_STATE + 2;
                Tape::second_tape_state = SECOND_TAPE_STATE + 1;
            }
        }
    }

    /**
     * @brief Drives the robot in a straight line until the distance(s) contained in the Encoders
     * namespace has been reached.
     * 
     */
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

    /**
     * @brief Special case of temp_drive_straight. Drives the robot over the chicken
     * wire in a straight line and sets chicken_wire_crossed. Sends the robot into
     * state_tape_homing when finished.      * 
     */
    void state_chicken_wire_drive_straight() {
        Encoders::setStraightDestinationDistance(CHICKEN_WIRE_DISTANCE);
        Drivetrain::startDrive();
        while (!Encoders::checkDestinationDistance()) {
            if ((Tape::current_pid_multiplier == 0 ||
                    Tape::current_pid_multiplier == FIRST_TAPE_STATE ||
                    Tape::current_pid_multiplier == -1 * FIRST_TAPE_STATE)
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
        Display::displayEncoderMetrics();
        Drivetrain::haltEncoders();
        chicken_wire_crossed = true;
        search_direction = true;
        StateHandler = state_tape_homing;
    }

    /**
     * @brief Rotates the robot in alternating CCW/CW rotations, with angle increasing by 2 each direction change.
     * Looks for the tape using the reflectance sensors exits when the reflectance sensors identify the tape again.     * 
     */
    void state_tape_homing() {
        double search_angle = 175.0;
        Drivetrain::speed_multiplier = 1.0;
        while (Tape::tapeLost) {
            delay(1000);
            Encoders::setSpinDestinationDistance(search_angle);
            if (search_direction) {
                pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);
                pwm_start(LEFT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED / 3, PWM_SIGNAL_RESOLUTION);
            } else {
                pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);
                pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED / 3, PWM_SIGNAL_RESOLUTION);
            }
            while (!Encoders::checkDestinationDistance()) {
                cycleCounter++;
                Tape::calculateTapeError();
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
            if (search_direction) {
                pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
                pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);
                delay(0.5 * BRAKING_TIME);
                pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            } else {
                pwm_start(RIGHT_BACKWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
                pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);
                delay(0.5 * BRAKING_TIME);
                pwm_start(RIGHT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            }
            search_direction = !search_direction;
            search_angle *= 1.5;
            Drivetrain::haltEncoders();
        }
        Drivetrain::speed_multiplier = 1.0;
        digitalWrite(INTERNAL_LED, HIGH);
        StateHandler = state_tape_following;
    }

    /**
     * @brief Finds the infrared signal if it has been lost. 
     * Rotates the robot in alternating CCW/CW direction, with angle 45 degrees initially,
     * increasing angle by 2 each rotation direction alternation. Once a signal is 
     * found by the infrared sensors, the state exits.
     * 
     */
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

    /**
     * @brief Follows the infrared signal based on PID, but disables the sonar
     * sensors such that other obstacles do not get recognized as idols.  
     */
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

    /**
     * @brief Follows the infrared signal and moves the robot in an adjustment sequence to 
     * pickup an idol if seen.  
     */
    void state_infrared_tracking() {
        Arm::idol_position = Arm::senseForIdol();
        Infrared::runPIDCycle();
        if (Arm::idol_position != 0) {
            idol_count++;
            Display::display_handler.clearDisplay();
            Display::display_handler.setCursor(0,0);
            Display::display_handler.print("Arm Position: ");
            Display::display_handler.print(Arm::idol_position);
            Display::display_handler.display();
            digitalWrite(PB2, LOW);
            Arm::pickup_count++;
            Drivetrain::haltFirstIdol();
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, DRIVETRAIN_BASE_SPEED, PWM_SIGNAL_RESOLUTION);
            delay(40);
            pwm_start(LEFT_FORWARD_MOTOR_PIN, PWM_CLOCK_FREQUENCY, 0, PWM_SIGNAL_RESOLUTION);
            delay(2000);
            Arm::wake();
            Encoders::setStraightDestinationDistance(IDOL_PICKUP_OFFSET + 2);
            QueuedState = state_moveToIdol;
            StateHandler = state_temp_drive_straight;
            LastMainState = state_infrared_tracking;
        }
    }

    /**
     * @brief Does nothing.
     * 
     */
    void state_do_nothing() {
    
    }

    // ============== Sensed Idol States ===============
    /**
     * @brief Moves out arm towards the idol.
     * 
     */
    void state_moveToIdol() {
        Arm::move_distance = Arm::idol_position + SONAR_OFFSET;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            delay(1000);
            StateHandler = state_lowerArmForIdol;
        }
    }

    /**
     * @brief Lowers the claw to pickup an idol.
     * 
     */
    void state_lowerArmForIdol() {
        Claw::open();
        if (Claw::searchForMagneticField()) {
            return;
        }
        if (Arm::see_idol_right) {
            Claw::leftGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        if (Arm::see_idol_left) {
            Claw::rightGoLowerLimit();
            StateHandler = state_grabIdol;
        }
        Claw::searchForMagneticField();
    }

    /**
     * @brief Executes a grab sequence for the idol. Aborts if the hall-effect sensor
     * senses a magnetic idol.
     * 
     */
    void state_grabIdol() {
        bool field = false;
        while(!(Claw::magnetic_idol) && (clawCounter <= SERVO_ANGLE_DIVISION)) {
            if (Claw::searchForMagneticField()) {
                return;
            }
            Claw::close(clawCounter);
            clawCounter += 1;
        }
        Display::display_handler.clearDisplay();
        Display::display_handler.println("grab complete");
        Display::display_handler.display();
        clawCounter = 0;
        StateHandler = state_raiseForDrop;
    }

    /**
     * @brief Raises the claws after the idol is grabbed.
     * 
     */
    void state_raiseForDrop() {
        Display::display_handler.clearDisplay();
        Display::display_handler.println("raise for drop");
        Claw::leftGoUpperLimit();
        Claw::rightGoUpperLimit();
        Display::display_handler.display();   
        StateHandler = state_goToBin;
    }

    /**
     * @brief Moves the robot arm to center it over top of the bin.
     * 
     */
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

    /**
     * @brief Releases the robot from the claw.
     * 
     */
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
    /**
     * @brief Closes the robot's claws and tilts them upwards to fit inside an arch.
     * 
     */
    void state_armThruArch() {
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
    }

    /**
     * @brief Moves the robot's arms into the "home" position.
     * 
     */
    void state_armHome() {   
        Arm::move_distance = 0;
        Arm::goTo();
        if(Arm::getDistanceToGo() == 0) {
            Display::display_handler.clearDisplay();
            Display::display_handler.println("arm home");
            Display::display_handler.display();
            delay(1000);
            Arm::min_dist = SONAR_MAX_RANGE + 1;  

            StateHandler = state_armHomeSetup;
        }
    }

    /**
     * @brief Sets up the robot's claws and arms before starting a course trial.
     * 
     */
    void state_armHomeSetup() {
        Claw::leftGoMiddle();
        Claw::rightGoMiddle();
        Claw::close(SERVO_ANGLE_DIVISION);
        Arm::see_idol_left = false;
        Arm::see_idol_right = false;
        PID::resetPID();
        StateHandler = LastMainState;
    }
}