#include "arm.h"
#include "config.h"
#include "display-manager.h"

namespace Arm {
    int min_dist = SONAR_MAX_RANGE + 1;
    bool see_idol_right = false;
    bool see_idol_left = false;
    bool left_sonar_on = false;
    int pickup_count = 0;


    int move_distance = 0;
    int idol_position = 0;

    AccelStepper stepper(MOTOR_INTERFACE, STP_PIN, DIR_PIN);

    void setupArm() {
        pinMode(SLP_PIN, OUTPUT);
        digitalWrite(SLP_PIN, HIGH);
        pinMode(SWT_PIN, INPUT_PULLUP);
        stepper.setMaxSpeed(MAX_SPD);
        stepper.setAcceleration(ACCEL);
    }

    /**
     * @brief move arm to distance in cm, moves stepper the appropriate amount of steps for this distance
     */
    void goTo() {
        if(stepper.distanceToGo() == 0) {
            stepper.moveTo((int)move_distance*DIST_RATIO*10);
        }
        stepper.run();
    }

    /**
     * @brief gets the stepper to perform a homing routine
     *
     *
     *
     * @code{}
     * @return true
     * @return false
     */
    bool setHome() {
        bool switch_initial = digitalRead(SWT_PIN);
        int direction;

        // initialize direction towards center
        if (switch_initial) {
            direction = LEFT;
        } else {
            direction = RIGHT;
        }

        do {
            stepper.move(direction);
            stepper.run();
            // digitalWrite(PB2, HIGH);
        } while (switch_initial == digitalRead(SWT_PIN));

        if (digitalRead(SWT_PIN)) { // if we have pressed it
            setHome();
        }

        stepper.setCurrentPosition(STEP_HOME_OFFSET);
        return true;
    }

    /**
     * @brief moves stepper to 0 from its current position. Warning: This method needs to be placed in loop in order to run the stepper motor properly.
     *
     * @return
     */
    void goHome() {
        if(stepper.distanceToGo() == 0) {
            stepper.moveTo(0);
        }
        stepper.run();
    }

    int getCurrPos() {
        return stepper.currentPosition();
    }

    int getDistanceToGo() {
        return stepper.distanceToGo();
    }


    int getDistance(uint8_t trig_pin, uint8_t echo_pin) {
        int duration;
        digitalWrite(trig_pin, LOW);  // Added this line
        delayMicroseconds(2); // Added this line
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10); // Added this line
        digitalWrite(trig_pin, LOW);
        duration = pulseIn(echo_pin, HIGH, 10000UL);
        return((int) ( (double) duration / 2 ) / 29.1);
    }

    /**
     * @brief Use sonar to sense for an idol.
     * This needs to be called inside a loop. Otherwise sonar
     * pings will not function properly.
     * \n
     * Ideally, this method is called with some delay
     * such that we are not pinging the sonar every loop.
     *
     * @return int, distance in cm needed to move in order to
     * reach idol.
     */
    int senseForIdol() {
        int left_distance;
        int right_distance;
        left_distance = getDistance(L_TRIG_PIN, L_ECHO_PIN);
        right_distance = getDistance(R_TRIG_PIN, R_ECHO_PIN);

        //establishes max range for sonar
        if (left_distance > SONAR_MAX_RANGE) {
            left_distance = 0;
        }
        if (right_distance > SONAR_MAX_RANGE) {
            right_distance = 0;
        }


        if (right_distance < min_dist && right_distance > 0 && left_sonar_on) {
            min_dist = right_distance;
            see_idol_right = true;
        } else if(left_distance < min_dist && left_distance > 0) {
            min_dist = left_distance;
            see_idol_left = true;
        }

        if(min_dist <= SONAR_MAX_RANGE && see_idol_right) {
            return -min_dist;
        } else if(min_dist <= SONAR_MAX_RANGE && see_idol_left) {
            return min_dist;
        } else {
            return 0;
        }
    }

    /**
     * @brief setup to initialize sonar on arm
     *
     */
    void setupSonars() {
        pinMode(L_TRIG_PIN, OUTPUT);
        pinMode(L_ECHO_PIN, INPUT);
        pinMode(R_TRIG_PIN, OUTPUT);
        pinMode(R_ECHO_PIN, INPUT);
    }

    void wake() {
        digitalWrite(SLP_PIN, HIGH);
    }

    void sleep() {
        digitalWrite(SLP_PIN, LOW);
    }

    int min_second_dist;
    int max_second_dist;
    int prior_distance;
    int greater_count;
    int lesser_count;

    void setupSecondScan() {
        max_second_dist = SECOND_SCAN_VARIANCE;
        min_second_dist = SECOND_SCAN_VARIANCE + SONAR_MAX_RANGE;
        prior_distance = idol_position;
        greater_count = 0;
        lesser_count = 0;
    }

    void secondScanLoop() {
        int currentDistance = getDistance(L_TRIG_PIN, L_ECHO_PIN);
        if (prior_distance > currentDistance) {
            
            if (currentDistance > max_second_dist && currentDistance <= SONAR_MAX_RANGE + SECOND_SCAN_VARIANCE && currentDistance <= idol_position + SECOND_SCAN_TOLERANCE) {
                greater_count++;
                max_second_dist = currentDistance;
            }
        }
        if (currentDistance < prior_distance) {
            
            if (currentDistance < min_second_dist && currentDistance >= SECOND_SCAN_VARIANCE) {
                lesser_count++;
                min_second_dist = currentDistance;
            }
        }
        prior_distance = currentDistance;
    }

    void setSecondDistance() {
        Display::getDisplayReady();
        Display::display_handler.print("Max Second: ");
        Display::display_handler.println(max_second_dist);
        Display::display_handler.print("Min Second: ");
        Display::display_handler.println(min_second_dist);
        Display::display_handler.print("Greater Count: ");
        Display::display_handler.println(greater_count);
        Display::display_handler.print("Lesser Count: ");
        Display::display_handler.print(lesser_count);
        Display::display_handler.display();
        delay(2000);
        int filtered_max = idol_position;
        int filtered_min = idol_position;

        filtered_max = max_second_dist;

        filtered_min = min_second_dist;

        if (greater_count > lesser_count) {
            idol_position = filtered_max;
            return;
        } else {
            idol_position = filtered_min;
            return;
        }
    }
}