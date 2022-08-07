#include "arm.h"
#include "config.h"

namespace Arm {
    volatile int min_dist = SONAR_MAX_RANGE + 1;
    bool see_idol_right = false;
    bool see_idol_left = false;

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

        stepper.setCurrentPosition(0);
        return true;
    }

    /**
     * @brief moves stepper to 0 from its current position. Warning: This method needs to be placed in loop in order to run the stepper motor properly.
     *
     * @return
     */
    void goHome() {
        stepper.moveTo(0);
        stepper.run();
    }

    int currPos() {
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
        return((duration/2) / 29.1);
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
        if(left_distance > SONAR_MAX_RANGE) {
            left_distance = 0;
        }
        if(right_distance > SONAR_MAX_RANGE) {
            right_distance = 0;
        }


        if(right_distance < min_dist && right_distance > 0) {
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

    void sleep() {
        digitalWrite(SLP_PIN, LOW);
    }

    void dropOff() {

    }
}