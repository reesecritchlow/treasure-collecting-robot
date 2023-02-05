/** @file */
#ifndef ARM_H
#define ARM_H
#include <stdlib.h>
#include <Arduino.h>
#include <AccelStepper.h>

namespace Arm {
    extern int move_distance;
    extern int idol_position;
    extern bool see_idol_right;
    extern bool see_idol_left;
    extern bool left_sonar_on;
    extern int pickup_count;
    extern int min_dist;

    /**
     * @brief initializes pins for stepper motor, sets stepper parameters.
     */
    void setupArm();

    /**
     * @brief initializes pins for sonar sensors.
     *
     */
    void setupSonars();

    /**
     * @brief performs homing routine with stepper motor in order to find a zero. 
     * 
     * @return true when arm has reached home
     */
    bool setHome();

    /**
     * @brief moves stepper to 0 from its current position. 
     * Warning: This method needs to be placed in loop in order to run the stepper motor properly.
     *
     */
    void goHome();

    /**
     * @brief move arm to distance in cm, moves stepper the appropriate amount of steps for this distance.
     */
    void goTo();

    /**
     * @brief Disables the sleep pin of the stepper motor.
     * 
     */
    void sleep();

    /**
     * @brief Enables the sleep pin of the stepper motor.
     * 
     */
    void wake();

    /**
     * @brief Get the current position of stepper motor
     * 
     * @return int 
     */
    int getCurrPos();

    /**
     * @brief Use sonar to sense for an idol.
     * This needs to be called inside a loop. Otherwise sonar
     * pings will not function properly.
     * Ideally, this method is called with some delay
     * such that we are not pinging the sonar every loop.
     *
     * @return int, distance in cm needed to move in order to
     * reach idol.
     */
    int senseForIdol();

    /**
     * @brief Get the steps remaining in the current movement.
     * 
     * @return int steps to go
     */
    int getDistanceToGo();

    /**
     * @brief Resets max/min values for a "secondary scan" for 
     * and idol. Zeroes a count for "greater" and "lesser" readings
     * (those that are either higher or lower than the original).
     * Caches the original scan distance.
     * 
     * A "secondary scan" involves the robot moving forwards in 
     * a straight line by a specified distance by the encoders, and 
     * takes readings from the sonar every loop, adjusting a floor/ceiling
     * value for the sonar reading. If the readings from the sonar
     * on the secondary scan are consistently higher/lower, then later 
     * functions will select either the ceiling or floor as the new 
     * distance to an idol, thus decreasing the amount of error in 
     * the sonar readings.
     * 
     */
    void setupSecondScan();

    /**
     * @brief Selects whether the robot should select the greater 
     * distance or the lesser distance for an idol pickup.
     * 
     */
    void setSecondDistance();

    /**
     * @brief Takes a sonar reading and increments the "greater" or 
     * "lesser" counts for a second scan based on the sonar reading.
     * Updates max/min values for a second scan. Changes the cached
     * original scan distance to the current scan distance for 
     * consecutive scans.
     * 
     */
    void secondScanLoop();


    /**
     * @brief get distance of object in CM from a sonar sensor.
     * 
     * @param trig_pin trigger pin of sonar sensor
     * @param echo_pin echo pin of sonar sensor
     * @return int distance traveled ultrasonic waves in CM. i.e. distance of object
     */
    int getDistance(uint8_t trig_pin, uint8_t echo_pin);
}

#endif