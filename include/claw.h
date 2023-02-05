#ifndef CLAW_H
#define CLAW_H

#include <Servo.h>
#include <Arduino.h>

namespace Claw {
    extern volatile bool magnetic_idol;
    extern bool seen_magnet;

    /**
     * @brief Moves claws to start angles, opens grippers
     * 
     */
    void setupServos();

    /**
     * @brief assigns hall effect sensor pins
     * 
     */
    void setupHallSensor();

    /**
     * @brief moves left claw to lowest limit
     * 
     */
    void leftGoLowerLimit();

    /**
     * @brief moves left claw to upper limit
     * 
     */
    void leftGoUpperLimit();

    /**
     * @brief moves left claw to middle positoin
     * 
     */
    void leftGoMiddle();
    
    /**
     * @brief moves right claw to lower limit
     * 
     */
    void rightGoLowerLimit();

    /**
     * @brief moves right claw to upper limit
     * 
     */
    void rightGoUpperLimit();

    /**
     * @brief moves right claw to middle position
     * 
     */
    void rightGoMiddle();
    
    /**
     * @brief opens gripper
     * 
     */
    void open();

    /**
     * @brief closes claws to an angle
     * 
     * @param int, angle 
     */
    void close(int angle);

    /**
     * @brief routine for handelling a detected magnet
     * 
     */
    void handleMagneticField();

    /**
     * @brief search for a magnetic field by closing claw at increments
     * 
     * @return true, if magnet is seen
     * @return false, if not magnet is seen
     */
    bool searchForMagneticField();

    /**
     * @brief closes gripper continously to some angle
     * 
     * @param angle 
     */
    void closeNoMagnet(int angle);
}

#endif