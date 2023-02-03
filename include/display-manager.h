#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Adafruit_SSD1306.h>

namespace Display {
    /**
     * @brief Sets up the display for use.
     * 
     */
    void setupDisplay();

    /**
     * @brief Display metrics on current state of the reflectance sensors.
     * 
     */
    void displayTapeMetrics();

    /**
     * @brief Displays metrics on current state of the infrared sensors.
     * 
     */
    void displayInfraredMetrics();

    /**
     * @brief Displays metrics on the current state of the motor encoders.
     * 
     */
    void displayEncoderMetrics();

    /**
     * @brief Displays current tuning values
     * 
     * @param counter   Current value of a counter.
     * @param tuner1    Current value of a tuned value.
     * @param tuner2    Current value of a tuned value.
     */
    void displayTuners(double counter, double tuner1, double tuner2);

    /**
     * @brief Displays the idol position reading, and whether the tape is lost 
     * or not.
     * 
     */
    void displayState();

    /**
     * @brief Clears the display and sets the cursor position.
     * 
     */
    void getDisplayReady();

    extern Adafruit_SSD1306 display_handler; // Display handler for outside use.
}



#endif