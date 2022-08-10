#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Adafruit_SSD1306.h>

namespace Display {
    void displayTapeMetrics();
    void displayInfraredMetrics();
    void setupDisplay();
    void displayEncoderMetrics();
    void displayTuners(double counter, double tuner1, double tuner2);
    void displayState();
    void getDisplayReady();
    extern Adafruit_SSD1306 display_handler;
}



#endif