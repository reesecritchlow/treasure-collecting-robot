#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

namespace Display {
    void displayTapeMetrics();
    void displayInfraredMetrics();
    void setupDisplay();
    void displayEncoderMetrics();
    void displayTuners(double counter, double tuner1, double tuner2);
}



#endif