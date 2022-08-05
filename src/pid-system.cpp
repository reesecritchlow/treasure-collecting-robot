#include <Arduino.h>
#include "pid-system.h"
#include "config.h"

namespace PID {
    unsigned long current_time = millis();
    unsigned long flag_time = millis();
    double elapsed_time = 0.0;
    int error = 0;
    int last_error = 0;
    int last_state = 0;
    double cumulative_error = 0.0;
    double rate_error = 0.0;
    double kp = DEFAULT_KP;
    double ki = DEFAULT_KI;
    double kd = DEFAULT_KD;

    // computes pid value based on history
    double computePID(int input) {
        current_time = millis();
        elapsed_time = (double)(current_time - flag_time);

        error = input;
        
        if (error == 0) {
            cumulative_error = 0;
        }

        cumulative_error += error * elapsed_time;
        rate_error = (error - last_state) / elapsed_time;

        double out = kp * error - kd * rate_error;

        if (elapsed_time > 1000) {
            out += ki * cumulative_error;
        }

        if (error != last_error) {
            last_state = last_error;
            flag_time = current_time;
        }

        last_error = error;

        return out;
    }

    void resetPID() {
        current_time = millis();
        flag_time = millis();
        elapsed_time = 0.0;
        error = 0;
        last_state = 0;
        cumulative_error = 0.0;
        rate_error = 0.0;
    }

    void newPIDSystem(double _kp, double _ki, double _kd) {
        resetPID();
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
}

