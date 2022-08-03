#ifndef PID_H
#define PID_H

namespace PID {
    extern unsigned long current_time;
    extern unsigned long flag_time;
    extern double elapsed_time;
    extern int error;
    extern int last_error;
    extern int last_state;
    extern double cumulative_error;
    extern double rate_error;
    extern double kp;
    extern double ki;
    extern double kd;

    double computePID(int input);
    void resetPID();
}



#endif