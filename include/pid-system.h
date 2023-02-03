#ifndef PID_H
#define PID_H

/**
 * @brief PID Namespace:
 * 
 * Namespace for PID related functions.
 * 
 */
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

    /**
     * @brief Computes a PID multiplier based on an error input.
     * 
     * @param input The error in the actual vs target value for a given parameter. This error
     * should the error that is for the current PID system.
     * @return double   A PID multiplier value for the current error value.
     */
    double computePID(int input);

    /**
     * @brief Sets up the PID system with new kp, ki, kd values. Resets all stored values to zero.
     * 
     * @param _kp 
     * @param _ki 
     * @param _kd 
     */
    void newPIDSystem(double _kp, double _ki, double _kd);

    /**
     * @brief Resets the PID system. Preserves kp, ki, kd, and resets all other stored values to zero.
     * 
     */
    void resetPID();
}



#endif