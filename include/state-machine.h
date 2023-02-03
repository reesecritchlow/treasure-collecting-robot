#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

namespace StateMachine {
    extern int cycleCounter; // Number of cycles that state machine has executed.
    extern bool search_direction; // Direction to search for tape/infrared in.

    /**
     * @brief State machine function pointer. Points the the current state of the 
     * state machine. 
     * 
     */
    extern void (*StateHandler)();

    /**
     * @brief Sets up the robot's claws and arms before starting a course trial.
     * 
     */
    extern void state_armHome();
    extern volatile bool searching_for_idol; // Boolean for whether the robot is currently searching for idols or not.
    extern int idol_count; // Number of idols collected.
}

#endif