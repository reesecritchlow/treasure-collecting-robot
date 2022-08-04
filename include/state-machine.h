#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

namespace StateMachine {
    extern int cycleCounter;
    extern void (*StateHandler)();
    extern void state_magneticField();
}

#endif