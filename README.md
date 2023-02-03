# Team Mode Push's Robot Firmware

## Current State: Build Passing
## Overall Structure

There exist five main file types in this repo:
- Main (`main.cpp`): Main entry point for the code. Initializes all drivers/pins in `setup()` and runs the main `loop()` function.
- State Machine (`state-machine.cpp`/`state-machine.h`): Contains loop functions for each of the different states for the robot.
- Config (`config.h`): Contains all pins for the robot and other important constants.
- PID (`pid-system.h`/`pid-system.cpp`): Contains helper functions for implementing PID-based control.
- Drivers (most other files): Provides interfacing for each of the sensors/actuators on the robot.
## Branches and Environments

In this repository, there are two-ish main branches:

**Production:** Branch containing the final, _competition_ code, with the understanding that this would be the new code that the robot would run on in the final competition. No edits should be made to this branch directly.

**Personal Feature Branches:** Branches which team members work on bug fixes, new features, or experimental code for the robot. Each branch should be named by the following convention: `initials-type-description`.



