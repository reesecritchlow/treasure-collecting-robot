#include <Arduino.h>
#include "tape-navigator.h"
#include "infrared-navigator.h"
#include "encoder-navigator.h"
#include "config.h"
#include "display-manager.h"
#include "state-machine.h"


void setup() {
  pinMode(INTERNAL_LED, OUTPUT);
  Tape::setupTapeTracking();
  Infrared::setupInfrared();
  Display::setupDisplay();
  Encoders::setupEncoders();

  // Drivetrain::startDrive();

  // Claw::setupServos();
  // Arm::setupArm();
  // Arm::setupSonars();
  Encoders::setStraightDestinationDistance(5.0);
  PID::newPIDSystem(TAPE_KP, TAPE_KI, TAPE_KD);
}

void loop() {
  StateMachine::StateHandler();
  StateMachine::cycleCounter++;
}