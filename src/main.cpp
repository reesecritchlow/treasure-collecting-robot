#include <Arduino.h>
#include "tape-navigator.h"
#include "infrared-navigator.h"
#include "encoder-navigator.h"
#include "config.h"
#include "display-manager.h"
#include "state-machine.h"


void setup() {
  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(TUNER_TWO_PIN, INPUT_ANALOG);
  pinMode(TUNER_ONE_PIN, INPUT_ANALOG);
  Tape::setupTapeTracking();
  Infrared::setupInfrared();
  Display::setupDisplay();
  // Encoders::setupEncoders();

  Drivetrain::startDrive();

  // Claw::setupServos();
  // Arm::setupArm();
  // Arm::setupSonars();
  PID::newPIDSystem(500.0, 0, 65.0);
}

void loop() {
  StateMachine::StateHandler();
  StateMachine::cycleCounter++;
}