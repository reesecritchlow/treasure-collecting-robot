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
}

void loop() {
  StateMachine::StateHandler();
  StateMachine::cycleCounter++;
}