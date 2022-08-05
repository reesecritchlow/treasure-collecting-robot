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
  Encoders::setupEncoders();
  // Claw::setupServos();
  // Arm::setupArm();
  // Arm::setupSonars();
  double custom_kp = analogRead(TUNER_ONE_PIN);
  double secondary = analogRead(TUNER_TWO_PIN) / 1000.0;
  for (int i = 0; i < 500; i++) {
      custom_kp = analogRead(TUNER_ONE_PIN);
      secondary = analogRead(TUNER_TWO_PIN) / 1000.0;
      Display::displayTuners(i, custom_kp, secondary);
  }
  PID::newPIDSystem(custom_kp, TAPE_KI, TAPE_KD);
}

void loop() {
  StateMachine::StateHandler();
  StateMachine::cycleCounter++;
}