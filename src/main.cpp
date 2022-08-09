#include <Arduino.h>
#include "tape-navigator.h"
#include "infrared-navigator.h"
#include "encoder-navigator.h"
#include "config.h"
#include "display-manager.h"
#include "state-machine.h"
#include "arm.h"
#include "claw.h"


void setup() {
  pinMode(PB2, OUTPUT);
  // Tape::setupTapeTracking();
  // Infrared::setupInfrared();
  Display::setupDisplay();
  // Encoders::setupEncoders();

  // Drivetrain::startDrive();
  Display::display_handler.clearDisplay();
	Display::display_handler.setCursor(0, 0);
  // Claw::setupServos();
  
  // Display::display_handler.println("servos");
  // Display::display_handler.display();
  // Arm::setupArm();
  // Display::display_handler.clearDisplay();
  // Display::display_handler.println("arm");
  // Display::display_handler.display();
  Arm::setupSonars();
  // Display::display_handler.clearDisplay();
  // Display::display_handler.println("sonar");
  // Display::display_handler.display();
  // Claw::setupHallSensor(); 
  // Display::display_handler.clearDisplay();
  // Display::display_handler.println("magnet");
  // Display::display_handler.display();
  // Arm::setHome();
  // Display::display_handler.clearDisplay();
  // Display::display_handler.println("home");
  // Display::display_handler.display();
  // PID::newPIDSystem(TAPE_KP, TAPE_KI, TAPE_KD);
  // Claw::setupHallSensor();
}

void loop() {
  StateMachine::StateHandler();
  StateMachine::cycleCounter++;
}