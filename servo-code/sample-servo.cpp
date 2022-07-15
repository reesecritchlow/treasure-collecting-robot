#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#define INTERNAL_LED PB2
#define SERVO_OUT PA_10
// #define SONAR_READ PA7
#define CLK_FREQ 100

Servo myservo;  // create servo object to control a servo
volatile bool fwd = true;

// volatile int sonar;  // analog pin used to connect the sonar

void setup() {
  pinMode(INTERNAL_LED, OUTPUT);
  // pinMode(SONAR_READ, INPUT_ANALOG);
  
  digitalWrite(INTERNAL_LED, HIGH);
  myservo.attach(SERVO_OUT);  // attaches the servo
  // pwm_start(SERVO_OUT, CLK_FREQ, 200, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop() {
  // sonar = analogRead(SONAR_READ);            // reads the value of the potentiometer (value between 0 and 1023)
  // sonar = map(sonar, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  if (fwd) {
    for (int i = 0; i < 180; i++) {
      myservo.write(i);
      delay(5);
    }
    fwd = false;
  } else {
    for (int i = 0; i < 180; i++) {
      myservo.write(180 - i);
      delay(5);
    }
    fwd = true;
  }
  digitalWrite(INTERNAL_LED, HIGH);
  delay(50);
  digitalWrite(INTERNAL_LED, LOW);
  delay(50);
}