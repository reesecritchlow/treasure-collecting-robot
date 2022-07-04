#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TAPE_LEFT PA0	// analog read pin
#define TAPE_RIGHT PA1   // analog read pin 2
#define TUNE_P PA4
#define TUNE_I PA5
#define TINE_D PA6
#define INTERNAL_LED PB2

float reflectance_left = 0;
float reflectance_right = 0;
float reflectance_diff;
double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

void setupDisplay() {
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();
  delay(400);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Hello world!");
  display_handler.display();
}

void loopDisplay() {
  display_handler.print(reflectance_diff);
  if (reflectance_diff > 50) {
    display_handler.println("very right");
  } else if (reflectance_diff < -50) {
    display_handler.println("very left");
  } else if (reflectance_diff > 30) {
    display_handler.println("med right");
  } else if (reflectance_diff < -30) {
    display_handler.println("med left");
  } else if (reflectance_diff > 10) {
    display_handler.println("slight right");
  } else if (reflectance_diff < -10) {
    display_handler.println("slight left");
  } else {
    display_handler.println("stright (mostly)");
  }

  display_handler.print("left read: ");
  display_handler.println(reflectance_left);
  display_handler.print("right read: ");
  display_handler.println(reflectance_right);
  display_handler.print("PID: ");
  display_handler.println(myPID.GetKp());
  display_handler.println(" ");
  display_handler.println(myPID.GetKi());
  display_handler.println(" ");
  display_handler.println(myPID.GetKd());
}

void setup() {
  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(TAPE_LEFT, INPUT_ANALOG);
  pinMode(TAPE_RIGHT, INPUT_ANALOG);
  
  digitalWrite(INTERNAL_LED, HIGH);

  reflectance_left = analogRead(TAPE_LEFT);
  reflectance_right = analogRead(TAPE_RIGHT);

  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  }

void loop() {
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  reflectance_left = analogRead(TAPE_LEFT);
  reflectance_right = analogRead(TAPE_RIGHT);
  reflectance_diff = reflectance_left - reflectance_right;
  Input = reflectance_diff;
  
  loopDisplay();

  display_handler.display();
  myPID.Compute();
  

  delay(10);
  }