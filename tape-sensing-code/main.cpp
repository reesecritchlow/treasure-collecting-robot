#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TAPE_LEFT PA0  // analog read pin
#define TAPE_RIGHT PA1 // analog read pin 2
#define TUNE_P PA4
#define TUNE_I PA5
#define TUNE_D PA6
#define INTERNAL_LED PB2
#define ON 1
#define OFF 0

#define SPEED 512
#define CLK_FREQ 100
#define PWM_PIN PA_8
#define PWM_PIN2 PA_9

float reflectance_left = 0;
float reflectance_right = 0;
double Setpoint = 0;
double Input, Output;
int pid_in = 0, last_pid = 0;
int counter = 0;

// PID constants
double kp;
double ki;
double kd;

unsigned long currentTime, flagTime;
double elapsedTime;
int error;
int lastError;
int lastState;

double input, output, setPoint;
double cumError, rateError;

void setupDisplay()
{
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();
  delay(400);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0, 0);
  display_handler.println("Hello world!");
  display_handler.display();
}

double computePID(int inp)
{
  currentTime = millis();                         // get current time
  elapsedTime = (double)(currentTime - flagTime); // compute time elapsed from previous computation

  error = inp;                                   // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastState) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError * 0 - kd * rateError; // PID output

  if (error != lastError)
  {
    lastState = lastError;
    flagTime = currentTime;
  }

  lastError = error; // remember current error

  return out; // have function return the PID output
}

void loopDisplay()
{

  display_handler.clearDisplay();
  display_handler.setCursor(0, 0);

  display_handler.print("L: ");
  display_handler.print(reflectance_left);
  display_handler.print(" R: ");
  display_handler.println(reflectance_right);
  display_handler.print("PID: ");
  display_handler.print(kp);
  display_handler.print(" ");
  display_handler.println(kd);
  display_handler.print(pid_in);
  display_handler.print(" ");
  display_handler.print(Output);
}

int PIDInput(double left_sensor, double right_sensor, double last)
{
  bool left = OFF, right = OFF;

  if (left_sensor > 200.0)
  {
    left = ON;
  }

  if (right_sensor > 200.0)
  {
    right = ON;
  }

  if (left == OFF && right == OFF && last <= 0)
  {
    return -3;
  }
  else if (left == OFF && right == OFF && last > 0)
  {
    return 3;
  }
  else if (left == OFF && right == ON)
  {
    return -1;
  }
  else if (left == ON && right == OFF)
  {
    return 1;
  }
  return 0;
}

void setup()
{

  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(TAPE_LEFT, INPUT_ANALOG);
  pinMode(TAPE_RIGHT, INPUT_ANALOG);
  pinMode(TUNE_P, INPUT_ANALOG);
  pinMode(TUNE_I, INPUT_ANALOG);
  pinMode(TUNE_D, INPUT_ANALOG);

  digitalWrite(INTERNAL_LED, LOW);
  setupDisplay();

  reflectance_left = analogRead(TAPE_LEFT);
  reflectance_right = analogRead(TAPE_RIGHT);

  Setpoint = 0;

  pwm_start(PWM_PIN, CLK_FREQ, SPEED, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(PWM_PIN2, CLK_FREQ, SPEED, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop()
{

  kp = analogRead(TUNE_P) / 10.0;
  ki = analogRead(TUNE_I) / 10.0;
  kd = analogRead(TUNE_D) / 10.0;

  reflectance_left = analogRead(TAPE_LEFT);
  reflectance_right = analogRead(TAPE_RIGHT);

  last_pid = pid_in;
  pid_in = PIDInput(reflectance_left, reflectance_right, last_pid);

  Output = computePID(pid_in);
  pwm_start(PWM_PIN, CLK_FREQ, SPEED + Output, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(PWM_PIN, CLK_FREQ, SPEED - Output, RESOLUTION_12B_COMPARE_FORMAT);

  if (counter % 100 == 0)
  {
    loopDisplay();
    display_handler.display();
  }
  counter += 1;
}