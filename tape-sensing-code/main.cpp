#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TAPE_LEFT PA0  // analog read tape 1
#define TAPE_RIGHT PA1 // analog read tape 2
#define TAPE_MID PA2 // analog read tape 3
//#define IR_LEFT
//#define IR_RIGHT
//#define EDGE_LEFT
//#define EDGE_RIGHT
#define TUNE_P PA4
#define TUNE_I PA5
#define TUNE_D PA6
#define INTERNAL_LED PB2
#define ON 1
#define OFF 0

// constants for tape sensing and pwm
#define SPEED 1024
#define TAPE_THRESHOLD 550.0
#define CLK_FREQ 100
#define PWM_PIN PA_8
#define PWM_PIN2 PA_9

// variables for tape sensing
double reflectance_left = 0;
double reflectance_right = 0;
double reflectance_mid = 0;
int pid_in = 0, last_pid = 0;
int counter = 0;

// PID constants
double kp;
double ki;
double kd;

// variables for PID computation
unsigned long currentTime, flagTime;
double elapsedTime;
int error;
int lastError;
int lastState;
double input, output;
double cumError, rateError;
double clawMult = 1;
bool tapeSensing = true;

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

void loopDisplay() // to be changed and updated
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
  display_handler.print(output);
}

// determines the PID multiplier of which state we're currently in, 
// output is then fed into PID calculation method
int TapeSensingInput(double left_sensor, double right_sensor, double mid_sensor, double last)
{
  bool left = OFF, right = OFF, mid = OFF; // using each sensor, determine where we are on the tape
  if (left_sensor > TAPE_THRESHOLD)
  {
    left = ON;
  }
  if (right_sensor > TAPE_THRESHOLD)
  {
    right = ON;
  }
  if (mid_sensor > TAPE_THRESHOLD)
  {
    mid = ON;
  } // Now determine which state
  if (left && right && mid)
  {
    return 10; // switch to IR
  }
  else if (!left && !right && !mid && last == 0) 
  { 
    return 0; // chicken wire, go straight
  }
  else if (!left && !right && !mid && last > 0)
  {
    return 5;
  }
  else if (!left && !right && !mid && last < 0)
  {
    return -5;
  }
  else if (!left && !mid && right)
  {
    return -2;
  }
  else if (left && !mid && !right)
  {
    return 2;
  }
  else if (!left && mid && right)
  {
    return -1;
  }
  else if (left && mid && !right)
  {
    return 1;
  }
  return 0; // mid sees tape, go straight
}

// int IRSensingInput(double left_sensor, double right_sensor, double last)
// {
//   bool left = OFF, right = OFF;
//   if (left_sensor > TAPE_THRESHOLD)
//   {
//     left = ON;
//   }
//   if (right_sensor > TAPE_THRESHOLD)
//   {
//     right = ON;
//   }
//   if (left == OFF && right == OFF && last <= 0)
//   {
//     return -3;
//   }
//   else if (left == OFF && right == OFF && last > 0)
//   {
//     return 3;
//   }
//   else if (left == OFF && right == ON)
//   {
//     return -1;
//   }
//   else if (left == ON && right == OFF)
//   {
//     return 1;
//   }
//   return 0;
// }

void setup()
{

  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(TAPE_LEFT, INPUT_ANALOG);
  pinMode(TAPE_RIGHT, INPUT_ANALOG);
  pinMode(TUNE_P, INPUT_ANALOG);
  pinMode(TUNE_I, INPUT_ANALOG);
  pinMode(TUNE_D, INPUT_ANALOG);

  digitalWrite(INTERNAL_LED, HIGH);
  setupDisplay();

  reflectance_left = analogRead(TAPE_LEFT);
  reflectance_right = analogRead(TAPE_RIGHT);
  reflectance_mid = analogRead(TAPE_MID);

  pwm_start(PWM_PIN, CLK_FREQ, SPEED*clawMult, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(PWM_PIN2, CLK_FREQ, SPEED*clawMult, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop()
{

  if (counter % 100 == 0)
  { // tuning, to be removed in final code
    kp = analogRead(TUNE_P) / 2.0; //*** Now defined at the top, uncomment for retuning
    ki = analogRead(TUNE_I) / 2.0;
    kd = analogRead(TUNE_D) / 2.0;
    
    if (tapeSensing) { 

      // read where each tape sensor
      reflectance_left = analogRead(TAPE_LEFT);
      reflectance_right = analogRead(TAPE_RIGHT);
      reflectance_mid = analogRead(TAPE_MID);

      // save last pid multiplier and get current one
      last_pid = pid_in;
      pid_in = TapeSensingInput(reflectance_left, reflectance_right, reflectance_mid, last_pid);

      // translate multiplier into pwm signal to wheel
      output = computePID(pid_in);
      pwm_start(PWM_PIN, CLK_FREQ, (SPEED + output)*clawMult*1.2, RESOLUTION_12B_COMPARE_FORMAT); // 1.2 factor for slow wheel
      pwm_start(PWM_PIN2, CLK_FREQ, (SPEED - output)*clawMult, RESOLUTION_12B_COMPARE_FORMAT);

    } /* else { // IR sensing, needs work

      //reflectance_left = analogRead(IR_LEFT); 
      //reflectance_right = analogRead(IR_RIGHT);

      last_pid = pid_in;
      pid_in = IRSensingInput(reflectance_left, reflectance_right, last_pid);

      output = computePID(pid_in);
      pwm_start(PWM_PIN, CLK_FREQ, (SPEED + output)*clawMult, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(PWM_PIN2, CLK_FREQ, (SPEED - output)*clawMult, RESOLUTION_12B_COMPARE_FORMAT);
    } */

  
    loopDisplay();
    display_handler.display();
  }
  counter += 1;
}