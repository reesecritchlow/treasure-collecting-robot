#ifndef CONFIG_H
#define CONFIG_H

// Pins

#define INTERNAL_LED PB2

#define INFRARED_RIGHT_CAP_RESET_PIN PB12   // Right infrared capacitor reset pin (digital output)
#define INFRARED_LEFT_CAP_RESET_PIN PB13    // Left infrared capacitor reset pin (digital output)

// Stepper Block
#define DIR_PIN PA8                        // (5V digital)
#define STP_PIN PB15                        // (5V digital)
#define SLP_PIN PB14                         // (5V PWM)

// Sonar Block
#define R_ECHO_PIN PA9
#define R_TRIG_PIN PA10
#define L_ECHO_PIN PA11
#define L_TRIG_PIN PA12

#define RIGHT_ENCODER_PIN PB3              // Right encoder (counter) pin (5V digital interrupt input)
#define LEFT_ENCODER_PIN PA15                // Left encoder (counter) pin (5V digital interrupt input)
// PB4 Empty
// PB5 Empty
          

#define RIGHT_FORWARD_MOTOR_PIN PB_8        // Right forwards motor pin (PWM output)
#define RIGHT_BACKWARD_MOTOR_PIN PB_9       // Right backward motor pin (PWM output)
#define LEFT_FORWARD_MOTOR_PIN PB_6         // Left forwards motor pin (PWM output)
#define LEFT_BACKWARD_MOTOR_PIN PB_7        // Left forwards motor pin (PWM output)

// PB11 is temporarily OLED (I2C)
// PB12 is temporarily OLED (I2C)
#define TUNER_TWO_PIN PB0
#define TUNER_ONE_PIN PB1

#define INFRARED_LEFT_SENSOR_PIN PA7        // Left infrared sensor pin (3v3 analog input)
#define INFRARED_RIGHT_SENSOR_PIN PA6       // Right infrared sensor pin (3v3 analog input)

#define TAPE_RIGHT_SENSOR_PIN PA5           // Right tape reflectance sensor pin (5V analog input)
#define TAPE_MIDDLE_SENSOR_PIN PA4          // Middle tape reflectance sensor pin (5V analog input)
#define TAPE_LEFT_SENSOR_PIN PA3            // Left tape reflectance sensor pin (5V analog input)

// Servo Block
#define SERVO_PIN_TILT_LEFT PA_2 
#define SERVO_PIN_GRAB PA_1         
#define SERVO_PIN_TILT_RIGHT PA_0

#define MAGNET_INTERRUPT_PIN PC14
#define SWT_PIN PC15

// Globals

#define DEFAULT_PID_STATE 0
#define PRINT_LOOP_COUNT 100

// State Machine

#define INFRARED_ARCH_ALIGNMENT_THRESHOLD 100
#define CHICKEN_WIRE_DISTANCE 30.0 // (centimeters)


// Display Manager

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

// Main Constants

#define TEST_DRIVE_DISTANCE 100.0

// PID Constants

#define DEFAULT_KP 65.0
#define DEFAULT_KI 65.0
#define DEFAULT_KD 65.0

#define INFRARED_KP 65.0
#define INFRARED_KI 65.0
#define INFRARED_KD 0.0

#define TAPE_KP 500.0
#define TAPE_KI 0.0
#define TAPE_KD 150.0

#define ENCODER_KP 750.0
#define ENCODER_KI 65.0
#define ENCODER_KD 200.0

// IR Constants

// (these are technically main constants but)

#define INFRARED_TRANSITION_RIGHT_THRESHOLD 75.0
#define INFRARED_TRANSITION_LEFT_THRESHOLD 75.0

#define ON 1
#define OFF 0

#define DEFAULT_RIGHT_INFRARED_THRESHOLD 100
#define DEFAULT_LEFT_INFRARED_THRESHOLD 100

#define CAP_DELAY 1000
#define READ_DELAY 1000

// Drivetrain Constants

#define HALT_SCALING_VALUE 5.0

#define PWM_CLOCK_FREQUENCY 100
#define PWM_SIGNAL_RESOLUTION RESOLUTION_12B_COMPARE_FORMAT
#define DRIVETRAIN_BASE_SPEED 1024

#define DRIVETRAIN_SPEED_MULTIPLIER 1.0

#define BRAKING_TIME 220

// Encoder Constants

#define ENCODER_SCALING_VALUE 0.5

    // Straight Line Constants

    #define STEPS_PER_ROTATION 1441.0
    #define PI 3.14159265359
    #define DISTANCE_CM_PER_STEP 0.143 // Hardcoded, because cpp loves you.

    // Spinning Constants

    #define SPIN_RADIUS 10.82  // (Half the track width)

    #define CLOCKWISE true
    #define COUNTER_CLOCKWISE false

// Tape Following Constants

#define TAPE_REFLECTANCE_THRESHOLD 60

#define FIRST_TAPE_STATE 1
#define SECOND_TAPE_STATE 2
#define THIRD_TAPE_STATE 8

#define CHICKEN_WIRE_THRESHOLD 250

// Arm 

#define LEFT -100
#define RIGHT 100

#define STEP_HOME_OFFSET -200

#define MOTOR_INTERFACE 1

#define SONAR_MAX_RANGE 20
#define USTEP_RATIO 8
#define DIST_RATIO 3.704
#define MAX_SPD 40000
#define ACCEL 300

#define BIN_DIST 10
#define SONAR_OFFSET 7

// Claw


#define CUSTOM_SERVO_MINIMUM_RANGE 36
#define CUSTOM_SERVO_MAXIMUM_RANGE 256
#define CUSTOM_SERVO_READ_FREQUENCY 100
#define CUSTOM_SERVO_RESOLUTION RESOLUTION_10B_COMPARE_FORMAT

#define LIBRARY_SERVO_MINIMUM_RANGE 0
#define LIBRARY_SERVO_MAXIMUM_RANGE 180

#define SERVO_CLOSE_ANGLE 65 // closed grabbed
#define SERVO_OPEN_ANGLE 0   // released open
#define LEFT_LOWER_ANGLE 95 // lowered
#define LEFT_UPPER_ANGLE 10  // raised
#define LEFT_DUTY_DROP 73
#define RIGHT_LOWER_ANGLE 55 // lowered
#define RIGHT_UPPER_ANGLE 140  // raised
#define RIGHT_DUTY_DROP 183
#define SERVO_WAIT_TIME 1000
#define SERVO_ANGLE_DIVISION 8

#endif