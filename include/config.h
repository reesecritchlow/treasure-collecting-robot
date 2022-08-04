#ifndef CONFIG_H
#define CONFIG_H

// Globals

#define DEFAULT_PID_STATE 0
#define INTERNAL_LED PB2

#define PRINT_LOOP_COUNT 100

// Display Manager

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

// Main Constants

#define TEST_DRIVE_DISTANCE 100.0

// PID Constants

#define INFRARED_KP 65.0
#define INFRARED_KI 65.0
#define INFRARED_KD 0.0

#define TAPE_KP 65.0
#define TAPE_KI 65.0
#define TAPE_KD 0.0

#define ENCODER_KP 65.0
#define ENCODER_KI 65.0
#define ENCODER_KD 0.0

// IR Constants

// (these are technically main constants but)

#define INFRARED_TRANSITION_RIGHT_THRESHOLD 75.0
#define INFRARED_TRANSITION_LEFT_THRESHOLD 75.0

#define ON 1
#define OFF 0

#define DEFAULT_RIGHT_SENSOR_PIN PA6
#define DEFAULT_LEFT_SENSOR_PIN PA7
#define DEFAULT_RIGHT_CAP_RESET_PIN PB12
#define DEFAULT_LEFT_CAP_RESET_PIN PB13

#define DEFAULT_RIGHT_INFRARED_THRESHOLD 100
#define DEFAULT_LEFT_INFRARED_THRESHOLD 100

#define CAP_DELAY 1000
#define READ_DELAY 1000

// Drivetrain Constants

#define DEFAULT_RIGHT_FORWARD_MOTOR_PIN PB_6
#define DEFAULT_RIGHT_BACKWARD_MOTOR_PIN PB_7
#define DEFAULT_LEFT_FORWARD_MOTOR_PIN PB_8
#define DEFAULT_LEFT_BACKWARD_MOTOR_PIN PB_9

#define DEFAULT_CLOCK_FREQUENCY 100
#define DEFAULT_RESOLUTION RESOLUTION_12B_COMPARE_FORMAT
#define DEFAULT_BASE_SPEED 512

#define DEFAULT_SPEED_MULTIPLIER 1.0

#define BRAKING_TIME 220

// Encoder Constants

#define RIGHT_ENCODER_PIN PA14
#define LEFT_ENCODER_PIN PB4

#define RIGHT_ENCODER_DIRECTION_PIN PA13
#define LEFT_ENCODER_DIRECTION_PIN PB3

#define RIGHT_ENCODER_FORWARDS_PHASE 1
#define LEFT_ENCODER_FORWARDS_PHASE 0

#define ENCODER_SCALING_VALUE 1.0

    // Straight Line Constants

#define STEPS_PER_ROTATION 1441.0

#define WHEEL_RADIUS 3.337

#define PI 3.14159265359

#define RADIANS_PER_STEP 2.0 * PI / STEPS_PER_ROTATION

#define DISTANCE_CM_PER_STEP 0.14825003500631917 // Hardcoded, because cpp loves you.

    // Spinning Constants

#define SPIN_RADIUS 10.845  // (Half the track width)

#define CLOCKWISE true
#define COUNTER_CLOCKWISE false

// PID Constants

#define DEFAULT_KP 100.0
#define DEFAULT_KI 80.0
#define DEFAULT_KD 65.0

// Tape Following Constants

#define RIGHT_REFLECTANCE_PIN PA2
#define MIDDLE_REFLECTANCE_PIN PA1
#define LEFT_REFLECTANCE_PIN PA0
#define DEFAULT_REFLECTANCE_THRESHOLD 100

// Arm 

#define LEFT -100
#define RIGHT 100

#define SWT_PIN PC15

#define MOTOR_INTERFACE 1
#define DIR_PIN PB9
#define STP_PIN PB8
#define SLP_PIN PB7

#define L_TRIG PA14
#define L_ECHO PA15
#define R_TRIG PA4
#define R_ECHO PA5

#define SONAR_MAX_RANGE 25
#define USTEP_RATIO 8
#define DIST_RATIO 3.704
#define MAX_SPD 40000
#define ACCEL 300

#define BIN_DIST 10
#define SONAR_OFFSET 5

// Claw

#define SERVO_PIN_GRAB PA10
#define SERVO_PIN_TILT_LEFT PA2
#define SERVO_PIN_TILT_RIGHT PA1

#define SERVO_CLOSE_ANGLE 80 // closed grabbed
#define SERVO_OPEN_ANGLE 0   // released open
#define LEFT_LOWER_ANGLE 85 // lowered
#define LEFT_UPPER_ANGLE 10  // raised
#define RIGHT_LOWER_ANGLE 35 // lowered
#define RIGHT_UPPER_ANGLE 125  // raised
#define SERVO_WAIT_TIME 1000
#define SERVO_ANGLE_DIVISION 8

#define MAGNET_INTERRUPT_PIN PC13

#endif