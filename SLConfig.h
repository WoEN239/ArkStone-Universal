#include <stdio.h>
#include <Arduino.h>
#include <Prizm_Controller.h>

const uint32_t WIRE_CLOCK = 400000;

/* Movement */
const double ROTATION_SPEED = 85.0;
const double MOVEMENT_SPEED = 75.0;
const double MAX_BATTERY_VOLTAGE = 16.8;

/* IMU */
//#define IMU_SENSOR IMU_SENSOR_BNO08X
const double IMU_REFRESH_PERIOD_MS = 5.0;

const uint8_t BARRIER_SERVO_OPEN = 87;
const uint8_t BARRIER_SERVO_CLOSE = 110;

/* DISTANCE SENSOR */
#define SONAR_SENSOR SONAR_SENSOR_MB1242_SOFTI2C
const uint8_t SONAR_SDA = A3;
const uint8_t SONAR_SCL = 5;
 
const uint16_t SONAR_MAX_DISTANCE = 350;
const uint16_t SONAR_MIN_DISTANCE = 21;
const double SONAR_UPDATE_PERIOD_MS = 500.0;

#define SONAR_DEADZONE
const uint16_t SONAR_DEADZONE_START = 31;
const uint32_t SONAR_DEADZONE_END = 55;

#define USE_ANALOG_WALL_SENSOR
const uint8_t ANALOG_WALL_SENSOR_LEFT = A1;
const uint8_t ANALOG_WALL_SENSOR_RIGHT = A2;
const uint16_t ANALOG_LEFT_WALL_SENSOR_THRESHOLD = 138;
const uint16_t ANALOG_RIGHT_WALL_SENSOR_THRESHOLD = 66;
const double ANALOG_WALL_SENSOR_TRIGGER_DISTANCE = 3.0;

/* Drivetrain */
PrizmDCExpansion driveExpansion(2);  //(3)
const uint8_t DRIVETRAIN_MAX_MOTOR_POWER = 100;
const double DRIVETRAIN_TRACKWIDTH_MM = 357.0;
const uint8_t DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER = 1;
const uint8_t DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER = 2;
const double DRIVETRAIN_WHEEL_RADIUS_MM = 102.4;
const double DRIVETRAIN_MOTOR_GEARBOX_RATIO = 13.7;
const double DRIVETRAIN_MOTOR_ENCODER_RESOLUTION = 24.0;
const double DRIVETRAIN_MOTOR_RPM = 6000.0;
const double DRIVETRAIN_NOMINAL_VOLTAGE = 12.0;
const bool DRIVETRAIN_MOTOR_INVERSE = 0;

//#define DRIVETRAIN_POWER_FILTERING
const double DRIVETRAIN_POWER_FILTER_T = 0.005;

/* Field Sensor */
const double FIELD_SENSOR_UPDATE_PERIOD_MS = 101;
#define FIELD_SENSOR FIELD_SENSOR_HITECHNIC

float REDFIELD_R = 0.98;
float REDFIELD_G = 0.18;
float REDFIELD_B = 0.13;
float BLUEFIELD_R = 0.15;
float BLUEFIELD_G = 0.31;
float BLUEFIELD_B = 0.94;
const float WHITEFIELD_R = 0.4;
const float WHITEFIELD_G = 0.64;
const float WHITEFIELD_B = 0.67;
const float FIELD_COLOR_DISTANCE_THRESHOLD = 0.3;
const float TEAM_FIELD_DETECTION_TIME_THRESHOLD_MS = 250;

/* Intake */
const uint8_t INTAKE_PORT_NUMBER = 2;
const uint8_t INTAKE_POWER = 100;

/* PID Parameters */
const float ANGLE_KP = 2.3;
const float ANGLE_KI = 0;//0.007;
const float ANGLE_KD = 0.185; //205
const float ANGLE_MAX_I = 20.0;
const float ANGLE_MIN_SAMPLE_TIME = 0.01;

const float DISTANCE_KP = 6.0;
const float DISTANCE_KI = 0 * 0.5;
const float DISTANCE_KD = 0*0.5;
const float DISTANCE_MAX_I = 10.0;
const float DISTANCE_MIN_SAMPLE_TIME = 0.01;

/* Puck Sensor */
#define PUCK_SENSOR_TYPE PUCK_SENSOR_TCS34725_SOFTI2C
const uint8_t PUCK_SENSOR_TCS34725_SOFTI2C_SCL = 9;
const uint8_t PUCK_SENSOR_TCS34725_SOFTI2C_SDA = 2;

const double PUCK_SENSOR_UPDATE_PERIOD_MS = 50;
const float REDPUCK_R = 155.0;
const float REDPUCK_G = 35.0;
const float REDPUCK_B = 14.0;
const float BLUEPUCK_R = 22.0;
const float BLUEPUCK_G = 86.0;
const float BLUEPUCK_B = 190.0;
const float NOPUCK_R = 107.0;
const float NOPUCK_G = 111.0;
const float NOPUCK_B = 104.0;
const double PUCK_COLOR_DISTANCE_THRESHOLD = 85.0;

#define SEPARATOR_PROPELLER_DETECTION
const float SEPARATOR_PROPELLER_R = 38.32;
const float SEPARATOR_PROPELLER_G = 107.39;
const float SEPARATOR_PROPELLER_B = 112.60;
const float SEPARATOR_PROPELLER_COLOR_DISTANCE_THRESHOLD = 6.5;
//#define SEPARATOR_PROPELLER_PARKING
const int SEPARATOR_PROPELLER_PARKING_SPEED = 12;
const int SEPARATOR_PROPELLER_PARKING_DELAY = 185;

/* Separator */
//#define SEPARATOR_INVERSE
const double SEPARATOR_UPDATE_PERIOD_MS = 10;
const int16_t SEPARATOR_POWER_LIMIT = 35.0;
const double SEPARATOR_STEP_DEGREES = 120.0;
const double SEPARATOR_ENCODER_RESOLUTION = 720.0;
const double SEPARATOR_PUCK_DETECTION_TIME_THRESHOLD_MS = 120.0;
const double SEPARATOR_MOVEMENT_TIMEOUT_MS = 300.0;
const uint8_t SEPARATOR_MOTOR_PORT_NUMBER = 1;
const double SEPARATOR_POSITION_THRESHOLD = 15;

const double SEPARATOR_KP = 3.5;
const double SEPARATOR_KI = 0 * 0.3;
const double SEPARATOR_KD = 0.1;
const double SEPARATOR_MAXI = 15;
const double SEPARATOR_MIN_SAMPLE_TIME = 0.01;

/* Speaker */
PrizmDCExpansion speakerDcExpansion = driveExpansion;
#define SPEAKER_STEREO

/* Start button */
//#define START_BUTTON_INVERSE
const uint8_t START_BUTTON_PIN = 3;

/* LED Strip */
#define ARGB_LED
const size_t LED_NUM = 17;
const uint8_t LED_DATA_PIN = 4;