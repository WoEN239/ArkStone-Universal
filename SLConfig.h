#include <stdio.h>
#include <Arduino.h>
#include <Prizm_Controller.h>

const double MAX_BATTERY_VOLTAGE = 16.8;

#define IMU_SENSOR IMU_SENSOR_BNO08X
const double IMU_REFRESH_PERIOD_MS = 20.0;

const uint8_t BARRIER_SERVO_OPEN = 97;    //???  //45
const uint8_t BARRIER_SERVO_CLOSE = 110;  //??? //25

const uint8_t SONAR_SDA = A2;
const uint8_t SONAR_SCL = A3;
 
const uint16_t SONAR_MAX_DISTANCE = 350;
const uint16_t SONAR_MIN_DISTANCE = 21;
#define SONAR_DEADZONE
const uint16_t SONAR_DEADZONE_START = 31;
const uint32_t SONAR_DEADZONE_END = 55;
const double SONAR_UPDATE_PERIOD_MS = 500.0;

const PrizmDCExpansion driveExpansion(2);  //(3)
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

const double FIELD_SENSOR_UPDATE_PERIOD_MS = 101;
#define FIELD_SENSOR FIELD_SENSOR_HITECHNIC
//#define FIELD_SENSOR FIELD_SENSOR_TCS34725_SOFTI2C

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

const uint8_t INTAKE_PORT_NUMBER = 2;
const uint8_t INTAKE_POWER = 0;

//const uint8_t LEDSTRIP_PORT_NUMBER = 2;

const float ANGLE_KP = 2.3;
const float ANGLE_KI = 0.015;
const float ANGLE_KD = 0.185;
const float ANGLE_MAX_I = 20.0;
const float ANGLE_MIN_SAMPLE_TIME = 0.01;

const float DISTANCE_KP = 5.0;
const float DISTANCE_KI = 0 * 0.5;
const float DISTANCE_KD = 0.5;
const float DISTANCE_MAX_I = 25.0;
const float DISTANCE_MIN_SAMPLE_TIME = 0.01;

const double PUCK_SENSOR_UPDATE_PERIOD_MS = 50;

const float REDPUCK_R = 132.0;
const float REDPUCK_G = 73.0;
const float REDPUCK_B = 55.0;
const float BLUEPUCK_R = 28.0;
const float BLUEPUCK_G = 81.0;
const float BLUEPUCK_B = 148.0;
const float NOPUCK_R = 61.0;
const float NOPUCK_G = 102.0;
const float NOPUCK_B = 88.0;
const double PUCK_COLOR_DISTANCE_THRESHOLD = 85.0;

#define SEPARATOR_PROPELLER_DETECTION
const float SEPARATOR_PROPELLER_R = 38.32;
const float SEPARATOR_PROPELLER_G = 107.39;
const float SEPARATOR_PROPELLER_B = 112.60;
const float SEPARATOR_PROPELLER_COLOR_DISTANCE_THRESHOLD = 6.5;
#define SEPARATOR_PROPELLER_PARKING
const int SEPARATOR_PROPELLER_PARKING_SPEED = 4;
const int SEPARATOR_PROPELLER_PARKING_DELAY = 185;

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

#define PUCK_SENSOR_TYPE PUCK_SENSOR_TCS34725_SOFTI2C
const uint8_t PUCK_SENSOR_TCS34725_SOFTI2C_SCL = 9;
const uint8_t PUCK_SENSOR_TCS34725_SOFTI2C_SDA = 2;
const uint8_t PUCK_SENSOR_UART_TX = 3;

PrizmDCExpansion speakerDcExpansion = driveExpansion;
#define SPEAKER_STEREO

#define START_BUTTON_INVERSE
const uint8_t START_BUTTON_PIN = NOT_A_PIN;

const double ROTATION_SPEED = 100;//100.0;
const double MOVEMENT_SPEED = 100;//70.0;