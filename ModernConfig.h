#include <Arduino.h>
#include <Prizm_Controller.h>

const uint8_t BARRIER_SERVO_OPEN = 45;
const uint8_t BARRIER_SERVO_CLOSE = 25;

const uint16_t SONAR_MAX_DISTANCE = 80;
const double SONAR_UPDATE_PERIOD_MS = 30.0;

const PrizmDCExpansion driveExpansion(3);  //(3)
const uint8_t DRIVETRAIN_MAX_MOTOR_POWER = 100;
const double DRIVETRAIN_TRACKWIDTH_MM = 350.0;
const uint8_t DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER = 1;
const uint8_t DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER = 2;
const double DRIVETRAIN_WHEEL_RADIUS_MM = 98.0;
const double DRIVETRAIN_MOTOR_GEARBOX_RATIO = 20.0;
const double DRIVETRAIN_MOTOR_ENCODER_RESOLUTION = 24.0;
const double DRIVETRAIN_MOTOR_RPM = 6000.0;
const double DRIVETRAIN_NOMINAL_VOLTAGE = 12.0;
const bool DRIVETRAIN_MOTOR_INVERSE = 0;

//#define FIELD_SENSOR FIELD_SENSOR_HITECHNIC
#define FIELD_SENSOR FIELD_SENSOR_TCS34725_SOFTI2C

const uint8_t FIELD_SENSOR_TCS34725_SOFTI2C_SCL = 2;
const uint8_t FIELD_SENSOR_TCS34725_SOFTI2C_SDA = 9;

float REDFIELD_R = 114.0;
float REDFIELD_G = 76.0;
float REDFIELD_B = 69.0;
float BLUEFIELD_R = 31.0;
float BLUEFIELD_G = 97.0;
float BLUEFIELD_B = 127.0;
const float WHITEFIELD_R = 61.0;
const float WHITEFIELD_G = 104.0;
const float WHITEFIELD_B = 89.0;
const float FIELD_COLOR_DISTANCE_THRESHOLD = 90.0;
const float TEAM_FIELD_DETECTION_TIME_THRESHOLD_MS = 250;

const int IMU_REFRESH_PERIOD_MS = 10;
const float IMU_DEGREES_RESOLUTION = 100.0;

const uint8_t INTAKE_PORT_NUMBER = 2;
const uint8_t INTAKE_POWER = 100;

//const uint8_t LEDSTRIP_PORT_NUMBER = 2;

const float ANGLE_KP = 3.7;
const float ANGLE_KI = 0.1;
const float ANGLE_KD = 0.25;
const float ANGLE_MAX_I = 20.0;
const float ANGLE_MIN_SAMPLE_TIME = 0.01;

const float DISTANCE_KP = 4.0;
const float DISTANCE_KI = 0 * 0.5;
const float DISTANCE_KD = 0.05;
const float DISTANCE_MAX_I = 25.0;
const float DISTANCE_MIN_SAMPLE_TIME = 0.01;

const float REDPUCK_R = 175.0;
const float REDPUCK_G = 44.0;
const float REDPUCK_B = 41.0;
const float BLUEPUCK_R = 46.0;
const float BLUEPUCK_G = 81.0;
const float BLUEPUCK_B = 124.0;
const float NOPUCK_R = 80.0;
const float NOPUCK_G = 87.0;
const float NOPUCK_B = 71.0;
const double PUCK_COLOR_DISTANCE_THRESHOLD = 90.0;

//#define SEPARATOR_INVERSE
const double SEPARATOR_POWER_LIMIT = 75.0;
const double SEPARATOR_STEP_DEGREES = 120.0;
const double SEPARATOR_ENCODER_RESOLUTION = 720.0;
const double SEPARATOR_PUCK_DETECTION_TIME_THRESHOLD_MS = 200.0;
const double SEPARATOR_MOVEMENT_TIMEOUT_MS = 400.0;
const uint8_t SEPARATOR_MOTOR_PORT_NUMBER = 1;
const double SEPARATOR_POSITION_THRESHOLD = 1.9;

const double SEPARATOR_KP = 3.5;
const double SEPARATOR_KI = 0 * 0.3;
const double SEPARATOR_KD = 0.1;
const double SEPARATOR_MAXI = 15;
const double SEPARATOR_MIN_SAMPLE_TIME = 0.01;

#define SEPARATOR_SENSOR SEPARATOR_SENSOR_UART_RGB

const uint8_t SEPARATOR_SENSOR_UART_TX = 3;

const PrizmDCExpansion speakerDcExpansion = driveExpansion;
const uint8_t SPEAKER_PORT_NUMBER = 2;
#define SPEAKER_STEREO

const uint8_t START_BUTTON_PIN = A1;

const double ROTATION_SPEED = 85.0;
const double MOVEMENT_SPEED = 85.0;