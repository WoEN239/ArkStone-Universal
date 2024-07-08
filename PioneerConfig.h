#include <Arduino.h>
#include <Prizm_Controller.h>

const uint32_t WIRE_CLOCK = 100000;

/* Movement */
const double ROTATION_SPEED = 100.0;
const double MOVEMENT_SPEED = 100.0;
const double MAX_BATTERY_VOLTAGE = 14.5;

/* IMU */
#define IMU_SENSOR IMU_SENSOR_BNO055
const double IMU_REFRESH_PERIOD_MS = 10; //?
const float IMU_DEGREES_RESOLUTION = 16.0;

const uint8_t BARRIER_SERVO_OPEN = 68;
const uint8_t BARRIER_SERVO_CLOSE = 84;

/* DISTANCE SENSOR */
#define SONAR_SENSOR SONAR_SENSOR_PING
const uint8_t SONAR1_TRIG_PIN = 9;
const uint8_t SONAR1_ECHO_PIN = 2;
//#define DUAL_SONAR
const uint8_t SONAR2_TRIG_PIN = 5;
const uint8_t SONAR2_ECHO_PIN = 4;

const uint16_t SONAR_MAX_DISTANCE = 80;
const uint16_t SONAR_MIN_DISTANCE = 5;
const double SONAR_UPDATE_PERIOD_MS = 30.0;

/* Drivetrain */
PrizmDCExpansion driveExpansion(2);
const uint8_t DRIVETRAIN_MAX_MOTOR_POWER = 100;
const double DRIVETRAIN_TRACKWIDTH_MM = 370.0;
const uint8_t DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER = 1;
const uint8_t DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER = 2;
const double DRIVETRAIN_WHEEL_RADIUS_MM = 102.4;
const double DRIVETRAIN_MOTOR_GEARBOX_RATIO = 20.0;
const double DRIVETRAIN_MOTOR_ENCODER_RESOLUTION = 24.0;
const double DRIVETRAIN_MOTOR_RPM = 6000.0;
const double DRIVETRAIN_NOMINAL_VOLTAGE = 12.0;
const bool DRIVETRAIN_MOTOR_INVERSE = 1;

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

//const uint8_t LEDSTRIP_PORT_NUMBER = 2;

/* PID Parameters */
const float ANGLE_KP = 3.3;
const float ANGLE_KI = 0.007;
const float ANGLE_KD = 0.35;
const float ANGLE_MAX_I = 20.0;
const float ANGLE_MIN_SAMPLE_TIME = 0.02;

const float DISTANCE_KP = 6.5;
const float DISTANCE_KI = 0*0.5;
const float DISTANCE_KD = 0*0.4;
const float DISTANCE_MAX_I = 25.0;
const float DISTANCE_MIN_SAMPLE_TIME = 0.01;

/* Puck Sensor */
#define PUCK_SENSOR_TYPE PUCK_SENSOR_HITECHNIC_SOFTI2C
const uint8_t PUCK_SENSOR_HITECHNIC_SOFTI2C_SCL = A3;
const uint8_t PUCK_SENSOR_HITECHNIC_SOFTI2C_SDA = A2;

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

/* Separator */
#define SEPARATOR_INVERSE
const double SEPARATOR_UPDATE_PERIOD_MS = 10;
const double SEPARATOR_POWER_LIMIT = 40.0;
const double SEPARATOR_STEP_DEGREES = 120.0;
const double SEPARATOR_ENCODER_RESOLUTION = 720.0;
const double SEPARATOR_PUCK_DETECTION_TIME_THRESHOLD_MS = 500.0;
const double SEPARATOR_MOVEMENT_TIMEOUT_MS = 700.0;
const uint8_t SEPARATOR_MOTOR_PORT_NUMBER = 1;
const double SEPARATOR_POSITION_THRESHOLD = 30;

const double SEPARATOR_KP = 3.5;
const double SEPARATOR_KI = 0 * 0.3;
const double SEPARATOR_KD = 0.1;
const double SEPARATOR_MAXI = 15;
const double SEPARATOR_MIN_SAMPLE_TIME = 0.01;

/* Speaker */
//const PrizmDCExpansion speakerDcExpansion = Prizm.MotorController;
//const uint8_t SPEAKER_PORT_NUMBER = 2;
const PrizmDCExpansion speakerDcExpansion = driveExpansion;
#define SPEAKER_STEREO

/* Start button*/
const uint8_t START_BUTTON_PIN = 3;

/* LED Strip */
#define ARGB_LED
const size_t LED_NUM = 16;
const uint8_t LED_DATA_PIN = 4;

/*
const uint8_t PIONEER_UART_START_SEPARATOR_BLUE = 0x10;
const uint8_t PIONEER_UART_START_SEPARATOR_RED = 0x11;
const uint8_t PIONEER_UART_STOP_SEPARATOR = 0x12;
const uint8_t PIONEER_UART_PAUSE_SEPARATOR = 0x13;
const uint8_t PIONEER_UART_RESUME_SEPARATOR = 0x14;

const uint8_t PIONEER_UART_RX_PIN = 16;
const uint8_t PIONEER_UART_TX_PIN = 17;

const int32_t PIONEER_UART_SPEED = 19200;
*/