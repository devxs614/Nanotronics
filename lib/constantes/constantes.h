#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <stdint.h>

#define DEBUG_ENABLED true
#define COMPETITION_MODE false
#define START_IMMEDIATELY true
#define ENABLE_DISPLAY true
#define ENABLE_ARUCO false
#define ENABLE_RETURN_BONUS false
#define ENABLE_VERBOSE_LOGGING false
#define ENABLE_SAFETY_LIMITS true

#define LOG_INFO(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_WARN(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_ERROR(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_DEBUG(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }

#define MOTOR_PWM_MIN 35
#define MOTOR_PWM_MAX 255
#define RPM_TARGET_MAX 250.0f
#define ACCELERATION_LIMIT 40.0f

#define WHEEL_DIAMETER_MM 70.0f
#define WHEEL_BASE_MM 170.0f
#define TRACK_WIDTH_MM 170.0f
#define ROBOT_WIDTH_MM 290.0f
#define ROBOT_LENGTH_MM 290.0f

#define ENCODER_TICKS_PER_OUTPUT_REV_M1 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M2 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M3 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M4 260.0f

#define KP_M1 0.35f
#define KI_M1 0.10f
#define KD_M1 0.02f
#define KP_M2 0.35f
#define KI_M2 0.10f
#define KD_M2 0.02f
#define KP_M3 0.35f
#define KI_M3 0.10f
#define KD_M3 0.02f
#define KP_M4 0.35f
#define KI_M4 0.10f
#define KD_M4 0.02f

#define KP_HEADING 1.0f
#define KI_HEADING 0.05f
#define KD_HEADING 0.12f
#define KP_POSITION 0.8f
#define KI_POSITION 0.04f
#define KD_POSITION 0.08f

#define FRONT_STOP_MM 150.0f
#define WALL_TARGET_MM 220.0f
#define TURN_CLEARANCE_MM 150.0f
#define MOVE_ONE_CELL_MM 300.0f

#define COLOR_CONFIRM_SAMPLES 3
#define COLOR_CLEAR_MIN 120
#define COLOR_SATURATION_MIN 80
#define COLOR_CONFIDENCE_MIN 0.55f

#define GRIPPER_OPEN_ANGLE 18
#define GRIPPER_CLOSE_ANGLE 122
#define GRIPPER_HOLD_ANGLE 90

#define START_MODE_START 0
#define START_MODE_CHECKPOINT_1 1
#define START_MODE_CHECKPOINT_2 2

#define OLED_ADDRESS 0x3C

constexpr uint8_t TCS34725_I2C_ADDRESS = 0x29;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;
constexpr uint8_t VL53_FRONT_ADDRESS = 0x30;
constexpr uint8_t VL53_RIGHT_ADDRESS = 0x31;
constexpr uint8_t VL53_LEFT_ADDRESS = 0x32;

#define MOTOR1_INVERTED false
#define MOTOR2_INVERTED false
#define MOTOR3_INVERTED false
#define MOTOR4_INVERTED false

#define ENCODER1_INVERTED false
#define ENCODER2_INVERTED false
#define ENCODER3_INVERTED false
#define ENCODER4_INVERTED false

// --- PINES CORREGIDOS ---
#define DISTANCE_FRONT_XSHUT 24
#define DISTANCE_RIGHT_XSHUT 22
#define DISTANCE_LEFT_XSHUT 26

#define TCS_POWER 28

#define MOTOR1_PWM_PIN 6
#define MOTOR1_AIN1 30
#define MOTOR1_AIN2 31
#define MOTOR2_PWM_PIN 7
#define MOTOR2_BIN1 32
#define MOTOR2_BIN2 33
#define MOTOR3_PWM_PIN 8
#define MOTOR3_AIN1 35
#define MOTOR3_AIN2 36
#define MOTOR4_PWM_PIN 9
#define MOTOR4_BIN1 37
#define MOTOR4_BIN2 38

#define ENCODER1_A 10
#define ENCODER1_B 11
#define ENCODER2_A 12
#define ENCODER2_B 13
#define ENCODER3_A 14
#define ENCODER3_B 15
#define ENCODER4_A 18
#define ENCODER4_B 19

#define GRIPPER_SERVO_PIN 5
#define QTR_SENSOR_PIN_BASE A0

namespace robot {
  enum class ActiveTrack {
    PISTA_A,
    PISTA_B
  };

  enum class StartMode {
    START_FROM_BEGINNING,
    START_FROM_CHECKPOINT_1,
    START_FROM_CHECKPOINT_2
  };

  enum class RobotMode {
    STARTUP,
    SELF_TEST,
    CALIBRATION,
    WAIT_FOR_START,
    PISTA_A,
    PISTA_B,
    RECOVERY,
    FINISHED,
    ERROR
  };

  enum class PistaAState {
    A_INIT,
    A_MAZE_NAVIGATION,
    A_COLOR_DETECTION,
    A_OBSTACLE_TRAVERSAL,
    A_CHECKPOINT,
    A_FINISH,
    A_RETURN_BONUS,
    A_ARUCO_OPTIONAL
  };

  enum class PistaBState {
    B_INIT,
    B_SECTION1,
    B_FIND_BALL,
    B_GRAB_BALL,
    B_TO_CHECKPOINT1,
    B_SECTION2,
    B_AVOID_LINES,
    B_TO_CHECKPOINT2,
    B_SECTION3,
    B_READ_TILE,
    B_MOVE_TO_NEXT_TILE,
    B_FINISH,
    B_ERROR
  };

  enum class BallState {
    UNKNOWN,
    NOT_CAPTURED,
    CAPTURING,
    CAPTURED,
    SECURED,
    LOST,
    BALL_UNCERTAIN
  };

  enum class ColorClass {
    COLOR_NONE,
    COLOR_CYAN,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_PINK,
    COLOR_UNKNOWN
  };

  enum class TrackDirection {
    TRACK_RIGHT,
    TRACK_LEFT,
    TRACK_UP,
    TRACK_DOWN
  };

  enum class SensorHealth {
    OK,
    NOT_FOUND,
    TIMEOUT,
    INVALID_DATA,
    CALIBRATION_REQUIRED
  };

  enum class RobotError {
    NONE,
    I2C_ERROR,
    IMU_ERROR,
    COLOR_ERROR,
    QTR_ERROR,
    VL53_FRONT_ERROR,
    VL53_LEFT_ERROR,
    VL53_RIGHT_ERROR,
    ENCODER_ERROR,
    MOTOR_ERROR,
    GRIPPER_ERROR,
    CALIBRATION_ERROR
  };

  struct ColorReading {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
    float rn;
    float gn;
    float bn;
    ColorClass color;
    float confidence;
  };

  struct LineObservation {
    bool detected;
    float normalizedPosition;
    uint8_t activeSensors;
    float confidence;
    bool leftBlocked;
    bool centerBlocked;
    bool rightBlocked;
  };
}

#endif