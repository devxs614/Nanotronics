#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>

constexpr uint8_t TCA9548A_ADDR = 0x70;

inline void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;

  // El TCA9548A debe quedar sin canales antes de seleccionar el siguiente.
  // Evita que un dispositivo de un canal previo mantenga SDA/SCL ocupado.
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(static_cast<uint8_t>(0x00));
  Wire.endTransmission();
  delayMicroseconds(100);

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(static_cast<uint8_t>(1U << channel));
  if (Wire.endTransmission() != 0) {
    // Un NACK del MUX puede aparecer durante la recuperacion del bus; reintenta
    // una vez sin dejar un canal anterior conectado.
    delayMicroseconds(200);
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(static_cast<uint8_t>(1U << channel));
    Wire.endTransmission();
  }
  delayMicroseconds(200);
}

#define MUX_CH_IMU 0
#define MUX_CH_COLOR 1
#define MUX_CH_FRONT 2
#define MUX_CH_LEFT 4
#define MUX_CH_RIGHT 3
#define MUX_CH_OLED 5

// Odometría y Dimensiones de Celda
constexpr float CELL_MM = 280.0f;               // 28 cm reales por casilla

// Geometría y Umbrales de Pared
constexpr uint16_t UMBRAL_PARED_MM = 180;       // Pared si 1 <= d < 180 mm; 0 = libre
constexpr uint16_t SETPOINT_LATERAL_MM = 45;    // Margen de 4.5 cm centrado

// Constantes de Calibración IMU (MPU6050)
constexpr uint8_t IMU_MUX_CHANNEL = 0;
constexpr uint8_t MPU6050_I2C_ADDR = 0x68;
constexpr float GYRO_LSB_PER_DPS = 16.4f;       // Configuración +-2000 dps (0x1B = 0x18)
constexpr float IMU_SCALE_FACTOR = 9.4235f;     // Factor de ganancia empírico (360° test)
constexpr int16_t TURN_BASE_PWM = 40;           // PWM base para giro ortogonal preciso
constexpr int16_t TURN_SLOW_PWM = 22;           // PWM en los últimos 15°

// Avance recto calibrado por tests_2.0/15_test_straight_pid.cpp.
constexpr float KP_STRAIGHT = 4.0f;
constexpr float KD_STRAIGHT = 0.0f;
constexpr int16_t NAV_STRAIGHT_BASE_PWM = 100;

// Reajuste ortogonal y centrado suave antes del siguiente escaneo de celda.
constexpr uint16_t NAV_FRONT_ALIGN_MIN_MM = 30;
constexpr uint16_t NAV_FRONT_ALIGN_MAX_MM = 160;
constexpr uint16_t NAV_FRONT_ALIGN_TARGET_MM = 80;
constexpr uint16_t NAV_FRONT_ALIGN_FORWARD_MIN_MM = 85;
constexpr uint16_t NAV_FRONT_ALIGN_REVERSE_MAX_MM = 75;
constexpr int16_t NAV_FRONT_ALIGN_PWM = 35;
constexpr uint32_t NAV_FRONT_ALIGN_TIMEOUT_MS = 400UL;
constexpr uint32_t NAV_FRONT_ALIGN_SETTLE_MS = 50UL;
constexpr uint32_t NAV_180_INTER_TURN_REST_MS = 300UL;
constexpr uint32_t NAV_180_FINAL_SNAP_REST_MS = 150UL;

// Heuristicas de recuperacion: evitan repetir la misma decision o mantener
// motores activos cuando la odometria no demuestra avance fisico.
constexpr uint8_t NAV_ANTI_LOOP_ACTION_LIMIT = 4;
constexpr float NAV_STUCK_MIN_PROGRESS_MM = 20.0f;
constexpr uint32_t NAV_STUCK_PROGRESS_TIMEOUT_MS = 1200UL;
constexpr float NAV_STUCK_REVERSE_MM = 50.0f;
constexpr int16_t NAV_STUCK_REVERSE_PWM = 80;
constexpr uint32_t NAV_STUCK_REVERSE_TIMEOUT_MS = 1500UL;
constexpr uint32_t NAV_STUCK_SETTLE_MS = 200UL;


// Tiempos de navegacion: el reposo por casilla mantiene la telemetria activa.
constexpr uint32_t NAV_STARTUP_STABILIZE_MS = 4000UL;
constexpr uint32_t NAV_STARTUP_SPLASH_MS = 1500UL;
constexpr uint32_t NAV_CELL_REST_MS = 3000UL;
constexpr uint32_t NAV_REST_TELEMETRY_MS = 200UL;

#define DEBUG_ENABLED true
#ifndef COMPETITION_MODE
#define COMPETITION_MODE false
#endif
#define START_IMMEDIATELY true
#ifndef ENABLE_DISPLAY
#define ENABLE_DISPLAY true
#endif
#ifndef ENABLE_ARUCO
#define ENABLE_ARUCO false
#endif
#ifndef ENABLE_RETURN_BONUS
#define ENABLE_RETURN_BONUS false
#endif
#ifndef ENABLE_VERBOSE_LOGGING
#define ENABLE_VERBOSE_LOGGING false
#endif
#ifndef ENABLE_SAFETY_LIMITS
#define ENABLE_SAFETY_LIMITS true
#endif

#define LOG_INFO(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_WARN(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_ERROR(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }
#define LOG_DEBUG(msg) if (DEBUG_ENABLED) { Serial.println(F(msg)); }

#define MOTOR_DEADBAND_PWM 15
#define MOTOR_PWM_MIN 15
#define MOTOR_PWM_MAX 255
#define MOTOR4_TRIM_FACTOR 1.0f
#define MOTOR_UPDATE_PERIOD_MS 20UL
#define ENCODER_RPM_SAMPLE_MS 50UL
#define RPM_TARGET_MAX 250.0f
#define ACCELERATION_LIMIT 40.0f
#define FAST_SENSOR_UPDATE_MS 5UL
#define DISTANCE_SENSOR_UPDATE_MS 25UL
#define COLOR_SENSOR_UPDATE_MS 50UL
#define LINE_SENSOR_UPDATE_MS 10UL
#define DISPLAY_REFRESH_MS 150UL
#define DEBUG_REFRESH_MS 200UL
#define ARUCO_MARKER_TIMEOUT_MS 3000UL

#define WHEEL_DIAMETER_MM 70.0f
#define WHEEL_BASE_MM 170.0f
#define TRACK_WIDTH_MM 170.0f
#define ROBOT_WIDTH_MM 290.0f
#define ROBOT_LENGTH_MM 290.0f

#define ENCODER_TICKS_PER_OUTPUT_REV_M1 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M2 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M3 260.0f
#define ENCODER_TICKS_PER_OUTPUT_REV_M4 260.0f

constexpr float TICKS_PER_MM = 2.6442f;

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

#define KP_HEADING 0.015f
#define KI_HEADING 0.0f
#define KD_HEADING 0.002f
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
#define COLOR_CYAN_GREEN_MIN 0.30f
#define COLOR_CYAN_BLUE_MIN 0.28f
#define COLOR_CYAN_RED_MAX 0.34f
#define COLOR_YELLOW_RED_MIN 0.34f
#define COLOR_YELLOW_GREEN_MIN 0.34f
#define COLOR_YELLOW_BLUE_MAX 0.24f
#define COLOR_ORANGE_RED_MIN 0.48f
#define COLOR_ORANGE_GREEN_MIN 0.22f
#define COLOR_ORANGE_GREEN_MAX 0.42f
#define COLOR_ORANGE_BLUE_MAX 0.20f
#define COLOR_PINK_RED_MIN 0.38f
#define COLOR_PINK_BLUE_MIN 0.25f
#define COLOR_PINK_GREEN_MAX 0.28f
#define COLOR_RED_RED_MIN 0.48f
#define COLOR_RED_GREEN_MAX 0.34f
#define COLOR_RED_BLUE_MAX 0.22f
#define WHITE_LINE_ACTIVE_THRESHOLD 600

#define GRIPPER_OPEN_ANGLE 18
#define GRIPPER_CLOSE_ANGLE 122
#define GRIPPER_HOLD_ANGLE 90
#define BALL_SEARCH_SPEED_MM_S 45.0f
#define BALL_SEARCH_DETECTION_MM 240U
#define BALL_APPROACH_MM 110.0f
#define BALL_FAST_APPROACH_SPEED_MM_S 55.0f
#define BALL_SLOW_APPROACH_SPEED_MM_S 28.0f
#define BALL_CAPTURE_DISTANCE_MM 45.0f
#define BALL_OBJECT_RANGE_CHANGE_MM 15.0f
#define BALL_ALIGN_SWEEP_MS 400UL
#define BALL_ALIGN_SWEEP_SPEED_MM_S 35.0f
#define BALL_ALIGN_SETTLE_MS 100UL
#define BALL_GRAB_DISTANCE_MM 70.0f
#define BALL_GRIPPER_SETTLE_MS 600UL
#define BALL_VERIFY_TIMEOUT_MS 700UL
#define BALL_RECOVERY_TURN_MS 500UL
#define BALL_RECOVERY_DURATION_MS 1000UL
#define BALL_SECURE_SPEED_MM_S 60.0f
#define BALL_SECURE_TIMEOUT_MS 8000UL
#define LINE_AVOID_SPEED_MM_S 55.0f
#define NAVIGATION_SPEED_MM_S 100.0f
#define ODOMETRY_CELL_TOLERANCE_MM 25.0f
#define COLOR_DISPLAY_TIME_MS 3000UL
#define TILE_ARRIVAL_COOLDOWN_MS 500UL
#define MAZE_TURN_TOLERANCE_DEG 4.0f
#define MAZE_TURN_TIMEOUT_MS 2500UL
#define PISTA_B_SECTION2_CELLS 4.0f
#define TCS_POWER_SETTLE_MS 100UL
#define VL53_RESET_SETTLE_MS 50UL
#define VL53_START_SETTLE_MS 20UL
#define VL53_CONTINUOUS_PERIOD_MS 25U
#define LOP_LIMIT_PER_SECTION 4
#define LOP_NO_PROGRESS_TIMEOUT_MS 12000UL
#define LOP_MIN_PROGRESS_MM 15.0f
#define RECOVERY_EEPROM_BASE 0
#define CHECKPOINT_EEPROM_BASE 8
#define CALIBRATION_DURATION_MS 120000UL

#define START_MODE_START 0
#define START_MODE_CHECKPOINT_1 1
#define START_MODE_CHECKPOINT_2 2

#define OLED_ADDRESS 0x3C

constexpr uint8_t TCS34725_I2C_ADDRESS = 0x29;

// Colores de pista reconocidos por el clasificador RGB normalizado del TCS34725.
// Piso es neutro: se muestra, pero no modifica el checklist obligatorio.
enum TrackColor : uint8_t {
  COLOR_FLOOR,
  COLOR_CYAN,
  COLOR_GREEN,
  COLOR_YELLOW,
  COLOR_PINK,
  COLOR_ORANGE,
  COLOR_RED,
  COLOR_LIME
};

// Centroides RGB normalizados de la calibracion de pista. El clasificador 1-NN
// siempre asigna el perfil con menor distancia euclidiana cuando hay senal.
struct ColorCentroidRgb {
  TrackColor color;
  float rNorm;
  float gNorm;
  float bNorm;
};

constexpr ColorCentroidRgb COLOR_CENTROIDS[] = {
  {COLOR_CYAN,   0.312f, 0.375f, 0.312f},
  {COLOR_YELLOW, 0.458f, 0.333f, 0.208f},
  {COLOR_ORANGE, 0.579f, 0.263f, 0.158f},
  {COLOR_PINK,   0.636f, 0.182f, 0.182f},
  {COLOR_RED,    0.640f, 0.200f, 0.160f},
  {COLOR_GREEN,  0.364f, 0.364f, 0.273f},
  {COLOR_FLOOR,  0.447f, 0.340f, 0.213f},
  {COLOR_LIME,   0.421f, 0.368f, 0.211f}
};
constexpr uint8_t COLOR_CENTROID_COUNT =
    sizeof(COLOR_CENTROIDS) / sizeof(COLOR_CENTROIDS[0]);

constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;
constexpr uint8_t VL53_FRONT_ADDRESS = 0x30;
constexpr uint8_t VL53_RIGHT_ADDRESS = 0x31;
constexpr uint8_t VL53_LEFT_ADDRESS = 0x32;

// --- INVERSIÓN DE MOTORES Y ENCODERS (LADO DERECHO INVERTIDO) ---
#define MOTOR1_INVERTED false
#define MOTOR2_INVERTED true
#define MOTOR3_INVERTED false
#define MOTOR4_INVERTED true

#define ENCODER1_INVERTED false
#define ENCODER2_INVERTED true
#define ENCODER3_INVERTED false
#define ENCODER4_INVERTED true

// --- PINES CORREGIDOS ---
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

// Pines de encoders (Mega 2560: canales A en 2, 3, 18, 19 con interrupción por hardware)
#define ENC1_PIN_A 2
#define ENC1_PIN_B 11
#define ENC2_PIN_A 3
#define ENC2_PIN_B 13
#define ENC3_PIN_A 18
#define ENC3_PIN_B 15
#define ENC4_PIN_A 19
#define ENC4_PIN_B 17

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
    COLOR_RED,
    COLOR_UNKNOWN
  };

  enum class MazeState {
    MAZE_INIT,
    MAZE_FIND_PATH,
    MAZE_FORWARD,
    MAZE_DECIDE_TURN,
    MAZE_TURN_LEFT,
    MAZE_TURN_RIGHT,
    MAZE_TURN_BACK,
    MAZE_COLOR_CHECK,
    MAZE_FINISH,
    MAZE_RECOVERY,
    MAZE_RETURN
  };

  enum class BallHandlerState {
    BALL_SEARCH,
    BALL_APPROACH,
    BALL_ALIGN,
    BALL_GRAB,
    BALL_VERIFY,
    BALL_SECURE,
    BALL_LOST,
    BALL_RECOVER,
    BALL_RELEASE
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

  struct RobotPose {
    float xMm;
    float yMm;
    float headingDeg;
  };
}

#endif
