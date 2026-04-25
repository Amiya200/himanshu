#pragma once

/*
 * config.h — ESP32 Solar Panel Cleaner v3.0
 * 
 * UPGRADES:
 *  - Servo motor control for wiper/viper boundary management
 *  - Improved PWM-based motor speed control for equal torque
 *  - Better MPU6050 heading PID constants
 *  - Extended grid support (up to 20x20)
 *  - Separate speed profiles for cleaning vs repositioning
 */

#include <stdint.h>

/* ============================================================
 *  L298N MOTOR DRIVER PINS + PWM
 * ============================================================ */
#define PIN_ENA         25      /* Left  motor PWM enable  */
#define PIN_ENB         26      /* Right motor PWM enable  */
#define PIN_IN1         27      /* Left  motor dir A       */
#define PIN_IN2         14      /* Left  motor dir B       */
#define PIN_IN3         33      /* Right motor dir A       */
#define PIN_IN4         13      /* Right motor dir B       */

/* LEDC PWM for motor speed control */
#define MOTOR_PWM_FREQ_HZ       1000
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_8_BIT   /* 0–255 */
#define MOTOR_PWM_TIMER         LEDC_TIMER_0
#define MOTOR_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE
#define MOTOR_CHANNEL_LEFT      LEDC_CHANNEL_0
#define MOTOR_CHANNEL_RIGHT     LEDC_CHANNEL_1

/* Speed profiles (0–255) */
#define MOTOR_SPEED_CLEAN       200     /* forward cleaning sweep    */
#define MOTOR_SPEED_REPOSITION  180     /* lateral/inter-panel moves */
#define MOTOR_SPEED_TURN        170     /* point-turn speed          */
#define MOTOR_SPEED_CORRECTION  160     /* drift correction speed    */

/* Left/right trim to compensate hardware imbalance (±50) */
/* Positive = boost left channel, negative = boost right  */
#define MOTOR_TRIM_OFFSET       0       /* tune if rover drifts       */

/* ============================================================
 *  SERVO MOTOR — Wiper/Viper boundary control
 * ============================================================ */
#define SERVO_ENABLED           1

#define PIN_SERVO               23      /* PWM signal to servo        */
#define SERVO_PWM_FREQ_HZ       50      /* Standard 50 Hz servo       */
#define SERVO_PWM_TIMER         LEDC_TIMER_1
#define SERVO_CHANNEL           LEDC_CHANNEL_2
#define SERVO_PWM_RESOLUTION    LEDC_TIMER_14_BIT  /* 0–16383          */

/* Servo pulse widths in microseconds */
#define SERVO_PULSE_MIN_US      500     /* 0°   — fully retracted     */
#define SERVO_PULSE_MAX_US      2500    /* 180° — fully extended      */
#define SERVO_ANGLE_PARK        90      /* park/stowed position (°)   */
#define SERVO_ANGLE_DEPLOY      10      /* deployed/sweeping (°)      */
#define SERVO_ANGLE_BOUNDARY    170     /* boundary-lift position (°) */

/* Servo move timing */
#define SERVO_DEPLOY_DELAY_MS   400     /* wait after deploy          */
#define SERVO_RETRACT_DELAY_MS  300     /* wait after retract         */

/* ============================================================
 *  TAIL LIGHT
 * ============================================================ */
#define PIN_TAIL_LIGHT          18

/* ============================================================
 *  RELAY-CONTROLLED PERIPHERALS  (active-LOW modules)
 * ============================================================ */
#define PIN_RELAY_PUMP          4
#define PIN_RELAY_BLOWER        19

/* ============================================================
 *  ROVER PHYSICAL DIMENSIONS
 * ============================================================ */
#define ROVER_WIDTH_CM          58      /* lateral (perpendicular to travel) */
#define ROVER_DEPTH_CM          51      /* fore-aft                          */

/* ============================================================
 *  IR OBSTACLE SENSORS
 *  GPIO 36/39 are INPUT-ONLY on ESP32 — fine for sensor inputs.
 * ============================================================ */
#define IR_ENABLED              1

#define PIN_IR_FRONT_LEFT       17
#define PIN_IR_FRONT_RIGHT      16
#define PIN_IR_BACK_LEFT        36      /* INPUT-ONLY: sensor OK */
#define PIN_IR_BACK_RIGHT       39      /* INPUT-ONLY: sensor OK */

/* Debounce: obstacle must persist for N × IR_POLL_INTERVAL_MS */
#define IR_DEBOUNCE_COUNT       3
#define IR_POLL_INTERVAL_MS     50

/* ============================================================
 *  MPU-6050  (I2C, master)
 * ============================================================ */
#define MPU_ENABLED             1

#define PIN_MPU_SDA             21
#define PIN_MPU_SCL             22

#define MPU6050_ADDR            0x68
#define MPU_I2C_PORT            I2C_NUM_0
#define MPU_I2C_FREQ_HZ         400000

/* ±250 dps, ±2 g full-scale */
#define GYRO_SENSITIVITY        131.0f
#define ACCEL_SENSITIVITY       16384.0f

/* Heading PID for straight-line correction */
#define HEADING_KP              8.0f    /* proportional gain          */
#define HEADING_KI              0.02f   /* integral gain              */
#define HEADING_KD              1.5f    /* derivative gain            */
#define HEADING_DEADBAND_DEG    2.0f    /* no correction inside band  */
#define HEADING_OK_DEG          3.0f    /* "on target" threshold      */

/* Vibration detection */
#define MPU_VIB_THRESHOLD_G     0.40f
#define MPU_VIB_DEBOUNCE        5
#define MPU_POLL_INTERVAL_MS    20      /* 50 Hz */

/* ============================================================
 *  WIFI
 * ============================================================ */
#define STA_SSID            "Mechanical"
#define STA_PASSWORD        "qqqqqqqq"
#define STA_MAX_RETRIES     10

#define AP_SSID             "SolarCleaner"
#define AP_PASSWORD         "solar1234"
#define AP_CHANNEL          6
#define AP_MAX_CONN         4

/* ============================================================
 *  HTTP SERVER
 * ============================================================ */
#define HTTP_SERVER_PORT        80

/* ============================================================
 *  MOVEMENT TIMING (ms per cm, empirically tuned)
 *  Calibrate: measure actual distance for 1000 ms drive,
 *  then set MS_PER_CM = 1000 / measured_cm
 * ============================================================ */
#define MS_PER_CM               15      /* at MOTOR_SPEED_CLEAN      */
#define MS_TURN_90              700     /* 90° point turn            */
#define MS_STABILISE            200     /* settle after stop         */

/* Heading correction */
#define OVERSHOOT_CM            3
#define HEADING_TOLERANCE_DEG   3.0f
#define HEADING_CORRECT_MAX_MS  5000

/* ============================================================
 *  AUTO-CLEAN PATH CONSTANTS
 * ============================================================ */
#define DEFAULT_PANEL_WIDTH_CM   200
#define DEFAULT_PANEL_HEIGHT_CM  170
#define STRIP_WIDTH_CM           ROVER_WIDTH_CM
#define INTER_PANEL_BACKUP_CM    15

#define LATERAL_SHIFT_CM        ROVER_WIDTH_CM   /* 58 cm */

/* ============================================================
 *  AUTO-CLEAN TUNING
 * ============================================================ */
#define TURN_90_MS              700     /* timed pre-spin for 90°    */
#define SETTLE_MS               300     /* post-stop gyro settle     */
#define CORRECT_MAX_MS          4000    /* max heading correction    */
#define TURN_FRACTION           0.70f   /* fraction of TURN_90_MS   */
#define POST_STEP_SETTLE_MS     600
#define POST_STRIP_PAUSE_MS     400

/* ============================================================
 *  WATCHDOG
 * ============================================================ */
#define WDT_TIMEOUT_S           30