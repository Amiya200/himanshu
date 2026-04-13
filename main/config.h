#pragma once

/* ============================================================
 *  config.h  —  ESP32 Solar Panel Cleaner
 *
 *  PRODUCTION CHECKLIST:
 *   1. Set STA_SSID / STA_PASSWORD for your network.
 *   2. Change AP_PASSWORD (minimum 8 chars).
 *   3. Replace TELEGRAM_BOT_TOKEN / TELEGRAM_CHAT_ID.
 *   4. Verify CAR_LOCATION GPS coordinates.
 *   5. Tune MS_PER_CM on your surface (measure 1 m run).
 *   6. Tune MS_TURN_90 (measure 90° turn in place).
 * ============================================================ */

/* ============================================================
 *  GUARD: catch accidental default credentials at compile time
 * ============================================================ */
/* Uncomment the line below before flashing to production:     */
/* #define PRODUCTION_BUILD */
#ifdef PRODUCTION_BUILD
  #if defined(STA_PASSWORD) && (sizeof(STA_PASSWORD) <= 2)
    #error "STA_PASSWORD is empty — set it before building"
  #endif
#endif

/* ============================================================
 *  L298N MOTOR DRIVER PINS
 *  All pins verified OUTPUT-capable on ESP32.
 *  GPIO 12 (boot-strapping) and 0, 2, 15 avoided.
 * ============================================================ */
#define PIN_ENA     25
#define PIN_ENB     26
#define PIN_IN1     27
#define PIN_IN2     14
#define PIN_IN3     33
#define PIN_IN4     13

/* ============================================================
 *  TAIL LIGHT
 *  GPIO 4: safe output, not boot-strapped.
 * ============================================================ */
#define PIN_TAIL_LIGHT  4

/* ============================================================
 *  RELAY-CONTROLLED PERIPHERALS  (active-LOW modules)
 *  GPIO 34/35 are INPUT-ONLY on ESP32 — fixed to 18/19.
 * ============================================================ */
#define PIN_RELAY_PUMP      18
#define PIN_RELAY_BLOWER    19

/* ============================================================
 *  IR OBSTACLE SENSORS
 *  Module output: LOW = obstacle, HIGH = clear.
 *  GPIO 36/39 are INPUT-ONLY — acceptable for sensors.
 * ============================================================ */
#define IR_ENABLED              1

#define PIN_IR_FRONT_LEFT       17
#define PIN_IR_FRONT_RIGHT      16
#define PIN_IR_BACK_LEFT        36   /* INPUT-ONLY: sensor OK */
#define PIN_IR_BACK_RIGHT       39   /* INPUT-ONLY: sensor OK */

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

/* ±250 dps, ±2 g full-scale (REG config 0x00 each) */
#define GYRO_SENSITIVITY        131.0f
#define ACCEL_SENSITIVITY       16384.0f

/* Vibration accident detection */
#define MPU_VIB_THRESHOLD_G     0.40f  /* deviation from 1 g */
#define MPU_VIB_DEBOUNCE        5      /* consecutive samples */
#define MPU_POLL_INTERVAL_MS    20     /* 50 Hz */

/* ============================================================
 *  WIFI
 * ============================================================ */
/* Station (upstream router) */
#define STA_SSID            "Mechanical"
#define STA_PASSWORD        "qqqqqqqq"
#define STA_MAX_RETRIES     10          /* increased from 5 for reliability */

/* Soft-AP (direct client access) */
#define AP_SSID             "SolarCleaner"
#define AP_PASSWORD         "12345678"  /* CHANGE before deployment */
#define AP_CHANNEL          6
#define AP_MAX_CONN         4

/* ============================================================
 *  TELEGRAM
 *  SECURITY NOTE: move token to NVS or encrypted partition
 *  before mass production; do NOT commit to source control.
 * ============================================================ */
#define TELEGRAM_BOT_TOKEN  "7752156618:AAF38Dz2sef1tTc7Dqo7pkGcCSx0116_qis"
#define TELEGRAM_CHAT_ID    "7984471289"
#define TELEGRAM_API_URL    "https://api.telegram.org/bot"
#define CAR_LOCATION        "28deg28'34.5 N 77deg28'43.7 E"

/* Telegram task stack / timeout */
#define TELEGRAM_TASK_STACK     6144
#define TELEGRAM_HTTP_TIMEOUT   10    /* seconds */

/* ============================================================
 *  HTTP SERVER
 * ============================================================ */
#define HTTP_SERVER_PORT        80

/* ============================================================
 *  MOVEMENT TIMING
 *  Calibration target: 1 second at full speed = 1/MS_PER_CM cm.
 *  Measure on your actual surface and adjust.
 * ============================================================ */
#define MS_PER_CM           15       /* ms to travel 1 cm      */
#define MS_TURN_90          280      /* ms for 90° pivot turn  */
#define MS_STABILISE        120      /* settle after stop       */

/* ============================================================
 *  AUTO-CLEAN PATH CONSTANTS
 * ============================================================ */
#define DEFAULT_PANEL_WIDTH_CM   100
#define DEFAULT_PANEL_HEIGHT_CM  170
#define STRIP_WIDTH_CM           10
#define INTER_PANEL_BACKUP_CM    15

/* ============================================================
 *  WATCHDOG
 *  Task watchdog timeout (seconds).  Set ≥ longest blocking op.
 *  Longest op: auto-clean blocking move ~ panel_h_cm × MS_PER_CM
 *  = 170 × 15 ms = 2.55 s  →  use 30 s safety margin.
 * ============================================================ */
#define WDT_TIMEOUT_S           30