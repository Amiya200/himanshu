#pragma once

/* ============================================================
 *  config.h  —  ESP32 Solar Panel Cleaner
 * ============================================================ */

/* ============================================================
 *  L298N MOTOR DRIVER PINS
 * ============================================================ */
#define PIN_ENA     25
#define PIN_ENB     26
#define PIN_IN1     27
#define PIN_IN2     14
#define PIN_IN3     33
#define PIN_IN4     13

/* ============================================================
 *  TAIL LIGHT
 * ============================================================ */
#define PIN_TAIL_LIGHT  4

/* ============================================================
 *  RELAY-CONTROLLED PERIPHERALS  (active-LOW modules)
 * ============================================================ */
#define PIN_RELAY_PUMP      18
#define PIN_RELAY_BLOWER    19

/* ============================================================
 *  IR OBSTACLE SENSORS
 *  FIX: was IR_ENABLED=0 — sensors were completely disabled.
 *  Enabled now.  GPIO 36/39 are INPUT-ONLY on ESP32 — fine for
 *  sensor inputs.
 * ============================================================ */
#define IR_ENABLED              1       /* WAS 0 — FIXED */

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

/* Vibration accident detection */
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
#define AP_PASSWORD         "12345678"
#define AP_CHANNEL          6
#define AP_MAX_CONN         4

/* ============================================================
 *  TELEGRAM
 * ============================================================ */
#define TELEGRAM_BOT_TOKEN  "7752156618:AAF38Dz2sef1tTc7Dqo7pkGcCSx0116_qis"
#define TELEGRAM_CHAT_ID    "7984471289"
#define TELEGRAM_API_URL    "https://api.telegram.org/bot"
#define CAR_LOCATION        "28deg28'34.5 N 77deg28'43.7 E"

#define TELEGRAM_TASK_STACK     6144
#define TELEGRAM_HTTP_TIMEOUT   10

/* ============================================================
 *  HTTP SERVER
 * ============================================================ */
#define HTTP_SERVER_PORT        80

/* ============================================================
 *  MOVEMENT TIMING
 * ============================================================ */
#define MS_PER_CM           15
#define MS_TURN_90          280
#define MS_STABILISE        120

/* ============================================================
 *  AUTO-CLEAN PATH CONSTANTS
 * ============================================================ */
#define DEFAULT_PANEL_WIDTH_CM   100
#define DEFAULT_PANEL_HEIGHT_CM  170
#define STRIP_WIDTH_CM           10
#define INTER_PANEL_BACKUP_CM    15

/* ============================================================
 *  WATCHDOG
 * ============================================================ */
#define WDT_TIMEOUT_S           30