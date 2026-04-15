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
#define ROVER_WIDTH_CM        58        /* cm, lateral (perpendicular to travel) */
#define ROVER_DEPTH_CM        51        /* cm, fore-aft */

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
#define AP_PASSWORD         "solar1234"
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



 #define OVERSHOOT_CM          5
#define HEADING_TOLERANCE_DEG 3.0f
#define HEADING_CORRECT_MAX_MS 4000
/* ============================================================
 *  AUTO-CLEAN PATH CONSTANTS
 * ============================================================ */
#define DEFAULT_PANEL_WIDTH_CM   100
#define DEFAULT_PANEL_HEIGHT_CM  170
#define STRIP_WIDTH_CM           10
#define INTER_PANEL_BACKUP_CM    15

#define LATERAL_SHIFT_CM      ROVER_WIDTH_CM   /* 58 cm */
/* ============================================================
 *  WATCHDOG
 * ============================================================ */
#define WDT_TIMEOUT_S           30

// /*
//  * config.h  —  ESP32 Solar Panel Cleaner  —  Central Configuration
//  *
//  * CHANGES:
//  *  - Accident detection / vibration alert system REMOVED
//  *  - Full auto-clean parameters added (boustrophedon pattern)
//  *  - Rover dimensions: 58 cm wide x 51 cm deep
//  *  - All timing derived from MPU-assisted movement
//  *  - backward motion only used for inter-panel lateral transitions
//  */

// #pragma once

// #include <stdint.h>

// /* ════════════════════════════════════════════════════════════
//    WiFi — Station (connects to home router)
// ════════════════════════════════════════════════════════════ */
// #define STA_SSID         "Amiy"
// #define STA_PASSWORD     "12345678"
// #define STA_MAX_RETRIES  5

// /* ════════════════════════════════════════════════════════════
//    WiFi — Access Point (always available at 192.168.4.1)
// ════════════════════════════════════════════════════════════ */
// #define AP_SSID          "SolarCleaner"
// #define AP_PASSWORD      "solar1234"
// #define AP_CHANNEL       6
// #define AP_MAX_CONN      4

// /* ════════════════════════════════════════════════════════════
//    HTTP Server
// ════════════════════════════════════════════════════════════ */
// #define HTTP_SERVER_PORT 80

// /* ════════════════════════════════════════════════════════════
//    Motor GPIO  (L298N)
// ════════════════════════════════════════════════════════════ */
// #define PIN_ENA   GPIO_NUM_25   /* Left  motor PWM / enable */
// #define PIN_ENB   GPIO_NUM_26   /* Right motor PWM / enable */
// #define PIN_IN1   GPIO_NUM_27   /* Left  motor dir A */
// #define PIN_IN2   GPIO_NUM_14   /* Left  motor dir B */
// #define PIN_IN3   GPIO_NUM_12   /* Right motor dir A */
// #define PIN_IN4   GPIO_NUM_13   /* Right motor dir B */

// /* ════════════════════════════════════════════════════════════
//    Peripheral GPIO
// ════════════════════════════════════════════════════════════ */
// #define PIN_TAIL_LIGHT    GPIO_NUM_2
// #define PIN_RELAY_PUMP    GPIO_NUM_32   /* Active-LOW relay */
// #define PIN_RELAY_BLOWER  GPIO_NUM_33   /* Active-LOW relay */

// /* ════════════════════════════════════════════════════════════
//    IR Obstacle Sensors  (digital, active-LOW)
// ════════════════════════════════════════════════════════════ */
// #define IR_ENABLED            1         /* 1 = active, 0 = disabled */
// #define PIN_IR_FRONT_LEFT     GPIO_NUM_34
// #define PIN_IR_FRONT_RIGHT    GPIO_NUM_35
// #define PIN_IR_BACK_LEFT      GPIO_NUM_36
// #define PIN_IR_BACK_RIGHT     GPIO_NUM_39
// #define IR_POLL_INTERVAL_MS   50        /* sensor polling period */
// #define IR_DEBOUNCE_COUNT     3         /* consecutive reads to confirm */

// /* ════════════════════════════════════════════════════════════
//    MPU-6050 / ICM-20600  I2C
// ════════════════════════════════════════════════════════════ */
// #define MPU_ENABLED           1
// #define MPU6050_ADDR          0x68      /* AD0=LOW */
// #define MPU_I2C_PORT          I2C_NUM_0
// #define PIN_MPU_SDA           GPIO_NUM_21
// #define PIN_MPU_SCL           GPIO_NUM_22
// #define MPU_I2C_FREQ_HZ       400000   /* 400 kHz Fast Mode */
// #define MPU_POLL_INTERVAL_MS  20        /* 50 Hz sensor loop */

// /* Sensitivity — MPU-6050 at default full-scale */
// #define ACCEL_SENSITIVITY     16384.0f  /* LSB/g  for +/-2g */
// #define GYRO_SENSITIVITY      131.0f    /* LSB/(deg/s) for +/-250 dps */

// /* ════════════════════════════════════════════════════════════
//    Rover Physical Dimensions
// ════════════════════════════════════════════════════════════ */
// /* Rover body: 58 cm wide (lateral axis) x 51 cm deep (forward axis) */
// #define ROVER_WIDTH_CM        58        /* cm, lateral (perpendicular to travel) */
// #define ROVER_DEPTH_CM        51        /* cm, fore-aft */

// /*
//  * Cleaning strip width = rover width.
//  * Each forward pass covers ROVER_WIDTH_CM of panel width.
//  * The rover sweeps the full panel height in one pass.
//  */
// #define CLEAN_STRIP_CM        ROVER_WIDTH_CM   /* 58 cm per strip */

// /* ════════════════════════════════════════════════════════════
//    Motion Timing  (ms per cm at full speed, empirically tuned)
//    Calibrate by measuring actual travel over a known distance.
// ════════════════════════════════════════════════════════════ */
// /*
//  * MS_PER_CM: how many milliseconds of motor-on time to travel 1 cm.
//  * Start with ~15 ms/cm and adjust for your surface / motor speed.
//  * On smooth glass at 7.4 V battery: typically 12–18 ms/cm.
//  */
// #define MS_PER_CM             15        /* ms per cm straight travel */

// /*
//  * MS_PER_DEG_TURN: how many ms to spin in place 1 degree.
//  * Measured with GYRO closed-loop, but also used as upper bound.
//  * Typically 10–15 ms/deg for L298N 4WD on smooth surface.
//  */
// #define MS_PER_DEG_TURN       12        /* ms per degree point-turn */

// /* Convenience macros */
// #define MS_TURN_90            (90  * MS_PER_DEG_TURN)   /* 1080 ms */
// #define MS_TURN_180           (180 * MS_PER_DEG_TURN)   /* 2160 ms */

// /* Short settle delay after each stop */
// #define MS_STABILISE          150       /* ms */

// /* ════════════════════════════════════════════════════════════
//    Auto-Clean Behaviour
// ════════════════════════════════════════════════════════════ */
// /*
//  * LATERAL_SHIFT_CM: distance the rover moves sideways between strips.
//  * Equal to rover width so adjacent strips abut exactly.
//  */
// #define LATERAL_SHIFT_CM      ROVER_WIDTH_CM   /* 58 cm */

// /*
//  * OVERSHOOT_CM: extra cm beyond panel edge to ensure full coverage
//  * and give room to turn without falling off the edge.
//  */
// #define OVERSHOOT_CM          5

// /*
//  * HEADING_TOLERANCE_DEG: how close the gyro heading must be to target
//  * before a correction move is considered "done".
//  */
// #define HEADING_TOLERANCE_DEG 3.0f

// /* Maximum time (ms) to spend on a single heading correction attempt */
// #define HEADING_CORRECT_MAX_MS 4000

// /* ════════════════════════════════════════════════════════════
//    Telegram  (accident alerts REMOVED — kept stub for build compat)
// ════════════════════════════════════════════════════════════ */
// #define BOT_TOKEN        ""   /* empty — Telegram not used */
// #define CHAT_ID          ""
// #define CAR_LOCATION     "Solar Array"

// /* ════════════════════════════════════════════════════════════
//    Misc
// ════════════════════════════════════════════════════════════ */
// #define STATUS_LED_GPIO   GPIO_NUM_2