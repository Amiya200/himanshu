#pragma once

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// --- L298N Motor Driver ---
#define PIN_ENA     18
#define PIN_ENB     19
#define PIN_IN1     23
#define PIN_IN2     22
#define PIN_IN3     21
#define PIN_IN4     5

// --- Tail Light ---
#define PIN_TAIL_LIGHT  2

// --- Relay-controlled peripherals (active-LOW modules) ---
#define PIN_RELAY_PUMP      12
#define PIN_RELAY_BLOWER    13

// --- Ultrasonic Sensors ---
// Set ULTRASONIC_ENABLED 0 when no sensors connected.
// Set to 1 and update pin numbers when you wire them up.
#define ULTRASONIC_ENABLED  0

// These pins must always be defined (compiler needs them even when
// ULTRASONIC_ENABLED=0). They are ignored at runtime when disabled.
#define PIN_TRIG_FRONT  15   // placeholder — update when sensor wired
#define PIN_ECHO_FRONT  4    // placeholder — update when sensor wired
#define PIN_TRIG_BACK   16   // placeholder — update when sensor wired
#define PIN_ECHO_BACK   17   // placeholder — update when sensor wired

// --- Vibration / Accident sensor (ADC1_CH7 = GPIO35) ---
// VIB_THRESHOLD 4096 = disabled (ADC max is 4095, so never triggers).
// Change to e.g. 2000 when a real sensor is connected.
#define VIB_THRESHOLD           4096
#define VIB_CHECK_INTERVAL_MS   100
#define VIB_DEBOUNCE_COUNT      3

// ============================================================
//  WIFI
// ============================================================
#define STA_SSID        "Mechanical"
#define STA_PASSWORD    "qqqqqqqq"
#define STA_MAX_RETRIES 5

#define AP_SSID         "SolarCleaner"
#define AP_PASSWORD     "12345678"
#define AP_CHANNEL      6
#define AP_MAX_CONN     4

// ============================================================
//  TELEGRAM
// ============================================================
#define TELEGRAM_BOT_TOKEN  "7752156618:AAF38Dz2sef1tTc7Dqo7pkGcCSx0116_qis"
#define TELEGRAM_CHAT_ID    "7984471289"
#define TELEGRAM_API_URL    "https://api.telegram.org/bot"
#define CAR_LOCATION        "28deg28'34.5 N 77deg28'43.7 E"

// ============================================================
//  HTTP
// ============================================================
#define HTTP_SERVER_PORT        80
#define ULTRASONIC_TIMEOUT_US   20000
#define OBSTACLE_DIST_CM        20
#define ULTRASONIC_INTERVAL_MS  80

// ============================================================
//  MOVEMENT TIMING  (200 RPM + 65 mm wheel ~680 mm/s)
// ============================================================
#define MS_PER_CM               15
#define MS_TURN_90              300
#define MS_STABILISE            120

#define DEFAULT_PANEL_WIDTH_CM   100
#define DEFAULT_PANEL_HEIGHT_CM  170
#define CLEANING_PASSES          5