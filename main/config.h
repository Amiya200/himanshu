// #pragma once

// // ================== WiFi AP CONFIG ==================
// #define AP_SSID         "CarController"
// #define AP_PASSWORD     "12345678"
// #define AP_CHANNEL      1
// #define AP_MAX_CONN     4

// // ================== WiFi STA CONFIG ==================
// #define STA_SSID        "Mechanical"
// #define STA_PASSWORD    "qqqqqqqq"
// #define STA_MAX_RETRIES 10

#pragma once

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// --- L298N Motor Driver (Drive Motors) ---
#define PIN_IN1     25
#define PIN_IN2     26
#define PIN_IN3     27
#define PIN_IN4     14
#define PIN_ENA     32
#define PIN_ENB     33

// --- Ultrasonic Sensors ---
#define PIN_TRIG_FRONT  5
#define PIN_ECHO_FRONT  18
#define PIN_TRIG_BACK   19
#define PIN_ECHO_BACK   21

// --- Lights ---
#define PIN_TAIL_LIGHT  2

// --- Relay-controlled peripherals ---
// Active-LOW relay modules (LOW = ON, HIGH = OFF)
#define PIN_RELAY_PUMP      12   // Water pump relay
#define PIN_RELAY_BLOWER    13   // Air blower motor relay

// --- Vibration / Accident sensor ---
// ADC1_CHANNEL_7 = GPIO35
#define VIB_THRESHOLD          2000
#define VIB_CHECK_INTERVAL_MS  100

// ============================================================
//  WIFI
// ============================================================
#define STA_SSID        "Mechanical"
#define STA_PASSWORD    "qqqqqqqq"
#define STA_MAX_RETRIES 5

#define AP_SSID         "CarController"
#define AP_PASSWORD     "12345678"
#define AP_CHANNEL      6
#define AP_MAX_CONN     4

// // ================== TELEGRAM ==================
#define TELEGRAM_BOT_TOKEN  "7752156618:AAF38Dz2sef1tTc7Dqo7pkGcCSx0116_qis"
#define TELEGRAM_CHAT_ID    "7984471289"
#define TELEGRAM_API_URL    "https://api.telegram.org/bot"
#define CAR_LOCATION        "28deg28'34.5 N 77deg28'43.7 E"
// ============================================================
//  HTTP
// ============================================================
#define HTTP_SERVER_PORT 80
#define ULTRASONIC_TIMEOUT_US     20000   // faster response
#define OBSTACLE_DIST_CM          20      // tighter safety
#define ULTRASONIC_INTERVAL_MS    80      // smoother reaction
// ============================================================
//  TELEGRAM
// ============================================================
// #define CAR_LOCATION    "Rooftop Solar Panel Cleaner"

// ============================================================
//  MOVEMENT TIMING  (calibrated for 200 RPM + 65 mm wheel)
//
//  Wheel diameter  : 65 mm  (standard yellow robot wheel)
//  Circumference   : π × 65 ≈ 204 mm
//  Speed @ 200 RPM : 200 × 204 mm/min = 40,800 mm/min = 680 mm/s
//  → 1 m  ≈ 1470 ms
//  → 1 cm ≈  14.7 ms  (use 15 ms for safety margin)
//
//  Turning (differential): robot width ~15 cm, track width ~13 cm
//  Quarter-turn arc ≈ 0.5 × π × 13 cm ≈ 20 cm per wheel
//  → 90° turn ≈  300 ms  (tune on hardware)
//  → Lane shift (10 cm) ≈ 150 ms
// ============================================================

// Time (ms) to travel exactly 1 cm at full speed
#define MS_PER_CM           15

// Time (ms) for a 90-degree point turn
#define MS_TURN_90          300

// Small stabilisation pause after each segment
#define MS_STABILISE        120

// Default solar panel width used for auto clean (cm) – overridden by grid
#define DEFAULT_PANEL_WIDTH_CM   100
#define DEFAULT_PANEL_HEIGHT_CM  170

// Number of cleaning passes per panel row
#define CLEANING_PASSES     5