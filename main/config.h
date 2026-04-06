#pragma once

// ================== WiFi AP CONFIG ==================
#define AP_SSID         "CarController"
#define AP_PASSWORD     "12345678"
#define AP_CHANNEL      1
#define AP_MAX_CONN     4

// ================== WiFi STA CONFIG ==================
#define STA_SSID        "Mechanical"
#define STA_PASSWORD    "qqqqqqqq"
#define STA_MAX_RETRIES 10

// ================== TELEGRAM ==================
#define TELEGRAM_BOT_TOKEN  "7752156618:AAF38Dz2sef1tTc7Dqo7pkGcCSx0116_qis"
#define TELEGRAM_CHAT_ID    "7984471289"
#define TELEGRAM_API_URL    "https://api.telegram.org/bot"
#define CAR_LOCATION        "28deg28'34.5 N 77deg28'43.7 E"

// ================== MOTOR PINS (L298N) ==================
#define PIN_ENA     18
#define PIN_ENB     19
#define PIN_IN1     23
#define PIN_IN2     22
#define PIN_IN3     21
#define PIN_IN4     5

// ================== OTHER PINS ==================
#define PIN_TAIL_LIGHT  2
#define PIN_VIB_SENSOR  35   // ADC1_CH7

// ================== ULTRASONIC PINS ==================
#define PIN_TRIG_FRONT  13
#define PIN_ECHO_FRONT  12
#define PIN_TRIG_BACK   14
#define PIN_ECHO_BACK   27

// ================== THRESHOLDS & TIMING ==================
#define OBSTACLE_DIST_CM        20
#define VIB_THRESHOLD           4000
#define VIB_CHECK_INTERVAL_MS   500
#define ULTRASONIC_INTERVAL_MS  80

// Ultrasonic timeout: ~25ms = ~430cm max range
#define ULTRASONIC_TIMEOUT_US   25000

// HTTP server
#define HTTP_SERVER_PORT        80

// Command queue depth
#define MOTOR_CMD_QUEUE_DEPTH   8