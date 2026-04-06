#include "ultrasonic.h"
#include "config.h"
#include "motor.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

static const char *TAG = "ULTRASONIC";

static volatile long s_front_dist = 999;
static volatile long s_back_dist  = 999;
static volatile bool s_front_blocked = false;
static volatile bool s_back_blocked  = false;

// ================== LOW-LEVEL READ ==================

static long read_sensor(int trig, int echo)
{
    // Send 10µs pulse
    gpio_set_level(trig, 0);
    ets_delay_us(2);
    gpio_set_level(trig, 1);
    ets_delay_us(10);
    gpio_set_level(trig, 0);

    // Wait for echo HIGH with timeout
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if (esp_timer_get_time() - start > ULTRASONIC_TIMEOUT_US) return 999;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - echo_start > ULTRASONIC_TIMEOUT_US) return 999;
    }
    int64_t echo_end = esp_timer_get_time();

    long duration_us = (long)(echo_end - echo_start);
    long dist_cm = duration_us / 58;
    return (dist_cm < 1 || dist_cm > 400) ? 999 : dist_cm;
}

// ================== PERIODIC TASK ==================

static void ultrasonic_task(void *arg)
{
    while (1) {
        s_front_dist = read_sensor(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
        vTaskDelay(pdMS_TO_TICKS(10));  // small gap between sensors
        s_back_dist  = read_sensor(PIN_TRIG_BACK,  PIN_ECHO_BACK);

        s_front_blocked = (s_front_dist < OBSTACLE_DIST_CM);
        s_back_blocked  = (s_back_dist  < OBSTACLE_DIST_CM);

        // Notify motor module so it can stop if needed
        motor_set_obstacles(s_front_blocked, s_back_blocked);

        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
    }
}

// ================== PUBLIC API ==================

void ultrasonic_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_TRIG_FRONT) | (1ULL << PIN_TRIG_BACK),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    cfg.pin_bit_mask = (1ULL << PIN_ECHO_FRONT) | (1ULL << PIN_ECHO_BACK);
    cfg.mode         = GPIO_MODE_INPUT;
    gpio_config(&cfg);

    xTaskCreatePinnedToCore(
        ultrasonic_task,
        "ultrasonic_task",
        2048,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Ultrasonic sensors initialized");
}

void ultrasonic_get_distances(long *front_cm, long *back_cm)
{
    *front_cm = s_front_dist;
    *back_cm  = s_back_dist;
}

bool ultrasonic_is_front_blocked(void)
{
    return s_front_blocked;
}

bool ultrasonic_is_back_blocked(void)
{
    return s_back_blocked;
}