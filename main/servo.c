#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define SERVO_GPIO 18
#define SERVO_MIN_PULSEWIDTH 500   // us
#define SERVO_MAX_PULSEWIDTH 2500  // us
#define SERVO_MAX_DEGREE 180

static const char *TAG = "SERVO";

static uint8_t current_angle = 90;
static servo_pos_t current_pos = SERVO_POS_PARK;
static bool enabled = false;

/* ─── PWM INIT ───────────────────────────── */

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ch);

    enabled = true;
    servo_set_angle(90);

    ESP_LOGI(TAG, "Servo initialized");
}

/* ─── CORE FUNCTIONS ───────────────────── */

static uint32_t angle_to_duty(uint8_t angle)
{
    uint32_t pulse = SERVO_MIN_PULSEWIDTH +
                     ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE;

    return (pulse * (1 << 16)) / 20000; // 20ms period
}

void servo_set_angle(uint8_t angle)
{
    if (!enabled) return;

    if (angle > 180) angle = 180;

    uint32_t duty = angle_to_duty(angle);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    current_angle = angle;
}

/* ─── POSITION CONTROL ─────────────────── */

void servo_set_position(servo_pos_t pos)
{
    current_pos = pos;

    switch (pos)
    {
        case SERVO_POS_PARK:
            servo_set_angle(20);
            break;

        case SERVO_POS_DEPLOY:
            servo_set_angle(90);
            break;

        case SERVO_POS_BOUNDARY:
            servo_set_angle(140);
            break;
    }
}

/* ─── HELPERS ─────────────────────────── */

uint8_t servo_get_angle(void)
{
    return current_angle;
}

servo_pos_t servo_get_position(void)
{
    return current_pos;
}

bool servo_is_enabled(void)
{
    return enabled;
}

/* ─── SHORTCUT FUNCTIONS ─────────────── */

void servo_park(void)
{
    servo_set_position(SERVO_POS_PARK);
}

void servo_deploy(void)
{
    servo_set_position(SERVO_POS_DEPLOY);
}

void servo_boundary(void)
{
    servo_set_position(SERVO_POS_BOUNDARY);
}