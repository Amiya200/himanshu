#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/ledc.h"

/* ─── Direction enum ──────────────────────────────────────── */
typedef enum {
    DIR_STOP        = 0,
    DIR_FORWARD     = 1,
    DIR_BACKWARD    = 2,
    DIR_LEFT        = 3,   /* point-turn left  */
    DIR_RIGHT       = 4,   /* point-turn right */
    DIR_DRIFT_LEFT  = 5,   /* gentle left correction */
    DIR_DRIFT_RIGHT = 6,   /* gentle right correction */
} motor_dir_t;

/* ─── Speed profile ───────────────────────────────────────── */
typedef struct {
    uint8_t left;    /* 0–255 */
    uint8_t right;   /* 0–255 */
} motor_speed_t;

/* ─── Init / control ──────────────────────────────────────── */
void motor_init(void);
void motor_send_cmd(motor_dir_t dir);
void motor_send_cmd_speed(motor_dir_t dir, uint8_t left_spd, uint8_t right_spd);
void motor_stop_immediate(void);

/* Speed profile helpers */
void motor_set_speed(uint8_t spd);               /* same for both channels */
void motor_set_speed_lr(uint8_t left, uint8_t right);
uint8_t motor_get_speed_left(void);
uint8_t motor_get_speed_right(void);

/* ─── Status ──────────────────────────────────────────────── */
motor_dir_t  motor_get_direction(void);
const char  *motor_dir_to_str(motor_dir_t dir);
bool         motor_get_ramming(void);
void         motor_set_ramming(bool en);

/* ─── Obstacle feedback (from ir_sensor) ─────────────────── */
void motor_set_obstacles(bool front, bool back);

/* ─── Blocking moves ─────────────────────────────────────── */
bool motor_move_blocking(motor_dir_t dir, int duration_ms);
bool motor_move_straight(motor_dir_t dir, int duration_ms,
                          float target_heading_deg, float kp);

/* ─── Relay peripherals ───────────────────────────────────── */
void pump_set(bool on);
bool pump_get(void);
void blower_set(bool on);
bool blower_get(void);