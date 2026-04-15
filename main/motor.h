/*
 * motor.h  —  L298N motor driver + relay peripherals
 *
 * CHANGES:
 *  - Accident detection removed entirely
 *  - motor_move_blocking() returns bool (false = IR blocked or stopped)
 *  - Added motor_move_blocking_with_gyro() for MPU-assisted straight travel
 */

#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* ─── Direction enum ──────────────────────────────────────── */
typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_DRIFT_LEFT,
    DIR_DRIFT_RIGHT,
} motor_dir_t;

/* ─── Command struct (queue element) ─────────────────────── */
typedef struct {
    motor_dir_t dir;
} motor_cmd_t;

/* ─── Public API ──────────────────────────────────────────── */
void        motor_init(void);
void        motor_send_cmd(motor_dir_t dir);
void        motor_stop_immediate(void);
motor_dir_t motor_get_direction(void);
const char *motor_dir_to_str(motor_dir_t dir);

/* Ramming mode — bypasses IR obstacle blocking */
void motor_set_ramming(bool en);
bool motor_get_ramming(void);

/* IR obstacle flags (set by ir_task) */
void motor_set_obstacles(bool front, bool back);

/* Relay peripherals */
void pump_set(bool on);
bool pump_get(void);
void blower_set(bool on);
bool blower_get(void);

/*
 * Blocking move: drives in direction for duration_ms, polls every 10 ms.
 * Returns true when completed, false if IR blocked the motion early.
 * No accident check — accident system removed.
 */
bool motor_move_blocking(motor_dir_t dir, int duration_ms);

/*
 * Gyro-assisted straight move: drives forward (or backward) for
 * duration_ms while continuously correcting heading using the MPU.
 * target_heading_deg: the yaw angle to maintain (from mpu_heading()).
 * correction_kp: proportional gain for heading error (try 3.0–8.0).
 * Returns true on completion.
 */
bool motor_move_straight(motor_dir_t dir,
                          int         duration_ms,
                          float       target_heading_deg,
                          float       correction_kp);