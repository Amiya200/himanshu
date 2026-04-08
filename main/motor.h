// #pragma once
// #include <stdbool.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"

// // ================== DIRECTION ENUM ==================
// typedef enum {
//     DIR_STOP = 0,
//     DIR_FORWARD,
//     DIR_BACKWARD,
//     DIR_LEFT,
//     DIR_RIGHT,
//     DIR_DRIFT_LEFT,
//     DIR_DRIFT_RIGHT,
//     DIR_COUNT
// } motor_dir_t;

// // ================== MOTOR COMMAND ==================
// typedef struct {
//     motor_dir_t dir;
// } motor_cmd_t;

// // ================== PUBLIC API ==================

// /**
//  * @brief  Initialize GPIO pins for L298N motor driver.
//  *         Creates the motor command queue and task.
//  */
// void motor_init(void);

// /**
//  * @brief  Enqueue a move command (non-blocking).
//  *         The motor task processes commands one at a time, so rapid
//  *         commands don't stack up — only the *latest* pending command
//  *         is kept (queue depth 1, overwrite mode).
//  */
// void motor_send_cmd(motor_dir_t dir);

// /**
//  * @brief  Immediately stop — called from ISR-safe context too.
//  */
// void motor_stop_immediate(void);

// /**
//  * @brief  Get the direction currently being executed.
//  */
// motor_dir_t motor_get_direction(void);

// /**
//  * @brief  Convert direction enum to human-readable string.
//  */
// const char *motor_dir_to_str(motor_dir_t dir);

// /**
//  * @brief  Enable / disable ramming mode (bypass obstacle block).
//  */
// void motor_set_ramming(bool en);
// bool motor_get_ramming(void);

// /**
//  * @brief  Set accident flag — blocks all movement except explicit stop.
//  */
// void motor_set_accident(bool en);
// bool motor_get_accident(void);
// void motor_move_blocking(motor_dir_t dir, int duration_ms);
// /**
//  * @brief  Update obstacle status from ultrasonic module each cycle.
//  */
// void motor_set_obstacles(bool front, bool back);



#pragma once
#include <stdbool.h>

typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_DRIFT_LEFT,
    DIR_DRIFT_RIGHT,
} motor_dir_t;

typedef struct {
    motor_dir_t dir;
} motor_cmd_t;

// ---- Drive motor API ----
void          motor_init(void);
void          motor_send_cmd(motor_dir_t dir);
void          motor_stop_immediate(void);
motor_dir_t   motor_get_direction(void);
const char   *motor_dir_to_str(motor_dir_t dir);

void  motor_set_ramming(bool en);
bool  motor_get_ramming(void);
void  motor_set_accident(bool en);
bool  motor_get_accident(void);
void  motor_set_obstacles(bool front, bool back);

// Blocking move for auto mode (sends cmd, waits duration_ms, then stops)
void  motor_move_blocking(motor_dir_t dir, int duration_ms);

// ---- Relay-controlled peripherals ----
void  pump_set(bool on);
bool  pump_get(void);
void  blower_set(bool on);
bool  blower_get(void);