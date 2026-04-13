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

typedef struct { motor_dir_t dir; } motor_cmd_t;

void        motor_init(void);
void        motor_send_cmd(motor_dir_t dir);
void        motor_stop_immediate(void);
bool        motor_move_blocking(motor_dir_t dir, int duration_ms);

motor_dir_t motor_get_direction(void);
const char *motor_dir_to_str(motor_dir_t dir);

void        motor_set_accident(bool en);
bool        motor_get_accident(void);
void        motor_set_ramming(bool en);
bool        motor_get_ramming(void);

/* Called by ir_task only when IR_ENABLED == 1 */
void        motor_set_obstacles(bool front, bool back);

void        pump_set(bool on);
bool        pump_get(void);
void        blower_set(bool on);
bool        blower_get(void);