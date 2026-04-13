#pragma once

#include <stdbool.h>

/* ============================================================
 *  ir_sensor.h  —  4-channel IR obstacle sensor API
 * ============================================================ */

typedef struct {
    bool fl;            /* front-left  raw */
    bool fr;            /* front-right raw */
    bool bl;            /* back-left   raw */
    bool br;            /* back-right  raw */
    bool front_blocked; /* debounced composite */
    bool back_blocked;  /* debounced composite */
} ir_status_t;

void        ir_init(void);

bool        ir_front_blocked(void);
bool        ir_back_blocked(void);
bool        ir_front_left(void);
bool        ir_front_right(void);
bool        ir_back_left(void);
bool        ir_back_right(void);

ir_status_t ir_get_status(void);