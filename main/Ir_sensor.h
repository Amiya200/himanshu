/*
 * ir_sensor.h  —  4-channel IR obstacle sensor public API
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool fl;             /* front-left  channel blocked */
    bool fr;             /* front-right channel blocked */
    bool bl;             /* back-left   channel blocked */
    bool br;             /* back-right  channel blocked */
    bool front_blocked;  /* fl || fr */
    bool back_blocked;   /* bl || br */
} ir_status_t;

void        ir_init(void);
bool        ir_front_blocked(void);
bool        ir_back_blocked(void);
bool        ir_front_left(void);
bool        ir_front_right(void);
bool        ir_back_left(void);
bool        ir_back_right(void);
ir_status_t ir_get_status(void);
void        ir_print_status(void);