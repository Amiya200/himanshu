#pragma once
/*
 * ir_sensor.h  —  4-channel IR obstacle sensor public API
 */

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool fl, fr, bl, br;
    bool front_blocked;
    bool back_blocked;
} ir_status_t;

void        ir_init(void);

bool        ir_front_blocked(void);
bool        ir_back_blocked(void);
bool        ir_front_left(void);
bool        ir_front_right(void);
bool        ir_back_left(void);
bool        ir_back_right(void);

ir_status_t ir_get_status(void);

/* Print current IR sensor state to serial (ESP_LOGI) */
void        ir_print_status(void);