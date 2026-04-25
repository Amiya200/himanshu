#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SERVO_POS_PARK = 0,
    SERVO_POS_DEPLOY,
    SERVO_POS_BOUNDARY
} servo_pos_t;

void servo_init(void);
void servo_set_angle(uint8_t angle);
void servo_set_position(servo_pos_t pos);

uint8_t servo_get_angle(void);
servo_pos_t servo_get_position(void);
bool servo_is_enabled(void);

void servo_park(void);
void servo_deploy(void);
void servo_boundary(void);