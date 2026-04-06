#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize ultrasonic GPIO pins and periodic timer task.
 */
void ultrasonic_init(void);

/**
 * @brief Get latest measured distances (in cm).
 */
void ultrasonic_get_distances(long *front_cm, long *back_cm);

/**
 * @brief Get blocked status.
 */
bool ultrasonic_is_front_blocked(void);
bool ultrasonic_is_back_blocked(void);