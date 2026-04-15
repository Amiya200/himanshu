/*
 * mpu6050.h  —  MPU-6050 / ICM-20600 driver public API
 *
 * CHANGES:
 *  - accident field removed from mpu_data_t
 *  - mpu_clear_accident() removed
 *  - heading integration and gyro bias calibration retained
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ─── Sensor data snapshot ────────────────────────────────── */
typedef struct {
    float   accel_x;        /* g */
    float   accel_y;        /* g */
    float   accel_z;        /* g */
    float   gyro_x;         /* deg/s */
    float   gyro_y;         /* deg/s */
    float   gyro_z;         /* deg/s, bias-corrected */
    float   heading_deg;    /* integrated yaw, 0–360 */
    float   vib_mag;        /* |a| - 1 g, vibration estimate */
    int     temp_c;         /* die temperature °C */
} mpu_data_t;

/* ─── Public API ──────────────────────────────────────────── */
void       mpu_init(void);
mpu_data_t mpu_get(void);
float      mpu_heading(void);
void       mpu_reset_heading(void);
bool       mpu_is_ready(void);