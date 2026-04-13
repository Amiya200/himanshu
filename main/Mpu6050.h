#pragma once

#include <stdbool.h>

/* ============================================================
 *  mpu6050.h  —  MPU-6050 accelerometer / gyroscope API
 * ============================================================ */

typedef struct {
    float accel_x;      /* g */
    float accel_y;
    float accel_z;
    float gyro_x;       /* deg/s */
    float gyro_y;
    float gyro_z;
    float heading_deg;  /* integrated yaw, 0–360 */
    float vib_mag;      /* |accel| deviation from 1 g */
    bool  accident;     /* true when vib exceeded threshold N times */
    int   temp_c;       /* die temperature, °C */
} mpu_data_t;

/* ── Init ─────────────────────────────────────────────────── */
void       mpu_init(void);

/* ── Data ─────────────────────────────────────────────────── */
mpu_data_t mpu_get(void);          /* thread-safe snapshot */
float      mpu_heading(void);      /* current integrated heading */
bool       mpu_is_ready(void);     /* true after successful init */

/* ── Control ──────────────────────────────────────────────── */
void       mpu_reset_heading(void);
void       mpu_clear_accident(void);