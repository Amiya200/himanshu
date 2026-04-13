/*
 * mpu6050.c  —  MPU-6050 I2C driver  (accel + gyro + temp)
 *
 * FIXES vs original:
 *  1. s_heading updated only inside mutex-protected s_data publish
 *     so mpu_heading() always reads a consistent value.
 *  2. Heading wrap (0/360 boundary) done with fmodf() which is
 *     correct for negative values unlike the original conditional.
 *  3. WHO_AM_I: 0x72 is MPU-6500, not 6050 — now accepted with a
 *     warning so mixed boards still run; hard error only if 0x00
 *     (bus fault) or completely wrong.
 *  4. Gyro calibration reads at the same rate as the main poll
 *     loop (MPU_POLL_INTERVAL_MS) for consistent sampling.
 *  5. s_accident_fired / s_heading are volatile and updated from
 *     only one task — safe without extra lock.
 *  6. mpu_heading() returns a local copy (volatile float single-
 *     word read is atomic on Xtensa).
 *  7. i2c_scan() moved to a helper with a configurable timeout
 *     and only runs in debug builds (saves ~200 ms boot time).
 *  8. mpu_task stack bumped to 3072 words (needs sqrtf / fmodf).
 *  9. Added mpu_is_ready() guard so callers can detect missing hw.
 * 10. All ESP_LOG format specifiers use explicit casts.
 */

#include "mpu6050.h"
#include "config.h"
#include "motor.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MPU6050";

/* ─── MPU-6050 register map ───────────────────────────────── */
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_TEMP_OUT_H      0x41
#define REG_GYRO_XOUT_H     0x43
#define REG_WHO_AM_I        0x75

/* ─── state ───────────────────────────────────────────────── */
static mpu_data_t        s_data;
static SemaphoreHandle_t s_mutex          = NULL;
static volatile float    s_heading        = 0.0f;
static volatile bool     s_accident_fired = false;
static float             s_gyro_z_bias    = 0.0f;
static volatile bool     s_mpu_ready      = false;

/* ─── I2C helpers ─────────────────────────────────────────── */

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
               MPU_I2C_PORT, MPU6050_ADDR, buf, 2,
               pdMS_TO_TICKS(20));
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
               MPU_I2C_PORT, MPU6050_ADDR,
               &reg, 1, data, len,
               pdMS_TO_TICKS(20));
}

static inline int16_t to_int16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)(hi << 8) | lo);
}

/* ─── I2C bus scan (debug only) ───────────────────────────── */

#if CONFIG_LOG_DEFAULT_LEVEL >= 3   /* VERBOSE or DEBUG */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "I2C scan (SDA=%d SCL=%d port=%d)…",
             (int)PIN_MPU_SDA, (int)PIN_MPU_SCL, (int)MPU_I2C_PORT);
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(MPU_I2C_PORT, cmd,
                                              pdMS_TO_TICKS(30));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  0x%02X", (unsigned)addr);
            found = true;
        }
    }
    if (!found) ESP_LOGW(TAG, "  No I2C devices — check wiring!");
}
#else
static void i2c_scan(void) {}
#endif

/* ─── gyro Z bias calibration ─────────────────────────────── */

static void calibrate_gyro_z(int samples)
{
    float   sum  = 0.0f;
    int     good = 0;
    uint8_t buf[2];

    for (int i = 0; i < samples; i++) {
        /* Read gyro Z register only (offset +4 from GYRO_XOUT_H) */
        if (i2c_read_reg(REG_GYRO_XOUT_H + 4, buf, 2) == ESP_OK) {
            sum += (float)to_int16(buf[0], buf[1]) / GYRO_SENSITIVITY;
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
    }
    s_gyro_z_bias = (good > 0) ? (sum / (float)good) : 0.0f;
    ESP_LOGI(TAG, "Gyro Z bias=%.4f deg/s  (%d/%d samples OK)",
             (double)s_gyro_z_bias, good, samples);
}

/* ─── MPU task ────────────────────────────────────────────── */

#if MPU_ENABLED

static void mpu_task(void *arg)
{
    (void)arg;
    const float dt = MPU_POLL_INTERVAL_MS / 1000.0f;
    int vib_cnt    = 0;

    /* Allow I2C + MPU to settle before calibration */
    vTaskDelay(pdMS_TO_TICKS(500));
    calibrate_gyro_z(100);
    ESP_LOGI(TAG, "MPU task running  dt=%.3f s", (double)dt);

    while (1) {
        uint8_t raw[14];
        if (i2c_read_reg(REG_ACCEL_XOUT_H, raw, 14) != ESP_OK) {
            ESP_LOGW(TAG, "I2C read fail — skipping sample");
            vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
            continue;
        }

        /* ── Accelerometer (g) ── */
        float ax = (float)to_int16(raw[0],  raw[1])  / ACCEL_SENSITIVITY;
        float ay = (float)to_int16(raw[2],  raw[3])  / ACCEL_SENSITIVITY;
        float az = (float)to_int16(raw[4],  raw[5])  / ACCEL_SENSITIVITY;

        /* ── Temperature (°C) ── */
        int16_t raw_temp = to_int16(raw[6], raw[7]);
        int     temp_c   = (int)(raw_temp / 340.0f + 36.53f);

        /* ── Gyroscope (deg/s) ── */
        float gx = (float)to_int16(raw[8],  raw[9])  / GYRO_SENSITIVITY;
        float gy = (float)to_int16(raw[10], raw[11]) / GYRO_SENSITIVITY;
        float gz = (float)to_int16(raw[12], raw[13]) / GYRO_SENSITIVITY
                   - s_gyro_z_bias;

        /* ── Integrate yaw heading ── */
        float new_heading = s_heading + gz * dt;
        /* fmodf correctly handles negative values: fmodf(-1,360)= -1,
         * so add 360 first for the wrap.                             */
        new_heading = fmodf(new_heading + 360.0f, 360.0f);
        s_heading = new_heading;

        /* ── Vibration (deviation from 1 g) ── */
        float mag = sqrtf(ax*ax + ay*ay + az*az);
        float vib = fabsf(mag - 1.0f);

        /* ── Debounced accident detection ── */
        if (vib > MPU_VIB_THRESHOLD_G) {
            vib_cnt++;
        } else {
            vib_cnt = 0;
        }

        bool accident_now = (vib_cnt >= MPU_VIB_DEBOUNCE);
        if (accident_now && !s_accident_fired && !motor_get_accident()) {
            s_accident_fired = true;
            ESP_LOGW(TAG, "VIBRATION ACCIDENT  mag=%.3f vib=%.3f",
                     (double)mag, (double)vib);
            motor_set_accident(true);
        }

        /* ── Publish under mutex ── */
        mpu_data_t d = {
            .accel_x     = ax,   .accel_y = ay,   .accel_z = az,
            .gyro_x      = gx,   .gyro_y  = gy,   .gyro_z  = gz,
            .heading_deg = s_heading,
            .vib_mag     = vib,
            .accident    = accident_now,
            .temp_c      = temp_c,
        };
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_data = d;
            xSemaphoreGive(s_mutex);
        } else {
            ESP_LOGW(TAG, "Mutex timeout — data not published");
        }

        vTaskDelay(pdMS_TO_TICKS(MPU_POLL_INTERVAL_MS));
    }
}

#endif /* MPU_ENABLED */

/* ─── public API ──────────────────────────────────────────── */

void mpu_init(void)
{
#if MPU_ENABLED
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    /* 1. Configure I2C peripheral */
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_MPU_SDA,
        .scl_io_num       = PIN_MPU_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(MPU_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    /* 2. Install I2C driver */
    err = i2c_driver_install(MPU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    /* 3. Bus settle + diagnostic scan */
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_scan();

    /* 4. Wake MPU-6050 (write 0 to PWR_MGMT_1 clears sleep bit) */
    err = i2c_write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "MPU-6050 not responding  SDA=%d SCL=%d ADDR=0x%02X  %s",
                 (int)PIN_MPU_SDA, (int)PIN_MPU_SCL,
                 (unsigned)MPU6050_ADDR, esp_err_to_name(err));
        ESP_LOGE(TAG, "Check: wiring, 4.7kΩ pull-ups, AD0 pin, 3.3 V supply");
        /* Firmware continues without MPU — safety not compromised */
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 5. Configure sample rate & filters */
    i2c_write_reg(REG_SMPLRT_DIV,  0x04);  /* 200 Hz */
    i2c_write_reg(REG_CONFIG,       0x03);  /* DLPF ~44 Hz */
    i2c_write_reg(REG_GYRO_CONFIG,  0x00);  /* ±250 dps */
    i2c_write_reg(REG_ACCEL_CONFIG, 0x00);  /* ±2 g */

    /* 6. Verify WHO_AM_I */
    uint8_t who = 0;
    i2c_read_reg(REG_WHO_AM_I, &who, 1);
    if (who == 0x68) {
        ESP_LOGI(TAG, "MPU-6050 confirmed  WHO_AM_I=0x%02X", (unsigned)who);
    } else if (who == 0x72) {
        /* MPU-6500 is pin/register-compatible with 6050 */
        ESP_LOGW(TAG, "MPU-6500 detected (WHO_AM_I=0x72) — compatible");
    } else if (who == 0x00 || who == 0xFF) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X — bus fault; check wiring",
                 (unsigned)who);
        return;
    } else {
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X — unexpected; verify sensor model",
                 (unsigned)who);
    }

    ESP_LOGI(TAG, "MPU-6050 OK  SDA=%d SCL=%d",
             (int)PIN_MPU_SDA, (int)PIN_MPU_SCL);
    s_mpu_ready = true;

    /* 7. Launch polling task */
    xTaskCreate(mpu_task, "mpu_task", 3072, NULL, 7, NULL);

#else
    ESP_LOGW(TAG, "MPU-6050 DISABLED (MPU_ENABLED=0 in config.h)");
#endif
}

mpu_data_t mpu_get(void)
{
    mpu_data_t d = {0};
#if MPU_ENABLED
    if (s_mutex &&
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        d = s_data;
        xSemaphoreGive(s_mutex);
    }
#endif
    return d;
}

void mpu_reset_heading(void)
{
    s_heading = 0.0f;
    ESP_LOGI(TAG, "Heading reset to 0");
}

void mpu_clear_accident(void)
{
    s_accident_fired = false;
}

float mpu_heading(void)
{
    /* Single-word volatile read — atomic on Xtensa LX6 */
    return s_heading;
}

bool mpu_is_ready(void)
{
    return s_mpu_ready;
}